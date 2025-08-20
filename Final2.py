# show_route_all_debug.py
# -*- coding: utf-8 -*-

import os, sys, glob, csv, argparse, xml.etree.ElementTree as ET
from collections import defaultdict
import warnings
warnings.filterwarnings("ignore", message=".*deprecated function getAllProgramLogics.*")

# ---- SUMO bootstrap ----
try:
    from sumolib import checkBinary
except Exception:
    raise RuntimeError("Install: pip install sumolib traci networkx")

if "SUMO_HOME" in os.environ:
    TOOLS = os.path.join(os.environ["SUMO_HOME"], "tools")
    if os.path.isdir(TOOLS):
        sys.path.append(TOOLS)

import traci, sumolib, networkx as nx

# ---- Params (compact & map-agnostic) ----
REROUTE_PERIOD = 10.0           # s
NEAR_JUNCTION_SKIP_DIST = 120.0 # m
DENSITY_ALPHA = 2.0
MIN_SPEED_FRACTION = 0.3
OCCUPANCY_FREE_THRESH = 0.05
SMOOTHING_MIN_SPEED = 1.5       # m/s fallback threshold

# TLS lookahead + green bias (ใหม่)
TLS_LOOKAHEAD_LIMIT = 300.0     # s
GREEN_BIAS = 1.0                # -3..+3 (บวก = เอนเอียงไปทางที่ "อาจทันเขียว")

INVALID_THRESHOLD = 2
COOLDOWN_SECONDS = 45.0
STUCK_EXCLUDE_WAIT = 120.0
PROGRESS_EVERY_SIMSEC = 30

# ---- cfg helpers ----
def find_cfg():
    for f in glob.glob("*.sumocfg"): return f
    raise FileNotFoundError("No .sumocfg found in current folder")

def read_net_from_cfg(cfg_path):
    try:
        root = ET.parse(cfg_path).getroot()
        tag = root.find(".//input/net-file")
        if tag is not None and "value" in tag.attrib:
            return os.path.join(os.path.dirname(cfg_path), tag.attrib["value"])
    except ET.ParseError:
        pass
    nets = glob.glob("*.net.xml")
    return nets[0] if nets else None

def read_route_files_from_cfg(cfg_path):
    try:
        root = ET.parse(cfg_path).getroot()
        tag = root.find(".//input/route-files")
        if tag is None or "value" not in tag.attrib: return []
        base = os.path.dirname(cfg_path)
        return [os.path.join(base, v) for v in tag.attrib["value"].split()]
    except ET.ParseError:
        return []

# ---- tiny helpers ----
def _safe_iter_out_lanes(lane_id):
    try:
        links = traci.lane.getLinks(lane_id)
    except traci.TraCIException:
        return []
    outs = []
    for lk in links:
        try:
            to_lane = lk[0] if isinstance(lk, (list, tuple)) else getattr(lk, "toLane", None)
        except Exception:
            to_lane = None
        if to_lane: outs.append(to_lane)
    return outs

def _lane_allows_class(lane_id, vehicle_class):
    if not vehicle_class: return True
    try:
        allowed = set(traci.lane.getAllowed(lane_id))
        disallowed = set(traci.lane.getDisallowed(lane_id))
        if vehicle_class in disallowed: return False
        if allowed and (vehicle_class not in allowed): return False
        return True
    except traci.TraCIException:
        return True

def get_vehicle_class(veh_id):
    try:
        vtype = traci.vehicle.getTypeID(veh_id)
        return traci.vehicletype.getVehicleClass(vtype)
    except traci.TraCIException:
        return None

def next_edges_allowed_from_current_lane(veh_id):
    try:
        lane_id = traci.vehicle.getLaneID(veh_id)
        return {ln.split("_")[0] for ln in _safe_iter_out_lanes(lane_id)}
    except traci.TraCIException:
        return set()

def lane_has_link_to_edge(lane_id, edge_id):
    for toLane in _safe_iter_out_lanes(lane_id):
        if toLane.split("_")[0] == edge_id: return True
    return False

def is_uturn_pair(a, b):
    return (a == "-" + b) or (b == "-" + a)

def has_edge_connection_any_lane(a, b):
    try:
        for lane_id in traci.edge.getLaneIDs(a):
            if lane_has_link_to_edge(lane_id, b):
                return True
    except Exception:
        pass
    return False

# ---- TLS utilities (compact) ----
def cache_tls_definitions():
    tls_defs = {}
    for tls_id in traci.trafficlight.getIDList():
        progs = traci.trafficlight.getCompleteRedYellowGreenDefinition(tls_id)
        if not progs: continue
        phases = [(p.state, float(p.duration)) for p in progs[0].phases]
        tls_defs[tls_id] = {"phases": phases, "cycle": sum(d for _, d in phases)}
    return tls_defs

def build_tls_linkmap():
    linkmap = {}
    for tls_id in traci.trafficlight.getIDList():
        mapping = {}
        for sig_idx, links in enumerate(traci.trafficlight.getControlledLinks(tls_id)):
            for triple in links:
                try:
                    in_lane, out_lane, _ = triple
                except Exception:
                    try:
                        in_lane, out_lane = triple[0], triple[1]
                    except Exception:
                        continue
                mapping[(in_lane.split("_")[0], out_lane.split("_")[0])] = sig_idx
        linkmap[tls_id] = mapping
    return linkmap

def _is_green(state, idx): return idx < len(state) and state[idx] in ("G", "g")

def tls_delay_seconds(tls_id, sig_index, tls_defs):
    defs = tls_defs.get(tls_id); 
    if not defs or defs["cycle"] <= 0: return 0.0
    phases = defs["phases"]
    try:
        cur = traci.trafficlight.getPhase(tls_id)
        remain = traci.trafficlight.getPhaseDuration(tls_id) - traci.trafficlight.getPhaseElapsed(tls_id)
    except Exception:
        return 0.0
    if _is_green(phases[cur][0], sig_index): return 0.0
    delay, scan, scanned = max(0.0, remain), (cur+1)%len(phases), 0.0
    while scanned < TLS_LOOKAHEAD_LIMIT:
        st, dur = phases[scan]
        if _is_green(st, sig_index): return max(0.0, delay)
        delay += dur; scanned += dur; scan = (scan+1) % len(phases)
    return 0.0

def expected_tls_delay_for_movement(in_edge, out_edge, tls_defs, tls_linkmap):
    for tls_id, mp in tls_linkmap.items():
        idx = mp.get((in_edge, out_edge))
        if idx is not None:
            return tls_delay_seconds(tls_id, idx, tls_defs)
    return 0.0

# ---- (1) Build EDGE graph from TraCI (per vehicle class) ----
def build_edge_graph_from_traci(vehicle_class=None):
    G = nx.DiGraph()
    # nodes
    for eid in traci.edge.getIDList():
        if not eid or eid.startswith(":"): continue
        try:
            length = traci.edge.getLength(eid)
        except Exception:
            length = 1.0
        try:
            speed_limit = traci.edge.getMaxSpeed(eid)
        except Exception:
            speed_limit = 13.9
        G.add_node(eid, length=length, speed_limit=speed_limit)
    # edges
    for lane_id in traci.lane.getIDList():
        try:
            base_edge = lane_id.split("_")[0]
        except Exception:
            continue
        if not base_edge or base_edge.startswith(":") or base_edge not in G: continue
        if not _lane_allows_class(lane_id, vehicle_class): continue
        for toLane in _safe_iter_out_lanes(lane_id):
            to_edge = toLane.split("_")[0]
            if not to_edge or to_edge.startswith(":") or to_edge not in G: continue
            if not _lane_allows_class(toLane, vehicle_class): continue
            G.add_edge(base_edge, to_edge)
    return G

# ---- (2) Weight function (สูตรเดิม + green bias, ตัดส่วนไม่จำเป็น) ----
def movement_weight(in_edge, out_edge, G_edge, tls_defs, tls_linkmap):
    # geometry & speed limit
    nd = G_edge.nodes.get(out_edge, {})
    length = nd.get("length", 1.0)
    speed_limit = nd.get("speed_limit", 13.9)

    # live probes
    try:
        occ = traci.edge.getLastStepOccupancy(out_edge)
    except Exception:
        occ = None
    try:
        mean_speed = traci.edge.getLastStepMeanSpeed(out_edge)
    except Exception:
        mean_speed = None

    # expected speed
    if occ is not None and occ < OCCUPANCY_FREE_THRESH:
        use_speed = max(speed_limit, 0.1)
    elif mean_speed is not None and mean_speed > SMOOTHING_MIN_SPEED:
        use_speed = mean_speed
    else:
        use_speed = max(speed_limit * MIN_SPEED_FRACTION, 0.1)

    base_tt = length / max(use_speed, 0.1)
    dens_mult = 1.0 + DENSITY_ALPHA * float(occ if occ is not None else 0.0)

    # TLS delay + green bias
    tls_del = expected_tls_delay_for_movement(in_edge, out_edge, tls_defs, tls_linkmap)
    green_bonus = -GREEN_BIAS * 0.5 if (0.0 < tls_del < 10.0) else 0.0

    return base_tt * dens_mult + tls_del + green_bonus

# ---- Main ----
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--no-opt", action="store_true", help="Baseline (no reroute)")
    ap.add_argument("--tag", type=str, default="", help="Tag for output filename")
    args = ap.parse_args()

    tag = f" ({args.tag})" if args.tag else ""
    cfg = find_cfg()
    net_path = read_net_from_cfg(cfg)
    if not net_path or not os.path.exists(net_path):
        raise FileNotFoundError("Cannot locate .net.xml")
    for rf in read_route_files_from_cfg(cfg):
        if not os.path.exists(rf):
            raise FileNotFoundError(f"route-file not found: {rf}")

    agg_csv = f"aggregates{tag}.csv"

    sumo = sumolib.checkBinary("sumo")
    cmd = [sumo, "-c", cfg, "--quit-on-end"]
    print("[INFO] Launching:", " ".join(cmd))
    traci.start(cmd)

    try:
        tls_defs = cache_tls_definitions()
        tls_linkmap = build_tls_linkmap()

        last_reroute = defaultdict(lambda: -1e9)
        veh_depart_time = {}
        veh_waiting_time_live = defaultdict(float)
        invalid_count = defaultdict(int)
        cooldown_until = defaultdict(float)

        stuck_excluded = set()
        finished_travel_times, finished_waiting_times = [], []
        arrived_total = 0

        # no teleport
        try:
            traci.simulation.setParameter("", "time-to-teleport", "-1")
            traci.simulation.setParameter("", "time-to-teleport.disconnected", "-1")
        except traci.TraCIException:
            pass

        edge_graph_cache = {}  # key = veh_class or "_ANY_" : G

        last_progress_print = -1
        while traci.simulation.getMinExpectedNumber() > 0:
            traci.simulationStep()
            t = traci.simulation.getTime()

            # live stats + stuck flag
            for vid in traci.vehicle.getIDList():
                if vid not in veh_depart_time: veh_depart_time[vid] = t
                try:
                    veh_waiting_time_live[vid] = traci.vehicle.getAccumulatedWaitingTime(vid)
                    if traci.vehicle.getWaitingTime(vid) >= STUCK_EXCLUDE_WAIT:
                        stuck_excluded.add(vid)
                except traci.TraCIException:
                    pass

            # light TLS refresh
            if int(t) % 60 == 0:
                try:
                    tls_defs = cache_tls_definitions()
                    tls_linkmap = build_tls_linkmap()
                except Exception:
                    pass

            # progress print
            if int(t) // PROGRESS_EVERY_SIMSEC != last_progress_print:
                last_progress_print = int(t) // PROGRESS_EVERY_SIMSEC
                running = len(traci.vehicle.getIDList())
                left = traci.simulation.getMinExpectedNumber()
                excl = len(stuck_excluded)
                n_ok = len(finished_travel_times)
                avg_tt = (sum(finished_travel_times)/n_ok) if n_ok else 0.0
                avg_wt = (sum(finished_waiting_times)/n_ok) if n_ok else 0.0
                print(f"[t={t:.0f}s] running={running} arrived={arrived_total} "
                      f"excluded={excl} left={left} | avgTT={avg_tt:.1f}s avgWT={avg_wt:.1f}s",
                      flush=True)

            # ----- REROUTE (optimized) -----
            if not args.no_opt:
                for vid in traci.vehicle.getIDList():
                    if t < cooldown_until[vid]: continue
                    if t - last_reroute[vid] < REROUTE_PERIOD: continue

                    # skip near junction
                    try:
                        lane_id = traci.vehicle.getLaneID(vid)
                        lane_pos = traci.vehicle.getLanePosition(vid)
                        lane_len = traci.lane.getLength(lane_id)
                        if (lane_len - lane_pos) < NEAR_JUNCTION_SKIP_DIST:
                            continue
                    except traci.TraCIException:
                        pass

                    cur_edge = traci.vehicle.getRoadID(vid)
                    if not cur_edge or cur_edge.startswith(":"): continue
                    try:
                        route = traci.vehicle.getRoute(vid)
                    except traci.TraCIException:
                        continue
                    if not route: continue
                    dest_edge = route[-1]
                    if not dest_edge or dest_edge.startswith(":"): continue

                    # per-class graph
                    veh_class = get_vehicle_class(vid)
                    key = veh_class or "_ANY_"
                    if key not in edge_graph_cache:
                        edge_graph_cache[key] = build_edge_graph_from_traci(veh_class)
                    G = edge_graph_cache[key]
                    if cur_edge not in G or dest_edge not in G:
                        invalid_count[vid] += 1
                        if invalid_count[vid] >= INVALID_THRESHOLD:
                            cooldown_until[vid] = t + COOLDOWN_SECONDS; invalid_count[vid] = 0
                        try: traci.vehicle.rerouteTraveltime(vid)
                        except traci.TraCIException: pass
                        continue

                    # shortest path with simple weight (+green bias)
                    def w(u, v, d): return movement_weight(u, v, G, tls_defs, tls_linkmap)
                    try:
                        edge_path = nx.shortest_path(G, cur_edge, dest_edge, weight=w)
                    except Exception:
                        invalid_count[vid] += 1
                        if invalid_count[vid] >= INVALID_THRESHOLD:
                            cooldown_until[vid] = t + COOLDOWN_SECONDS; invalid_count[vid] = 0
                        try: traci.vehicle.rerouteTraveltime(vid)
                        except traci.TraCIException: pass
                        continue

                    new_route = list(edge_path)

                    # validations (minimal but safe)
                    allowed_from_lane = next_edges_allowed_from_current_lane(vid)
                    if len(new_route) >= 2 and allowed_from_lane and new_route[1] not in allowed_from_lane:
                        invalid_count[vid] += 1
                        if invalid_count[vid] >= INVALID_THRESHOLD:
                            cooldown_until[vid] = t + COOLDOWN_SECONDS; invalid_count[vid] = 0
                            try: traci.vehicle.rerouteTraveltime(vid)
                            except traci.TraCIException: pass
                        continue

                    if len(new_route) >= 2 and is_uturn_pair(new_route[0], new_route[1]):
                        invalid_count[vid] += 1
                        if invalid_count[vid] >= INVALID_THRESHOLD:
                            cooldown_until[vid] = t + COOLDOWN_SECONDS; invalid_count[vid] = 0
                            try: traci.vehicle.rerouteTraveltime(vid)
                            except traci.TraCIException: pass
                        continue

                    if not all(has_edge_connection_any_lane(x, y) for x, y in zip(new_route, new_route[1:])):
                        invalid_count[vid] += 1
                        if invalid_count[vid] >= INVALID_THRESHOLD:
                            cooldown_until[vid] = t + COOLDOWN_SECONDS; invalid_count[vid] = 0
                            try: traci.vehicle.rerouteTraveltime(vid)
                            except traci.TraCIException: pass
                        continue

                    try:
                        traci.vehicle.setRoute(vid, new_route)
                        last_reroute[vid] = t
                        invalid_count[vid] = 0
                    except traci.TraCIException:
                        invalid_count[vid] += 1
                        if invalid_count[vid] >= INVALID_THRESHOLD:
                            cooldown_until[vid] = t + COOLDOWN_SECONDS; invalid_count[vid] = 0
                            try: traci.vehicle.rerouteTraveltime(vid)
                            except traci.TraCIException: pass

            # arrivals
            for vid in traci.simulation.getArrivedIDList():
                arrived_total += 1
                dep = veh_depart_time.get(vid, None); arr = t
                if (vid not in stuck_excluded) and (dep is not None):
                    finished_travel_times.append(arr - dep)
                    finished_waiting_times.append(veh_waiting_time_live.get(vid, 0.0))

        # ---- summary ----
        n = len(finished_travel_times)
        avg_travel = (sum(finished_travel_times)/n) if n else 0.0
        avg_wait   = (sum(finished_waiting_times)/n) if n else 0.0

        write_header = not os.path.exists(agg_csv)
        with open(agg_csv, "a", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            if write_header: w.writerow(["vehicles", "avg_travel_time", "avg_waiting_time"])
            w.writerow([n, f"{avg_travel:.3f}", f"{avg_wait:.3f}"])

        print("\n===== Simulation Summary =====")
        print(f"Vehicles simulated (included in averages): {n}")
        print(f"Vehicles excluded (stuck >= {STUCK_EXCLUDE_WAIT:.0f}s): {len(stuck_excluded)}")
        print(f"Average travel time: {avg_travel:.3f} s")
        print(f"Average waiting time: {avg_wait:.3f} s")
        print(f"(Aggregates written to: {agg_csv})")

    finally:
        try: traci.close(False)
        except Exception: pass

if __name__ == "__main__":
    main()
