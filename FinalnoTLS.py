# show_route_all_debug.py
# -*- coding: utf-8 -*-

import os, sys, glob, csv, time, xml.etree.ElementTree as ET
from collections import defaultdict, deque  # <-- เพิ่ม deque แค่นี้
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

# =========================
# Parameters
# =========================
REROUTE_PERIOD = 15.0
NEAR_JUNCTION_SKIP_DIST = 0.0
DENSITY_ALPHA = 1.5
MIN_SPEED_FRACTION = 0.3
OCCUPANCY_FREE_THRESH = 0.1
SMOOTHING_MIN_SPEED = 1.5

INVALID_THRESHOLD = 2
PROGRESS_EVERY_SIMSEC = 30

NO_OPT = False
TAG = ""

# ---- NEW: บัฟเฟอร์เก็บ N ค่า/edge ----
WINDOW_N = 10
EDGE_BUF = {}  # edge_id -> {"speed": deque(maxlen=WINDOW_N), "occ": deque(maxlen=WINDOW_N)}

# =========================
# Config / file helpers
# =========================
def find_cfg():
    for f in glob.glob("*.sumocfg"):
        return f
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
        if tag is None or "value" not in tag.attrib:
            return []
        base = os.path.dirname(cfg_path)
        return [os.path.join(base, v) for v in tag.attrib["value"].split()]
    except ET.ParseError:
        return []


# =========================
# Lane/edge utils
# =========================
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
        if to_lane:
            outs.append(to_lane)
    return outs

def _lane_allows_class(lane_id, vehicle_class):
    if not vehicle_class:
        return True
    try:
        allowed = set(traci.lane.getAllowed(lane_id))
        disallowed = set(traci.lane.getDisallowed(lane_id))
        if vehicle_class in disallowed:
            return False
        if allowed and (vehicle_class not in allowed):
            return False
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
        if toLane.split("_")[0] == edge_id:
            return True
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


# =========================
# Build EDGE graph
# =========================
def build_edge_graph_from_traci(vehicle_class=None):
    G = nx.DiGraph()
    for eid in traci.edge.getIDList():
        if not eid or eid.startswith(":"):
            continue
        try:
            length = traci.edge.getLength(eid)
        except Exception:
            length = 1.0
        try:
            speed_limit = traci.edge.getMaxSpeed(eid)
        except Exception:
            speed_limit = 13.9
        G.add_node(eid, length=length, speed_limit=speed_limit)
    for lane_id in traci.lane.getIDList():
        base_edge = lane_id.split("_")[0] if "_" in lane_id else lane_id
        if (not base_edge) or base_edge.startswith(":") or (base_edge not in G):
            continue
        if not _lane_allows_class(lane_id, vehicle_class):
            continue
        for toLane in _safe_iter_out_lanes(lane_id):
            to_edge = toLane.split("_")[0]
            if (not to_edge) or to_edge.startswith(":") or (to_edge not in G):
                continue
            if not _lane_allows_class(toLane, vehicle_class):
                continue
            G.add_edge(base_edge, to_edge)
    return G


# =========================
# Cost model (no TLS)
# =========================
def expected_speed(out_edge, speed_limit):
    """เก็บค่า N ค่าล่าสุด (speed/occupancy) ต่อ edge แล้ว 'เฉลี่ย' ใช้ต่อ"""
    # อ่านล่าสุด
    try:
        occ = traci.edge.getLastStepOccupancy(out_edge)
    except Exception:
        occ = 0.0
    try:
        mean_speed = traci.edge.getLastStepMeanSpeed(out_edge)
    except Exception:
        mean_speed = None

    # ปรับค่า fallback ให้ปลอดภัย
    if not isinstance(occ, (int, float)) or occ != occ:  # NaN check
        occ = 0.0
    occ = max(0.0, min(1.0, float(occ)))

    if not isinstance(mean_speed, (int, float)) or mean_speed <= 0:
        mean_speed = speed_limit * MIN_SPEED_FRACTION

    # บัฟเฟอร์ per-edge
    buf = EDGE_BUF.setdefault(out_edge, {
        "speed": deque(maxlen=WINDOW_N),
        "occ":   deque(maxlen=WINDOW_N),
    })
    buf["speed"].append(float(mean_speed))
    buf["occ"].append(float(occ))

    # เฉลี่ยธรรมดา (mean) N ค่าล่าสุด
    sm_speed = sum(buf["speed"]) / len(buf["speed"])
    sm_occ   = sum(buf["occ"]) / len(buf["occ"])

    # ใช้เงื่อนไขเดิม แต่ใช้ค่าที่เฉลี่ยแล้ว
    if sm_occ < OCCUPANCY_FREE_THRESH:
        use_speed = max(speed_limit, 0.1)
    elif sm_speed > SMOOTHING_MIN_SPEED:
        use_speed = sm_speed
    else:
        use_speed = max(speed_limit * MIN_SPEED_FRACTION, 0.1)

    return use_speed, sm_occ



# =========================
# Main
# =========================
def main():
    wall_start = time.perf_counter()  # measure processing time (wall-clock)

    cfg = find_cfg()
    net_path = read_net_from_cfg(cfg)
    if not net_path or not os.path.exists(net_path):
        raise FileNotFoundError("Cannot locate .net.xml")
    for rf in read_route_files_from_cfg(cfg):
        if not os.path.exists(rf):
            raise FileNotFoundError(f"route-file not found: {rf}")

    agg_csv = f"aggregates{TAG}.csv"
    sumo = sumolib.checkBinary("sumo-gui")
    cmd = [sumo, "-c", cfg, "--quit-on-end"]
    print("[INFO] Launching:", " ".join(cmd))
    traci.start(cmd)

    try:
        last_reroute = defaultdict(lambda: -1e9)
        invalid_count = defaultdict(int)

        # Per-vehicle states
        vehicle_states = {}

        # Aggregates
        total_travel_time = 0.0
        total_waiting_time = 0.0
        total_co2_mg = 0.0
        total_fuel_ml = 0.0
        vehicle_count = 0

        edge_graph_cache = {}
        last_progress_bucket = -1

        # simulate
        while traci.simulation.getMinExpectedNumber() > 0:
            traci.simulationStep()
            sim_time = traci.simulation.getTime()

            # step size in seconds (Δt)
            try:
                step_sec = traci.simulation.getDeltaT() / 1000.0
            except Exception:
                step_sec = 1.0

            # 1) accumulate per-step
            active_ids = set(traci.vehicle.getIDList())
            for vid, vs in list(vehicle_states.items()):
                if vid not in active_ids:
                    continue
                try:
                    curr_acc_wait = traci.vehicle.getAccumulatedWaitingTime(vid)
                    delta_wait = max(0.0, curr_acc_wait - vs.get("last_wait_acc", 0.0))
                    vs["waiting_time"] += delta_wait
                    vs["last_wait_acc"] = curr_acc_wait
                    vs["co2_mg"]  += traci.vehicle.getCO2Emission(vid) * step_sec
                    vs["fuel_ml"] += traci.vehicle.getFuelConsumption(vid) * step_sec
                except traci.TraCIException:
                    pass

            # 2) init new vehicles
            for vid in active_ids:
                if vid not in vehicle_states:
                    try:
                        vehicle_states[vid] = {
                            "depart_time": sim_time,
                            "waiting_time": 0.0,
                            "last_wait_acc": traci.vehicle.getAccumulatedWaitingTime(vid),
                            "co2_mg": 0.0,
                            "fuel_ml": 0.0,
                        }
                    except traci.TraCIException:
                        vehicle_states[vid] = {
                            "depart_time": sim_time,
                            "waiting_time": 0.0,
                            "last_wait_acc": 0.0,
                            "co2_mg": 0.0,
                            "fuel_ml": 0.0,
                        }

            # 3) reroute
            if not NO_OPT:
                # ====== [B] PRECOMPUTE EDGE WEIGHTS PER STEP / PER VEHICLE CLASS ======
                def compute_edge_weights_for_class(veh_class_key, G):
                    # เตรียม cost ต่อ out_edge (node v) ครั้งเดียว
                    per_out_edge_cost = {}
                    for v in G.nodes:
                        nd = G.nodes[v]
                        speed_limit = nd.get("speed_limit", 13.9)
                        use_speed, occ = expected_speed(v, speed_limit)
                        base_tt = nd.get("length", 1.0) / max(use_speed, 0.1)
                        cost = base_tt * (1.0 + DENSITY_ALPHA * float(occ))
                        per_out_edge_cost[v] = cost
                    # กำหนด weight ให้ทุกขอบ (u, v)
                    for u, v in G.edges:
                        G.edges[u, v]["w"] = per_out_edge_cost[v]

                # รวบรวมคลาสของรถที่ active ในสเต็ปนี้
                step_classes = set()
                for vid in active_ids:
                    step_classes.add(get_vehicle_class(vid) or "_ANY_")

                # ให้แน่ใจว่ามีกราฟของแต่ละคลาส และคำนวณ weight ล่วงหน้า
                for cls in step_classes:
                    if cls not in edge_graph_cache:
                        # สร้างกราฟตามคลาส (เหมือนเดิม)
                        real_cls = None if cls == "_ANY_" else cls
                        edge_graph_cache[cls] = build_edge_graph_from_traci(real_cls)
                    compute_edge_weights_for_class(cls, edge_graph_cache[cls])
                # ====== [B] END ======

                for vid in active_ids:
                    # เดิม: if sim_time < cooldown_until[vid] or (sim_time - last_reroute[vid] < REROUTE_PERIOD): continue
                    try:
                        lane_id = traci.vehicle.getLaneID(vid)
                        lane_pos = traci.vehicle.getLanePosition(vid)
                        lane_len = traci.lane.getLength(lane_id)
                        if (lane_len - lane_pos) < NEAR_JUNCTION_SKIP_DIST:
                            print("near")
                            continue
                    except traci.TraCIException:
                        pass

                    cur_edge = traci.vehicle.getRoadID(vid)
                    if not cur_edge or cur_edge.startswith(":"):
                        continue
                    try:
                        route = traci.vehicle.getRoute(vid)
                    except traci.TraCIException:
                        continue
                    if not route:
                        continue
                    dest_edge = route[-1]
                    if not dest_edge or dest_edge.startswith(":"):
                        continue

                    veh_class = get_vehicle_class(vid)
                    key = veh_class or "_ANY_"
                    if key not in edge_graph_cache:
                        edge_graph_cache[key] = build_edge_graph_from_traci(veh_class)
                    G = edge_graph_cache[key]
                    if cur_edge not in G or dest_edge not in G:
                        invalid_count[vid] += 1
                        if invalid_count[vid] >= INVALID_THRESHOLD:
                            invalid_count[vid] = 0
                        try:
                            traci.vehicle.rerouteTraveltime(vid)
                        except traci.TraCIException:
                            print("inv")
                            pass
                        continue

                    # === ใช้ weight='w' ที่คำนวณล่วงหน้าแล้ว ===
                    try:
                        edge_path_new = nx.shortest_path(G, cur_edge, dest_edge, weight="w")
                    except Exception:
                        invalid_count[vid] += 1
                        if invalid_count[vid] >= INVALID_THRESHOLD:
                            invalid_count[vid] = 0
                        try:
                            traci.vehicle.rerouteTraveltime(vid)
                        except traci.TraCIException:
                            print("fail reroute")
                            pass
                        continue

                    new_route = list(edge_path_new)

                    # minimal validations
                    allowed_from_lane = next_edges_allowed_from_current_lane(vid)
                    if len(new_route) >= 2 and allowed_from_lane and (new_route[1] not in allowed_from_lane):
                        continue
                    if len(new_route) >= 2 and is_uturn_pair(new_route[0], new_route[1]):
                        continue

                    try:
                        traci.vehicle.setRoute(vid, new_route)
                        last_reroute[vid] = sim_time
                        invalid_count[vid] = 0
                    except traci.TraCIException:
                        continue

            # 4) arrivals → metrics
            for vid in traci.simulation.getArrivedIDList():
                vs = vehicle_states.get(vid)
                if vs is None:
                    continue
                try:
                    travel_time = sim_time - vs.get("depart_time", sim_time)
                    total_travel_time   += travel_time
                    total_waiting_time  += vs.get("waiting_time", 0.0)
                    total_co2_mg        += vs.get("co2_mg", 0.0)
                    total_fuel_ml       += vs.get("fuel_ml", 0.0)
                    vehicle_count       += 1
                except Exception:
                    pass
                finally:
                    vehicle_states.pop(vid, None)

            # 5) progress
            bucket = int(sim_time) // PROGRESS_EVERY_SIMSEC
            if bucket != last_progress_bucket:
                last_progress_bucket = bucket
                print(f"{vehicle_count}", flush=True)

        # ---- Summary ----
        avg_travel = (total_travel_time / vehicle_count) if vehicle_count else 0.0
        avg_wait   = (total_waiting_time / vehicle_count) if vehicle_count else 0.0
        avg_co2_mg = (total_co2_mg / vehicle_count) if vehicle_count else 0.0
        avg_fuel_ml= (total_fuel_ml / vehicle_count) if vehicle_count else 0.0

        write_header = not os.path.exists(agg_csv)
        with open(agg_csv, "a", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            if write_header:
                w.writerow(["vehicles", "avg_travel_time_s", "avg_waiting_time_s", "avg_CO2_mg", "avg_fuel_ml"])
            w.writerow([vehicle_count, f"{avg_travel:.3f}", f"{avg_wait:.3f}", f"{avg_co2_mg:.3f}", f"{avg_fuel_ml:.3f}"])

        wall_end = time.perf_counter()
        processing_time = wall_end - wall_start

        print("\n===== Simulation Summary =====")
        print(f"Vehicles simulated (included):  {vehicle_count}")
        print(f"Average travel time:            {avg_travel:.3f} s")
        print(f"Average waiting time:           {avg_wait:.3f} s")
        print(f"Average CO2 emission:           {avg_co2_mg:.3f} mg/vehicle")
        print(f"Average fuel consumption:       {avg_fuel_ml:.3f} ml/vehicle")
        print(f"Total CO2 emission:             {total_co2_mg:.3f} mg")
        print(f"Total fuel consumption:         {total_fuel_ml:.3f} ml")
        print(f"(Aggregates written to: {agg_csv})")
        print(f"\nProcessing time (wall-clock):   {processing_time:.2f} s")

    finally:
        try:
            traci.close(False)
        except Exception:
            pass


if __name__ == "__main__":
    main()
