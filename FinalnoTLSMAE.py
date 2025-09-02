# FinalnoTLS.py
# -*- coding: utf-8 -*-

import os, sys, glob, csv, time, xml.etree.ElementTree as ET
from collections import defaultdict, deque
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

# Rolling buffers per-edge
WINDOW_N = 10
EDGE_BUF = {}  # edge_id -> {"speed": deque(maxlen=WINDOW_N), "occ": deque(maxlen=WINDOW_N)}

# =========================
# Config helpers
# =========================
def resolve_cfg_path() -> str:
    """
    Order:
      1) argv[1] if provided and exists
      2) env SUMO_CFG if exists
      3) ./osm.sumocfg if exists
      4) first *.sumocfg in CWD
    """
    # 1) CLI arg
    if len(sys.argv) > 1:
        arg = sys.argv[1]
        if os.path.isabs(arg) and os.path.exists(arg):
            return arg
        cand = os.path.abspath(arg)
        if os.path.exists(cand):
            return cand

    # 2) env
    env_cfg = os.environ.get("SUMO_CFG")
    if env_cfg:
        cand = os.path.abspath(env_cfg)
        if os.path.exists(cand):
            return cand

    # 3) preferred default
    if os.path.exists("osm.sumocfg"):
        return os.path.abspath("osm.sumocfg")

    # 4) any *.sumocfg
    all_cfgs = sorted(glob.glob("*.sumocfg"))
    if all_cfgs:
        return os.path.abspath(all_cfgs[0])

    raise FileNotFoundError("No .sumocfg found. Pass it as argv[1] or set SUMO_CFG or place osm.sumocfg in CWD.")

def read_net_from_cfg(cfg_path):
    try:
        root = ET.parse(cfg_path).getroot()
        tag = root.find(".//input/net-file")
        if tag is not None and "value" in tag.attrib:
            return os.path.abspath(os.path.join(os.path.dirname(cfg_path), tag.attrib["value"]))
    except ET.ParseError:
        pass
    nets = glob.glob("*.net.xml*")
    return os.path.abspath(nets[0]) if nets else None

def read_route_files_from_cfg(cfg_path):
    try:
        root = ET.parse(cfg_path).getroot()
        tag = root.find(".//input/route-files")
        if tag is None or "value" not in tag.attrib:
            return []
        base = os.path.dirname(cfg_path)
        return [os.path.abspath(os.path.join(base, v)) for v in tag.attrib["value"].split()]
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
    try:
        occ = traci.edge.getLastStepOccupancy(out_edge)
    except Exception:
        occ = 0.0
    try:
        mean_speed = traci.edge.getLastStepMeanSpeed(out_edge)
    except Exception:
        mean_speed = None

    if not isinstance(occ, (int, float)) or occ != occ:
        occ = 0.0
    occ = max(0.0, min(1.0, float(occ)))

    if not isinstance(mean_speed, (int, float)) or mean_speed <= 0:
        mean_speed = speed_limit * MIN_SPEED_FRACTION

    buf = EDGE_BUF.setdefault(out_edge, {"speed": deque(maxlen=WINDOW_N), "occ": deque(maxlen=WINDOW_N)})
    buf["speed"].append(float(mean_speed))
    buf["occ"].append(float(occ))

    sm_speed = sum(buf["speed"]) / len(buf["speed"])
    sm_occ   = sum(buf["occ"]) / len(buf["occ"])

    if sm_occ < OCCUPANCY_FREE_THRESH:
        use_speed = max(speed_limit, 0.1)
    elif sm_speed > SMOOTHING_MIN_SPEED:
        use_speed = sm_speed
    else:
        use_speed = max(speed_limit * MIN_SPEED_FRACTION, 0.1)

    return use_speed, sm_occ

# =========================
# Prediction helpers (Approach 2: per decision window)
# =========================
def predict_remaining_time_for_route(G, current_edge, lane_id, lane_pos, route_edges):
    """
    คาดเวลาเดินทางที่เหลือจาก 'จุดปัจจุบัน' บน lane_id/lane_pos
    - คิดเศษระยะบนเลนปัจจุบัน
    - บวกน้ำหนักของขอบถัด ๆ ไป (G.edges[u,v]["w"]) ถ้าไม่มี weight ใช้ length/speed_limit ของ node ปลายทาง v
    """
    try:
        lane_len = traci.lane.getLength(lane_id)
    except Exception:
        lane_len = None

    # 1) เศษระยะบน edge ปัจจุบัน
    rem_time_cur = 0.0
    if lane_len is not None:
        rem_len = max(0.0, lane_len - lane_pos)
        nd = G.nodes.get(current_edge, {})
        speed_limit = nd.get("speed_limit", 13.9)
        use_speed, _ = expected_speed(current_edge, speed_limit)
        rem_time_cur = rem_len / max(use_speed, 0.1)

    # 2) เวลาตามเส้นทางจาก edge ถัดไป
    path_time = 0.0
    if len(route_edges) >= 2:
        for u, v in zip(route_edges[:-1], route_edges[1:]):
            w = G.edges[u, v].get("w", None) if (u, v) in G.edges else None
            if w is None:
                nd = G.nodes.get(v, {})
                sl = nd.get("speed_limit", 13.9)
                ln = nd.get("length", 1.0)
                w = (ln / max(sl, 0.1))
            path_time += float(w)

    return rem_time_cur + path_time

def snapshot_prediction(vs, snap_time, pred_remaining):
    """เริ่มรอบการทำนายใหม่"""
    vs["last_decision_time"] = float(snap_time)
    vs["last_pred_remaining"] = float(pred_remaining)

def close_out_prediction(vs, now_time):
    """ปิดรอบการทำนายปัจจุบัน → คืน (abs_err, rel_err) หรือ (None, None) ถ้าไม่มีรอบค้าง"""
    last_t = vs.get("last_decision_time", None)
    pred   = vs.get("last_pred_remaining", None)
    if (last_t is None) or (pred is None):
        return (None, None)
    actual = max(0.0, float(now_time) - float(last_t))
    abs_err = abs(actual - float(pred))
    rel_err = (abs_err / actual) if actual > 1e-6 else 0.0
    # เคลียร์ snapshot เดิม (ถ้าอยากเก็บเป็นลิสต์ประวัติ ค่อยขยายเพิ่มได้)
    vs.pop("last_decision_time", None)
    vs.pop("last_pred_remaining", None)
    return (abs_err, rel_err)

# =========================
# Main
# =========================
def main():
    wall_start = time.perf_counter()

    cfg = resolve_cfg_path()
    net_path = read_net_from_cfg(cfg)
    if not net_path or not os.path.exists(net_path):
        raise FileNotFoundError(f"Cannot locate .net.xml referenced by cfg: {cfg}")

    route_files = read_route_files_from_cfg(cfg)
    for rf in route_files:
        if not os.path.exists(rf):
            raise FileNotFoundError(f"route-file not found: {rf}")

    print(f"[CFG] Using cfg: {cfg}")
    print(f"[NET] Using net: {net_path}")
    if route_files:
        print("[ROUTES] " + ", ".join(route_files))

    agg_csv = f"aggregates{TAG}.csv"
    sumo_bin = sumolib.checkBinary("sumo")  # change to "sumo-gui" if you want GUI
    cmd = [sumo_bin, "-c", cfg, "--quit-on-end"]
    print("[INFO] Launching:", " ".join(cmd))
    traci.start(cmd)

    # ---- Accumulators (per decision window) ----
    mae_abs_sum = 0.0
    mae_rel_sum = 0.0
    mae_count   = 0
    MAE_ROLL_N  = 10
    mae_roll_abs = deque(maxlen=MAE_ROLL_N)
    mae_roll_rel = deque(maxlen=MAE_ROLL_N)

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

        while traci.simulation.getMinExpectedNumber() > 0:
            traci.simulationStep()
            sim_time = traci.simulation.getTime()

            # step size (s)
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

            # 3) reroute + (build graphs/weights)
            if not NO_OPT:
                def compute_edge_weights_for_class(veh_class_key, G):
                    per_out_edge_cost = {}
                    for v in G.nodes:
                        nd = G.nodes[v]
                        speed_limit = nd.get("speed_limit", 13.9)
                        use_speed, occ = expected_speed(v, speed_limit)
                        base_tt = nd.get("length", 1.0) / max(use_speed, 0.1)
                        cost = base_tt * (1.0 + DENSITY_ALPHA * float(occ))
                        per_out_edge_cost[v] = cost
                    for u, v in G.edges:
                        G.edges[u, v]["w"] = per_out_edge_cost[v]

                step_classes = {get_vehicle_class(vid) or "_ANY_" for vid in active_ids}

                for cls in step_classes:
                    if cls not in edge_graph_cache:
                        real_cls = None if cls == "_ANY_" else cls
                        edge_graph_cache[cls] = build_edge_graph_from_traci(real_cls)
                    compute_edge_weights_for_class(cls, edge_graph_cache[cls])

                # reroute per vehicle
                for vid in active_ids:
                    try:
                        lane_id = traci.vehicle.getLaneID(vid)
                        lane_pos = traci.vehicle.getLanePosition(vid)
                        lane_len = traci.lane.getLength(lane_id)
                        if (lane_len - lane_pos) < NEAR_JUNCTION_SKIP_DIST:
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
                            pass
                        continue

                    try:
                        edge_path_new = nx.shortest_path(G, cur_edge, dest_edge, weight="w")
                    except Exception:
                        invalid_count[vid] += 1
                        if invalid_count[vid] >= INVALID_THRESHOLD:
                            invalid_count[vid] = 0
                        try:
                            traci.vehicle.rerouteTraveltime(vid)
                        except traci.TraCIException:
                            pass
                        continue

                    new_route = list(edge_path_new)

                    allowed_from_lane = next_edges_allowed_from_current_lane(vid)
                    if len(new_route) >= 2 and allowed_from_lane and (new_route[1] not in allowed_from_lane):
                        continue
                    if len(new_route) >= 2 and is_uturn_pair(new_route[0], new_route[1]):
                        continue

                    # ---- Close previous decision window BEFORE setting new route ----
                    vs = vehicle_states.get(vid)
                    if vs is not None:
                        abs_err, rel_err = close_out_prediction(vs, sim_time)
                        if abs_err is not None:
                            mae_abs_sum += abs_err
                            mae_rel_sum += rel_err
                            mae_count   += 1
                            mae_roll_abs.append(abs_err)
                            mae_roll_rel.append(rel_err)

                    try:
                        traci.vehicle.setRoute(vid, new_route)
                        last_reroute[vid] = sim_time
                        invalid_count[vid] = 0

                        # ---- Snapshot NEW decision window ----
                        vs = vehicle_states.get(vid)
                        if vs is not None:
                            try:
                                lane_id = traci.vehicle.getLaneID(vid)
                                lane_pos = traci.vehicle.getLanePosition(vid)
                            except traci.TraCIException:
                                lane_id, lane_pos = None, 0.0
                            pred_rem = predict_remaining_time_for_route(G, cur_edge, lane_id, lane_pos, new_route)
                            snapshot_prediction(vs, sim_time, pred_rem)

                    except traci.TraCIException:
                        continue

                # ---- Initial snapshot for vehicles that never snapped yet ----
                for vid in active_ids:
                    vs = vehicle_states.get(vid)
                    if not vs or ("last_decision_time" in vs):
                        continue
                    try:
                        cur_edge = traci.vehicle.getRoadID(vid)
                        if not cur_edge or cur_edge.startswith(":"):
                            continue
                        route = traci.vehicle.getRoute(vid)
                        if not route:
                            continue
                        veh_class = get_vehicle_class(vid)
                        key = veh_class or "_ANY_"
                        G = edge_graph_cache.get(key)
                        if not G or (cur_edge not in G) or (route[-1] not in G):
                            continue
                        lane_id = traci.vehicle.getLaneID(vid)
                        lane_pos = traci.vehicle.getLanePosition(vid)
                        pred_rem = predict_remaining_time_for_route(G, cur_edge, lane_id, lane_pos, route)
                        snapshot_prediction(vs, sim_time, pred_rem)
                    except traci.TraCIException:
                        pass

            # 4) arrivals → metrics (and close any open window)
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

                # ---- Close any open decision window at arrival ----
                try:
                    abs_err, rel_err = close_out_prediction(vs, sim_time)
                    if abs_err is not None:
                        mae_abs_sum += abs_err
                        mae_rel_sum += rel_err
                        mae_count   += 1
                        mae_roll_abs.append(abs_err)
                        mae_roll_rel.append(rel_err)
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

        # ---- MAE / RMAE over decision windows ----
        if mae_count > 0:
            mae_dec  = mae_abs_sum / mae_count
            rmae_dec = mae_rel_sum / mae_count
        else:
            mae_dec = rmae_dec = 0.0

        roll_mae_dec  = (sum(mae_roll_abs) / len(mae_roll_abs)) if mae_roll_abs else 0.0
        roll_rmae_dec = (sum(mae_roll_rel) / len(mae_roll_rel)) if mae_roll_rel else 0.0

        print("\n===== Simulation Summary =====")
        print(f"Vehicles simulated (included):  {vehicle_count}")
        print(f"Average travel time:            {avg_travel:.3f} s")
        print(f"Average waiting time:           {avg_wait:.3f} s")
        print(f"Average CO2 emission:           {avg_co2_mg:.3f} mg/vehicle")
        print(f"Average fuel consumption:       {avg_fuel_ml:.3f} ml/vehicle")
        print(f"Total CO2 emission:             {total_co2_mg:.3f} mg")
        print(f"Total fuel consumption:         {total_fuel_ml:.3f} ml")
        print(f"(Aggregates written to: {agg_csv})")
        print(f"\nMAE per decision window:        {mae_dec:.3f} s")
        print(f"RMAE per decision window:       {rmae_dec:.3f}")
        print(f"MAE last {len(mae_roll_abs) or 0} windows:       {roll_mae_dec:.3f} s")
        print(f"RMAE last {len(mae_roll_rel) or 0} windows:      {roll_rmae_dec:.3f}")
        print(f"\nProcessing time (wall-clock):   {processing_time:.2f} s")

    finally:
        try:
            traci.close(False)
        except Exception:
            pass

if __name__ == "__main__":
    main()
