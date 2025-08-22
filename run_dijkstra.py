# export SUMO_HOME="/usr/share/sumo"
# export PYTHONPATH="$SUMO_HOME/tools:$PYTHONPATH"

import traci
import csv
import os
from sumolib.net import readNet
from dijkstra import Dijkstra

CSV_FILE = "edge_graph.csv"

def load_network(net_file):
    net = readNet(net_file)
    return net

def load_graph_from_csv(filename=CSV_FILE):
    graph = {}
    if not os.path.isfile(filename):
        print(f"[CSV Load] {filename} not found. Proceeding with default initialization.")
        return graph
    try:
        with open(filename, mode='r') as file:
            reader = csv.DictReader(file)
            for row in reader:
                from_edge = row["From"]
                to_edge = row["To"]
                if from_edge.startswith(":") or to_edge.startswith(":"):
                    continue  # Skip internal edges
                avg_time = float(row["AvgTime"])
                graph.setdefault(from_edge, {})[to_edge] = avg_time
        print(f"[CSV Load] Loaded edge graph from {filename}")
    except Exception as e:
        print(f"[CSV Load] Failed to read {filename}: {e}")
    return graph

def save_graph_to_csv(graph, filename=CSV_FILE):
    with open(filename, mode='w', newline='') as file:
        writer = csv.DictWriter(file, fieldnames=["From", "To", "AvgTime"])
        writer.writeheader()
        for from_edge, targets in graph.items():
            for to_edge, avg_time in targets.items():
                writer.writerow({"From": from_edge, "To": to_edge, "AvgTime": avg_time})

def initialize_graph_with_defaults(net, graph):
    for edge in net.getEdges():
        from_edge = edge.getID()
        if from_edge.startswith(":"):
            continue
        to_node = edge.getToNode()
        for next_edge in to_node.getOutgoing():
            to_edge = next_edge.getID()
            if to_edge.startswith(":"):
                continue    
            length = next_edge.getLength()
            max_speed = max([lane.getSpeed() for lane in next_edge.getLanes()] or [1.0])
            default_time = length / (0.5 * max_speed) if max_speed > 0 else 9999
            graph.setdefault(from_edge, {})[to_edge] = default_time

def update_travel_stats(veh_id, sim_time, vehicle_states, edge_to_junction, junction_stats, junction_graph, net):
    current_edge = traci.vehicle.getRoadID(veh_id)
    last_edge = vehicle_states[veh_id]["last_edge"]

    if current_edge != last_edge:
        entry_time = vehicle_states[veh_id]["entry_time"]
        duration = sim_time - entry_time

        from_junc, _ = edge_to_junction.get(last_edge, (None, None))
        _, to_junc = edge_to_junction.get(current_edge, (None, None))

        if from_junc and to_junc:
            valid = any(edge.getToNode().getID() == to_junc for edge in net.getNode(from_junc).getOutgoing())
            if valid:
                key = (from_junc, to_junc)
                junction_stats.setdefault(key, []).append(duration)
                junction_stats[key] = junction_stats[key][-5:]
                avg = sum(junction_stats[key]) / len(junction_stats[key])
                junction_graph.setdefault(from_junc, {})[to_junc] = avg
                print(f"TQ ({from_junc} → {to_junc}): avg time = {avg:.2f}s")

        vehicle_states[veh_id]["last_edge"] = current_edge
        vehicle_states[veh_id]["entry_time"] = sim_time

def junctions_to_edges(junction_path, net):
    edge_path = []
    last_edge = None
    for i in range(len(junction_path) - 1):
        from_j = junction_path[i]
        to_j = junction_path[i + 1]
        found = False
        for edge in net.getNode(from_j).getOutgoing():
            if edge.getToNode().getID() == to_j:
                candidate = edge.getID()
                if last_edge and candidate.lstrip('-') == last_edge.lstrip('-') and candidate != last_edge:
                    print(f"[Edge Filter] Skipping U-turn: {last_edge} → {candidate}")
                    continue
                edge_path.append(candidate)
                last_edge = candidate
                found = True
                break
        if not found:
            print(f"[Edge Warning] No connecting edge from {from_j} to {to_j} in path: {junction_path}")
            return []
    return edge_path

def try_reroute_vehicle(veh_id, sim_time, vehicle_states, edge_to_junction, junction_graph, net):
    current_edge = traci.vehicle.getRoadID(veh_id)
    if current_edge not in edge_to_junction:
        return
    _, via_junc = edge_to_junction[current_edge]
    route = traci.vehicle.getRoute(veh_id)
    if len(route) < 2 or route[-1] not in edge_to_junction:
        return
    _, dest_junc = edge_to_junction[route[-1]]

    try:
        dijk = Dijkstra(junction_graph, via_junc)
        dijk.dijkstra()
        path = dijk.build_path(dest_junc)
    except KeyError:
        return

    if not path or len(path) < 2:
        return

    full_path = [current_edge] + junctions_to_edges(path, net)
    if not full_path or len(full_path) < 2:
        return  

    if sim_time - vehicle_states[veh_id]["last_reroute_time"] > 30:
        try:
            traci.vehicle.setRoute(veh_id, full_path)
            vehicle_states[veh_id]["last_reroute_time"] = sim_time
            print(f"[Dijkstra] Recomputed route for {veh_id}")
        except traci.TraCIException as e:
            print(f"[SetRoute Error] {veh_id}: {e}")

def run_simulation():
    net = load_network("osm.net.xml.gz")
    edge_to_junction = {
        edge.getID(): (edge.getFromNode().getID(), edge.getToNode().getID())
        for edge in net.getEdges()
    }
    junction_graph = load_graph_from_csv()
    if not junction_graph:
        initialize_graph_with_defaults(net, junction_graph)

    vehicle_states = {}
    junction_stats = {}

    total_travel_time = 0.0
    total_CO2 = 0.0
    total_waiting_time = 0.0
    vehicle_count = 0

    traci.start(["sumo", "-c", "osm.sumocfg"])

    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        sim_time = traci.simulation.getTime()

        step_sec = traci.simulation.getDeltaT() / 1000.0
        # Per-step accumulation for CO2 (mg) and waiting time (s)
        for _vid in traci.vehicle.getIDList():
            vs = vehicle_states.get(_vid)
            if vs is None:
                continue
            try:
                vs["co2"] += traci.vehicle.getCO2Emission(_vid) * step_sec
                curr_acc_wait = traci.vehicle.getAccumulatedWaitingTime(_vid)
                delta_wait = max(0.0, curr_acc_wait - vs.get("last_wait_acc", 0.0))
                vs["waiting_time"] += delta_wait
                vs["last_wait_acc"] = curr_acc_wait
            except traci.TraCIException:
                pass

        teleported_count = traci.simulation.getEndingTeleportNumber()
        if teleported_count > 0:
            print(f"[Teleport] {teleported_count} vehicles teleported.")

        for veh_id in traci.vehicle.getIDList():
            current_edge = traci.vehicle.getRoadID(veh_id)

            if veh_id not in vehicle_states:
                vehicle_states[veh_id] = {
                    "last_edge": current_edge,
                    "entry_time": sim_time,
                    "depart_time": sim_time,
                    "last_reroute_time": -999,
                    "co2": 0.0,
                    "waiting_time": 0.0,
                    "last_wait_acc": traci.vehicle.getAccumulatedWaitingTime(veh_id),
                }
                continue

            update_travel_stats(veh_id, sim_time, vehicle_states, edge_to_junction, junction_stats, junction_graph, net)

        for veh_id in traci.simulation.getArrivedIDList():
            try:
                depart_time = vehicle_states[veh_id].get("depart_time", vehicle_states[veh_id]["entry_time"])
                travel_time = sim_time - depart_time
                co2 = vehicle_states[veh_id]["co2"]
                waiting_time = vehicle_states[veh_id]["waiting_time"]

                total_travel_time += travel_time
                total_CO2 += co2
                total_waiting_time += waiting_time
                vehicle_count += 1

                vehicle_states.pop(veh_id, None)
            except Exception as e:
                print(f"[Metrics Error] {veh_id}: {e}")

        for veh_id in traci.vehicle.getIDList():
            try_reroute_vehicle(veh_id, sim_time, vehicle_states, edge_to_junction, junction_graph, net)

    traci.close()
    save_graph_to_csv(junction_graph)

    if vehicle_count > 0:
        avg_travel_time = total_travel_time / vehicle_count
        avg_CO2 = total_CO2 / vehicle_count
        avg_waiting_time = total_waiting_time / vehicle_count

        print(f"\n--- Reroute Performance Summary ---")
        print(f"Vehicles Simulated:     {vehicle_count}")
        print(f"Avg Travel Time:        {avg_travel_time:.2f} s")
        print(f"Avg CO₂ Emissions:      {avg_CO2:.2f} mg")
        print(f"Avg Waiting Time:       {avg_waiting_time:.2f} s")
    else:
        print("No vehicle performance data collected.")

    print("Simulation complete.")


if __name__ == "__main__":
    run_simulation()
