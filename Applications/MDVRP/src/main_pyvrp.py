"""Solve a Multi-Depot VRP (homogeneous, unlimited fleet) with PyVRP.

Reads instances in the Cordeau-style ".mdovrp" format (NODE_COORD_SECTION /
DEMAND_SECTION / DEPOT_SECTION), where the last vertices listed are the depots.
"""

import argparse
import csv
import math
import os
import re
import time
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import dataclass

from pyvrp import Model
from pyvrp.stop import MaxRuntime

DATA_DIR = os.path.join(os.path.dirname(__file__), "..", "data")
LITERATURE_CSV = os.path.join(os.path.dirname(__file__), "..", "..", "..", "literature", "MDVRP_literature.csv")


@dataclass
class Instance:
    name: str
    capacity: float
    coords: list[tuple[float, float]]   # index 0..dim-1, in file order
    demands: list[float]
    depot_ids: list[int]                # 0-indexed positions into coords/demands


def read_mdovrp(path: str) -> Instance:
    with open(path, "r", encoding="utf-8") as f:
        text = f.read()

    tokens = re.split(r"[\s:]+", text.strip())
    tokens = [t for t in tokens if t]

    name = "instance"
    capacity = 0.0
    dim = 0
    coords: list[tuple[float, float]] = []
    demands: list[float] = []
    depot_ids: list[int] = []

    i = 0
    while i < len(tokens):
        tok = tokens[i]
        if tok == "NAME":
            name = tokens[i + 1]
            i += 2
        elif tok == "DIMENSION":
            dim = int(tokens[i + 1])
            i += 2
        elif tok == "CAPACITY":
            capacity = float(tokens[i + 1])
            i += 2
        elif tok == "NODE_COORD_SECTION":
            j = i + 1
            coords = [(0.0, 0.0)] * dim
            while tokens[j] != "DEMAND_SECTION":
                idx = int(tokens[j]) - 1
                coords[idx] = (float(tokens[j + 1]), float(tokens[j + 2]))
                j += 3
            i = j
        elif tok == "DEMAND_SECTION":
            j = i + 1
            demands = [0.0] * dim
            for _ in range(dim):
                idx = int(tokens[j]) - 1
                demands[idx] = float(tokens[j + 1])
                j += 2
            i = j
        elif tok == "DEPOT_SECTION":
            j = i + 1
            while tokens[j] != "-1" and tokens[j] != "EOF":
                depot_ids.append(int(tokens[j]) - 1)
                j += 1
            i = j
        else:
            i += 1

    return Instance(name, capacity, coords, demands, depot_ids)


# PyVRP requires integer distances/coordinates. Scaling by this factor before
# rounding keeps 3 decimal digits of precision; divide reported costs by this
# factor to recover the true (unscaled) distance.
DIST_SCALE = 1000


def build_model(inst: Instance, num_vehicles_per_depot: int, free_return: bool = False) -> Model:
    """Build the PyVRP model.

    If free_return is True, vehicles start at their assigned real depot but
    end at a virtual "any depot" sink whose arc cost from each client equals
    the minimum distance from that client to any real depot. This mirrors
    the relaxed distance matrix used by the Julia heuristic (buildDistMatrix),
    where each side of a route independently picks its cheapest depot. The
    resulting routes are NOT physically executable as-is (the vehicle
    doesn't actually return to a single real depot) -- use this only to
    reproduce/compare against that relaxed lower bound.

    Distances are computed from the original (unrounded) coordinates, scaled
    by DIST_SCALE and rounded to the nearest integer, since PyVRP requires
    integer edge distances; client/depot x,y attributes are rounded only for
    display purposes and do not affect the distance matrix. Callers must
    divide reported costs by DIST_SCALE to recover true distances.
    """
    model = Model()

    depots = []
    for d_idx in inst.depot_ids:
        x, y = inst.coords[d_idx]
        depots.append(model.add_depot(x=round(x), y=round(y), name=f"depot_{d_idx}"))

    sink = None
    if free_return:
        cx = sum(inst.coords[d][0] for d in inst.depot_ids) / len(inst.depot_ids)
        cy = sum(inst.coords[d][1] for d in inst.depot_ids) / len(inst.depot_ids)
        sink = model.add_depot(x=round(cx), y=round(cy), name="depot_any")

    for depot in depots:
        model.add_vehicle_type(
            num_available=num_vehicles_per_depot,
            capacity=round(inst.capacity),
            start_depot=depot,
            end_depot=sink if free_return else depot,
            name=f"veh_{depot.name}",
        )

    clients = {}
    for idx in range(len(inst.coords)):
        if idx in inst.depot_ids:
            continue
        x, y = inst.coords[idx]
        clients[idx] = model.add_client(
            x=round(x),
            y=round(y),
            delivery=round(inst.demands[idx]),
            name=f"client_{idx}",
        )

    # true (unrounded) coordinates, keyed the same way as `locations` below
    true_coords = {**{d_idx: inst.coords[d_idx] for d_idx in inst.depot_ids}, **{idx: inst.coords[idx] for idx in clients}}
    if free_return:
        true_coords["_sink"] = (cx, cy)

    locations = depots + list(clients.values())
    loc_keys = list(inst.depot_ids) + list(clients.keys())
    for i, loc_i in enumerate(locations):
        xi, yi = true_coords[loc_keys[i]]
        for j, loc_j in enumerate(locations):
            if i == j:
                continue
            xj, yj = true_coords[loc_keys[j]]
            dist = round(math.hypot(xi - xj, yi - yj) * DIST_SCALE)
            model.add_edge(loc_i, loc_j, distance=dist)

    if free_return:
        for depot in depots:
            model.add_edge(depot, sink, distance=0)
            model.add_edge(sink, depot, distance=0)
        for idx, client in clients.items():
            xi, yi = inst.coords[idx]
            best = min(math.hypot(xi - inst.coords[d][0], yi - inst.coords[d][1]) for d in inst.depot_ids)
            dist = round(best * DIST_SCALE)
            model.add_edge(client, sink, distance=dist)
            model.add_edge(sink, client, distance=dist)

    return model


def solve_instance(path: str, time_limit: float, seed: int,
                    vehicles_per_depot: int | None = None,
                    free_return: bool = False, display: bool = False):
    inst = read_mdovrp(path)
    n_clients = len(inst.coords) - len(inst.depot_ids)
    fleet_cap = vehicles_per_depot or n_clients

    model = build_model(inst, fleet_cap, free_return=free_return)

    start = time.perf_counter()
    result = model.solve(stop=MaxRuntime(time_limit), seed=seed, display=display)
    elapsed = time.perf_counter() - start

    return inst, model, result, elapsed


def collect_instances(data_dir: str) -> list[str]:
    return sorted(
        os.path.join(data_dir, f)
        for f in os.listdir(data_dir)
        if f.endswith(".mdovrp")
    )


def _run_one(path: str, seed: int):
    inst = read_mdovrp(path)
    n_clients = len(inst.coords) - len(inst.depot_ids)
    time_limit = (n_clients / 100) * 60

    _, _, result, elapsed = solve_instance(path, time_limit, seed)
    best = result.best
    cost = best.distance() / DIST_SCALE
    print(f"{inst.name} seed={seed} time_limit={time_limit:.2f}s time={elapsed:.2f}s cost={cost} "
          f"feasible={best.is_feasible()} routes={len(best.routes())}")

    return inst.name, cost, elapsed


def run_all(seeds: list[int] = (1, 2, 3, 4, 5), workers: int | None = None) -> None:
    instances = collect_instances(DATA_DIR)

    os.makedirs(os.path.dirname(LITERATURE_CSV), exist_ok=True)
    with ProcessPoolExecutor(max_workers=workers) as pool:
        futures = {
            pool.submit(_run_one, path, seed): path
            for path in instances
            for seed in seeds
        }
        by_instance: dict[str, list[tuple[float, float]]] = {}
        for future in as_completed(futures):
            name, cost, elapsed = future.result()
            by_instance.setdefault(name, []).append((cost, elapsed))

    with open(LITERATURE_CSV, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["Instance", "BKS", "Ref", "BestRef", "AvgRef", "TimeRef"])
        for name in sorted(by_instance):
            runs = by_instance[name]
            costs = [c for c, _ in runs]
            times = [t for _, t in runs]
            best_cost = min(costs)
            avg_cost = sum(costs) / len(costs)
            avg_time = sum(times) / len(times)
            writer.writerow([name, best_cost, "PyVRP", best_cost, round(avg_cost, 2), round(avg_time, 3)])


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("instance", nargs="?", default=None,
                         help="path to a .mdovrp instance file; omit to run every "
                              "instance in data/ and write literature/MDVRP_literature.csv")
    parser.add_argument("--time-limit", type=float, default=30.0,
                         help="max runtime in seconds for a single instance run (default: 30); "
                              "ignored when running every instance, where time_limit = (n/100)*60")
    parser.add_argument("--seed", type=int, default=1,
                         help="seed for a single instance run; ignored when running every "
                              "instance, where seeds = [1, 2, 3, 4, 5]")
    parser.add_argument("--vehicles-per-depot", type=int, default=None,
                         help="cap on vehicles per depot; default = number of clients (effectively unlimited)")
    parser.add_argument("--free-return", action="store_true",
                         help="let vehicles end at whichever depot is cheapest (NOT physically "
                              "valid -- reproduces the Julia heuristic's relaxed lower bound)")
    parser.add_argument("--workers", type=int, default=None,
                         help="parallel worker processes when running every instance "
                              "(default: os.cpu_count())")
    args = parser.parse_args()

    if args.instance is None:
        run_all(workers=args.workers)
        return

    inst, model, result, elapsed = solve_instance(
        args.instance, args.time_limit, args.seed,
        vehicles_per_depot=args.vehicles_per_depot,
        free_return=args.free_return, display=True,
    )

    best = result.best
    print(f"{inst.name} time={elapsed:.2f}s cost={best.distance() / DIST_SCALE} "
          f"feasible={best.is_feasible()} routes={len(best.routes())}")

    for r in best.routes():
        depot_name = model.locations[r.start_depot()].name
        visits = " ".join(model.locations[c].name for c in r.visits())
        print(f"  [{depot_name}] {visits}  (demand={r.delivery()}, dist={r.distance() / DIST_SCALE})")


if __name__ == "__main__":
    main()
