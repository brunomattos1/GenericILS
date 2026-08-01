"""Solve a Heterogeneous Fleet VRP (HFVRP) with PyVRP.

Reads instances in the Brandao-style flat format used by data.jl's
readBrandaoData: n, then (n+1) "id x y" coord lines (vertex 0 = depot),
then (n+1) "id demand" lines, then nv, then nv "Q fixed factor l u" lines
(one per vehicle type).

Each vehicle type has its own fixed cost and per-distance factor, mirroring
Applications/HFVRP/src/data.jl's cost(i,j,k) = factor[k] * euclidean(i,j),
with a half-fixed-cost surcharge on depot arcs baked into the Julia cost
dict -- here we instead charge fixed_cost once per route via PyVRP's native
vehicle_type.fixed_cost, which is the standard (non-double-counted) way to
model it.
"""

import argparse
import math
import re
import time
from dataclasses import dataclass

from pyvrp import Model
from pyvrp.stop import MaxRuntime


@dataclass
class VehicleType:
    capacity: int
    fixed: float
    factor: float
    lb: int
    ub: int


@dataclass
class Instance:
    name: str
    coords: list[tuple[float, float]]   # index 0 = depot, 1..n = customers
    demands: list[int]
    veh_types: list[VehicleType]


def _read_vehicle_types(it) -> list[VehicleType]:
    nv = int(next(it))
    veh_types = []
    for _ in range(nv):
        q = int(float(next(it)))
        fixed = float(next(it))
        factor = float(next(it))
        lb = int(float(next(it)))
        ub = int(float(next(it)))
        veh_types.append(VehicleType(q, fixed, factor, lb, ub))
    return veh_types


def read_brandao(path: str) -> Instance:
    """Coords block (id x y) for all n+1 vertices, then demand block (id demand)."""
    tokens: list[str] = []
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            tokens.extend(line.split())

    it = iter(tokens)
    n = int(next(it))

    coords = []
    for _ in range(n + 1):
        next(it)  # id (implicit 0..n order)
        x = float(next(it))
        y = float(next(it))
        coords.append((x, y))

    demands = []
    for _ in range(n + 1):
        next(it)  # id
        demands.append(int(float(next(it))))

    veh_types = _read_vehicle_types(it)

    name = __import__("os").path.splitext(__import__("os").path.basename(path))[0]
    return Instance(name, coords, demands, veh_types)


def read_classic(path: str) -> Instance:
    """Single block of (id x y demand) for all n+1 vertices, then vehicle types."""
    tokens: list[str] = []
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            tokens.extend(line.split())

    it = iter(tokens)
    n = int(next(it))

    coords = []
    demands = []
    for _ in range(n + 1):
        next(it)  # id
        x = float(next(it))
        y = float(next(it))
        d = int(float(next(it)))
        coords.append((x, y))
        demands.append(d)

    veh_types = _read_vehicle_types(it)

    name = __import__("os").path.splitext(__import__("os").path.basename(path))[0]
    return Instance(name, coords, demands, veh_types)


DIST_SCALE = 1000  # PyVRP distances/costs are integers; scale to preserve factor precision.


def build_model(inst: Instance) -> Model:
    model = Model()

    depot = model.add_depot(x=round(inst.coords[0][0]), y=round(inst.coords[0][1]))

    clients = []
    for idx in range(1, len(inst.coords)):
        x, y = inst.coords[idx]
        clients.append(model.add_client(x=round(x), y=round(y), delivery=inst.demands[idx],
                                         name=f"client_{idx}"))

    locations = [depot] + clients
    euclid = {}
    for i, loc_i in enumerate(locations):
        for j, loc_j in enumerate(locations):
            if i == j:
                continue
            euclid[(i, j)] = math.hypot(loc_i.x - loc_j.x, loc_i.y - loc_j.y)

    for k, vt in enumerate(inst.veh_types):
        profile = model.add_profile(name=f"profile_{k}")
        for (i, j), d in euclid.items():
            model.add_edge(locations[i], locations[j],
                            distance=round(d * vt.factor * DIST_SCALE), profile=profile)

        # ub == 0 means "unlimited" in the Brandao/Cordeau convention.
        num_available = vt.ub if vt.ub > 0 else len(clients)
        model.add_vehicle_type(
            num_available=num_available,
            capacity=vt.capacity,
            start_depot=depot,
            end_depot=depot,
            fixed_cost=round(vt.fixed * DIST_SCALE),
            unit_distance_cost=1,
            profile=profile,
            name=f"veh_type_{k}",
        )

    return model


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("instance", nargs="?",
                         default=r"C:\Users\Administrador\Documents\GitHub\GenericILS\Applications\HFVRP\data\brandaoN0.txt",
                         help="path to a Brandao-format HFVRP instance file")
    parser.add_argument("--time-limit", type=float, default=30.0,
                         help="max runtime in seconds (default: 30)")
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--classic", action="store_true",
                         help="instance uses the classic single-block (id x y demand) format "
                              "instead of Brandao's separate coord/demand blocks")
    args = parser.parse_args()

    inst = read_classic(args.instance) if args.classic else read_brandao(args.instance)
    model = build_model(inst)

    start = time.perf_counter()
    result = model.solve(stop=MaxRuntime(args.time_limit), seed=args.seed, display=False)
    elapsed = time.perf_counter() - start

    best = result.best
    data = model.data()
    fixed_total = sum(data.vehicle_types()[r.vehicle_type()].fixed_cost for r in best.routes())
    dist_total = sum(r.distance_cost() for r in best.routes())
    real_cost = (fixed_total + dist_total) / DIST_SCALE

    print(f"{inst.name} time={elapsed:.2f}s cost={real_cost:.2f} "
          f"feasible={best.is_feasible()} routes={len(best.routes())}")

    for r in best.routes():
        vt = data.vehicle_types()[r.vehicle_type()]
        visits = " ".join(model.locations[c].name for c in r.visits())
        route_cost = (vt.fixed_cost + r.distance_cost()) / DIST_SCALE
        print(f"  [{vt.name}] {visits}  (demand={r.delivery()}, cost={route_cost:.2f})")


if __name__ == "__main__":
    main()
