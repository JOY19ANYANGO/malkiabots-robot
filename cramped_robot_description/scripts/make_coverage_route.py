#!/usr/bin/env python3
"""Generate a waypoint route that covers every reachable part of a map.

    python3 make_coverage_route.py maps/gamefield.yaml config/coverage_waypoints.yaml [--start X Y] [--spacing 0.45] [--png route.png]

Method: free cells of the map are eroded by the robot's circumscribed radius (so only positions the
robot can actually occupy remain), the component reachable from the start is kept, a grid of candidate
points (one per `spacing` x `spacing` cell, snapped to the most open position in that cell) is laid over
it, and the points are ordered by a nearest-neighbour tour + 2-opt using geodesic (through-the-maze)
distances. Each waypoint's yaw points at the next one. Any map_server-compatible yaml/pgm works.
"""
import argparse, math, sys
from collections import deque

import numpy as np
import yaml
from PIL import Image
from scipy import ndimage

# Minimum clearance from walls for a waypoint / for a passage to count as passable.
# Robot: 0.27 x 0.255 m -> inscribed radius 0.13 m; Nav2 treats cells closer than that as lethal.
CLEARANCE = 0.20


def load_map(yaml_path):
    meta = yaml.safe_load(open(yaml_path))
    img_path = yaml_path.rsplit("/", 1)[0] + "/" + meta["image"] if "/" in yaml_path else meta["image"]
    img = np.array(Image.open(img_path).convert("L"))
    img = np.flipud(img)                                      # row 0 = min y
    res = float(meta["resolution"]); ox, oy = meta["origin"][:2]
    free = img >= 250                                         # 254 = free; 205 (unknown) and 0 (wall) are not
    unknown = (img > 10) & (img < 250)
    return free, unknown, res, ox, oy


def geodesic(mask, src):
    """BFS distances (in cells, 8-connected with sqrt2) from src over mask."""
    H, W = mask.shape
    dist = np.full((H, W), np.inf); dist[src] = 0
    # Dijkstra-lite with two-bucket approximation: use scipy's exact approach via iterative BFS on 4-neigh, then correct diagonals
    import heapq
    pq = [(0.0, src)]
    while pq:
        d, (i, j) = heapq.heappop(pq)
        if d > dist[i, j]:
            continue
        for di, dj, w in ((1,0,1),(-1,0,1),(0,1,1),(0,-1,1),(1,1,1.4142),(1,-1,1.4142),(-1,1,1.4142),(-1,-1,1.4142)):
            ni, nj = i+di, j+dj
            if 0 <= ni < H and 0 <= nj < W and mask[ni, nj] and d + w < dist[ni, nj]:
                dist[ni, nj] = d + w; heapq.heappush(pq, (d + w, (ni, nj)))
    return dist


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("map_yaml"); ap.add_argument("out_yaml")
    ap.add_argument("--start", type=float, nargs=2, default=[-0.54, 1.15])
    ap.add_argument("--spacing", type=float, default=0.45)
    ap.add_argument("--clearance", type=float, default=CLEARANCE)
    ap.add_argument("--png", default=None)
    a = ap.parse_args()

    free, unknown, res, ox, oy = load_map(a.map_yaml)
    H, W = free.shape
    # positions the robot centre may occupy
    dist_to_wall = ndimage.distance_transform_edt(free) * res
    drivable = dist_to_wall > a.clearance
    to_cell = lambda x, y: (int((y - oy) / res), int((x - ox) / res))
    to_xy = lambda i, j: (ox + (j + 0.5) * res, oy + (i + 0.5) * res)
    s = to_cell(*a.start)
    if not drivable[s]:
        # snap start to nearest drivable cell
        ii, jj = np.where(drivable); k = np.argmin((ii - s[0])**2 + (jj - s[1])**2); s = (ii[k], jj[k])
    lab, nlab = ndimage.label(drivable, structure=np.ones((3, 3)))
    reach = lab == lab[s]
    print(f"reachable (clearance {a.clearance} m): {reach.sum()*res*res:.2f} m^2 of {drivable.sum()*res*res:.2f} m^2 drivable, "
          f"{free.sum()*res*res:.2f} m^2 free")
    for k in range(1, nlab + 1):
        if k != lab[s] and (lab == k).sum() * res * res > 0.2:
            ii, jj = np.where(lab == k)
            print(f"  NOT reachable: {((lab==k).sum()*res*res):.2f} m^2 at x {ox+jj.min()*res:.2f}..{ox+jj.max()*res:.2f}, "
                  f"y {oy+ii.min()*res:.2f}..{oy+ii.max()*res:.2f} (entrance narrower than {2*a.clearance:.2f} m)")

    # one candidate per spacing cell: the most open reachable point in it
    cands = []
    n = int(round(a.spacing / res))
    for i0 in range(0, H, n):
        for j0 in range(0, W, n):
            block = reach[i0:i0+n, j0:j0+n]
            if not block.any():
                continue
            dsub = np.where(block, dist_to_wall[i0:i0+n, j0:j0+n], -1)
            k = np.unravel_index(np.argmax(dsub), dsub.shape)
            cands.append((i0 + k[0], j0 + k[1]))
    cands = [c for c in cands if c != s]
    pts = [s] + cands
    print(f"{len(pts)} waypoints (spacing {a.spacing} m)")

    # geodesic distance matrix
    D = np.zeros((len(pts), len(pts)))
    for k, p in enumerate(pts):
        g = geodesic(reach, p)
        D[k] = [g[q] * res for q in pts]
    assert np.isfinite(D).all()

    # nearest-neighbour tour from the start, then 2-opt
    order = [0]; left = set(range(1, len(pts)))
    while left:
        last = order[-1]; nxt = min(left, key=lambda q: D[last, q]); order.append(nxt); left.remove(nxt)
    def tour_len(o): return sum(D[o[k], o[k+1]] for k in range(len(o)-1))
    improved = True
    while improved:
        improved = False
        for i in range(1, len(order) - 1):
            for j in range(i + 1, len(order)):
                if j - i < 2: continue
                a_, b_, c_ = order[i-1], order[i], order[j]
                d_ = order[j+1] if j + 1 < len(order) else None
                old = D[a_, b_] + (D[c_, d_] if d_ is not None else 0)
                new = D[a_, c_] + (D[b_, d_] if d_ is not None else 0)
                if new + 1e-9 < old:
                    order[i:j+1] = order[i:j+1][::-1]; improved = True
    route = [to_xy(*pts[k]) for k in order]
    print(f"tour length {tour_len(order):.1f} m")

    # yaw = heading to the next waypoint; last one faces the start
    wps = []
    for k, (x, y) in enumerate(route):
        nx, ny = route[k+1] if k + 1 < len(route) else route[0]
        wps.append({"x": round(float(x), 3), "y": round(float(y), 3), "yaw": round(float(math.atan2(ny - y, nx - x)), 3)})
    yaml.safe_dump({"frame_id": "map", "waypoints": wps}, open(a.out_yaml, "w"), sort_keys=False)

    # coverage check: free cells within 0.35 m of the route polyline
    covered = np.zeros_like(reach)
    for (x0, y0), (x1, y1) in zip(route[:-1], route[1:]):
        for t in np.linspace(0, 1, max(2, int(math.hypot(x1-x0, y1-y0) / res))):
            i, j = to_cell(x0 + t*(x1-x0), y0 + t*(y1-y0)); covered[i, j] = True
    covered = ndimage.binary_dilation(covered, iterations=int(0.35/res))
    print(f"reachable free space within 0.35 m of the route: {100*(covered & reach).sum()/reach.sum():.1f}%")

    if a.png:
        import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(12, 6.5))
        img = np.where(free, 254, np.where(unknown, 160, 0)).astype(np.uint8)
        ax.imshow(np.flipud(img), cmap="gray", extent=[ox, ox+W*res, oy, oy+H*res], vmin=0, vmax=255)
        ax.imshow(np.flipud(np.where(reach, 1.0, np.nan)), cmap="Greens", alpha=0.25, extent=[ox, ox+W*res, oy, oy+H*res], vmin=0, vmax=2)
        xs, ys = zip(*route); ax.plot(xs, ys, "r-", lw=1.2); ax.plot(xs, ys, "ro", ms=3)
        ax.plot(xs[0], ys[0], "bs", ms=8, label="start"); ax.legend(loc="upper right")
        ax.set_title(f"coverage route: {len(route)} waypoints, {tour_len(order):.1f} m"); ax.set_aspect("equal"); ax.grid(alpha=0.2)
        plt.tight_layout(); plt.savefig(a.png, dpi=90)


if __name__ == "__main__":
    main()
