"""Build maps/gamefield.{pgm,yaml} from the arena mesh: a horizontal slice at the lidar scan height.
Floor is probed with rays cast DOWN from 0.40 m, so floor under overhead structures (the bridge) counts as drivable."""
import trimesh, numpy as np, yaml, sys
from scipy import ndimage
from PIL import Image
pkg = sys.argv[1]
m = trimesh.load(f"{pkg}/models/gamefield_arena/meshes/Gamefield.stl"); m.apply_scale(0.001)
pose = np.array([0.135886, 6.443587, 0.789877]); m.apply_translation(pose)       # <pose> of the arena link in model.sdf
b = m.bounds
RES, margin = 0.02, 0.30
x0, y0 = b[0][0]-margin, b[0][1]-margin
W = int(np.ceil((b[1][0]+margin-x0)/RES)); H = int(np.ceil((b[1][1]+margin-y0)/RES))
# floor (where the robot can stand): top-down raycast, 1 cm sampling
SUB = 2; fine = RES/SUB
xs = x0 + (np.arange(W*SUB)+0.5)*fine; ys = y0 + (np.arange(H*SUB)+0.5)*fine
X, Y = np.meshgrid(xs, ys)
origins = np.column_stack([X.ravel(), Y.ravel(), np.full(X.size, 0.40)])   # start below overhead structures (bridge underside 0.53 m)
loc, idx, _ = m.ray.intersects_location(origins, np.tile([[0,0,-1.0]], (len(origins),1)), multiple_hits=False)
Z = np.full(X.size, np.nan); Z[idx] = loc[:,2]; Z = Z.reshape(H*SUB, W*SUB)
floor = ((~np.isnan(Z)) & (Z <= 0.12)).reshape(H, SUB, W, SUB).all(axis=(1,3))
# obstacles: what the lidar sees — slice at scan height (0.22 m above the floor at 0.05 / pads at 0.08), +-3 cm
occ = np.zeros((H, W), bool)
for zc in (0.27, 0.30, 0.24, 0.33):
    sec = m.section(plane_origin=[0,0,zc], plane_normal=[0,0,1])
    if sec is None: continue
    for poly in sec.discrete:
        for a, c in zip(poly[:-1], poly[1:]):
            pts = np.linspace(a, c, max(2, int(np.linalg.norm(c-a)/0.004)))
            j = ((pts[:,0]-x0)/RES).astype(int); i = ((pts[:,1]-y0)/RES).astype(int)
            ok = (i>=0)&(i<H)&(j>=0)&(j<W); occ[i[ok], j[ok]] = True
# solid-fill small enclosed shapes (pillars, posts)
pockets = ndimage.binary_fill_holes(ndimage.binary_dilation(occ)) & ~occ
lab, n = ndimage.label(pockets); sizes = ndimage.sum(pockets, lab, range(1, n+1))
occ |= np.isin(lab, [k+1 for k, s in enumerate(sizes) if s < 0.08/(RES*RES)])
free = floor & ~occ
img = np.full((H, W), 205, np.uint8); img[free] = 254; img[occ] = 0
Image.fromarray(np.flipud(img)).save(f"{pkg}/maps/gamefield.pgm")
yaml.safe_dump({"image": "gamefield.pgm", "mode": "trinary", "resolution": RES,
                "origin": [round(float(x0),3), round(float(y0),3), 0.0], "negate": 0,
                "occupied_thresh": 0.65, "free_thresh": 0.196}, open(f"{pkg}/maps/gamefield.yaml","w"), sort_keys=False)
print(f"map {W}x{H} @ {RES} m, origin ({x0:.3f},{y0:.3f}); occupied {occ.mean()*100:.1f}%, free {free.mean()*100:.1f}%")
dist = ndimage.distance_transform_edt(free) * RES
wps = [(-0.54, 1.15), (-2.65, 1.15), (-2.65, -0.45), (-0.85, -1.10), (0.92, -0.49), (2.55, -1.10), (2.55, 1.15), (1.50, 0.55)]
for x,y in wps: print(f"  wp ({x:5.2f},{y:5.2f}) clearance {dist[int((y-y0)/RES), int((x-x0)/RES)]:.2f} m")
if len(sys.argv) > 2:
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(12,6.2)); ax.imshow(img, cmap="gray", extent=[x0, x0+W*RES, y0, y0+H*RES], vmin=0, vmax=255, origin="lower")
    ax.plot([w[0] for w in wps]+[wps[0][0]], [w[1] for w in wps]+[wps[0][1]], "r--", lw=1)
    for k,(x,y) in enumerate(wps): ax.plot(x,y,"ro",ms=7); ax.annotate(str(k+1),(x+0.05,y+0.05),color="red",fontsize=10,weight="bold")
    ax.set_title("maps/gamefield.pgm — arena sliced at lidar height (2 cm cells) + patrol waypoints"); ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)"); ax.grid(alpha=0.25)
    plt.tight_layout(); plt.savefig(sys.argv[2], dpi=90)
