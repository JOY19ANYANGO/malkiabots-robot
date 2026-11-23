"""Build a ROS 2 description package (URDF/xacro + meshes) from Main_Assembly.step."""
import math, os, sys, json
import numpy as np
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TDocStd import TDocStd_Document
from OCP.TCollection import TCollection_ExtendedString
from OCP.XCAFDoc import XCAFDoc_DocumentTool, XCAFDoc_ColorSurf, XCAFDoc_ColorGen
from OCP.TDF import TDF_Label
from OCP.collections import Sequence_TDF_Label
from OCP.TDataStd import TDataStd_Name
from OCP.TopLoc import TopLoc_Location
from OCP.BRepBndLib import BRepBndLib
from OCP.Bnd import Bnd_Box
from OCP.GProp import GProp_GProps
from OCP.BRepGProp import BRepGProp
from OCP.IFSelect import IFSelect_RetDone
from OCP.gp import gp_Trsf, gp_Vec, gp_Ax1, gp_Pnt, gp_Dir, gp_Quaternion
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.BRep import BRep_Builder
from OCP.TopoDS import TopoDS_Compound, TopoDS
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.StlAPI import StlAPI_Writer
from OCP.Quantity import Quantity_Color
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_FACE
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.GeomAbs import GeomAbs_Plane

STEP = "/mnt/user-data/uploads/Main_Assembly.step"
PKG = "cramped_robot_description"
ROBOT = "cramped_robot"
OUT = f"/home/claude/{PKG}"
for d in ("urdf", "meshes/visual", "launch", "rviz", "config"):
    os.makedirs(f"{OUT}/{d}", exist_ok=True)

# ---------------------------------------------------------------- load STEP
doc = TDocStd_Document(TCollection_ExtendedString("doc"))
rdr = STEPCAFControl_Reader(); rdr.SetNameMode(True); rdr.SetColorMode(True)
assert rdr.ReadFile(STEP) == IFSelect_RetDone
rdr.Transfer(doc)
st = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
ct = XCAFDoc_DocumentTool.ColorTool_s(doc.Main())

def name_of(lbl):
    n = TDataStd_Name()
    return n.Get().ToExtString() if lbl.FindAttribute(TDataStd_Name.GetID_s(), n) else "?"

leaves = []  # (path_names, located shape)
def walk(lbl, loc, path):
    ref = TDF_Label(); target = ref if st.GetReferredShape_s(lbl, ref) else lbl
    nm = name_of(lbl); this_loc = loc.Multiplied(st.GetLocation_s(lbl))
    if st.IsAssembly_s(target):
        comps = Sequence_TDF_Label(); st.GetComponents_s(target, comps)
        for i in range(1, comps.Length() + 1):
            walk(comps.Value(i), this_loc, path + [nm])
    else:
        leaves.append((path + [nm], st.GetShape_s(target).Located(this_loc)))
roots = Sequence_TDF_Label(); st.GetFreeShapes(roots)
for i in range(1, roots.Length() + 1):
    walk(roots.Value(i), TopLoc_Location(), [])

# ---------------------------------------------------------------- helpers
def volume(shp):
    g = GProp_GProps(); BRepGProp.VolumeProperties_s(shp, g); return g.Mass()
def bbox(shp):
    b = Bnd_Box(); BRepBndLib.Add_s(shp, b, False)
    mn, mx = b.CornerMin(), b.CornerMax()
    return np.array([mn.X(), mn.Y(), mn.Z()]), np.array([mx.X(), mx.Y(), mx.Z()])
def com(shp):
    g = GProp_GProps(); BRepGProp.VolumeProperties_s(shp, g); c = g.CentreOfMass()
    return np.array([c.X(), c.Y(), c.Z()])
def color_of(shp):
    c = Quantity_Color()
    if ct.GetInstanceColor(shp, XCAFDoc_ColorSurf, c) or ct.GetInstanceColor(shp, XCAFDoc_ColorGen, c):
        return (round(c.Red(), 3), round(c.Green(), 3), round(c.Blue(), 3))
    return None

# CAD -> URDF (REP-103): x_urdf = -y_cad (front = -Y in CAD), y_urdf = x_cad, z_urdf = z_cad  == Rz(+90 deg)
R_cu = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]], dtype=float)
def cad2urdf(p): return R_cu @ np.asarray(p, float)

def to_local(shp, origin_cad):
    """Return shape expressed in a link frame located at origin_cad (CAD coords) and rotated to URDF axes."""
    t1 = gp_Trsf(); t1.SetTranslation(gp_Vec(-origin_cad[0], -origin_cad[1], -origin_cad[2]))
    t2 = gp_Trsf(); t2.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)), math.pi / 2)
    t = t2.Multiplied(t1)
    return BRepBuilderAPI_Transform(shp, t, True).Shape()

def mass_props(parts):
    """parts: list of (shape_in_link_frame_mm, density kg/m^3). Returns mass, com(m), inertia about com (kg m^2)."""
    M = 0.0; first = np.zeros(3); I0 = np.zeros((3, 3))
    for shp, rho in parts:
        g = GProp_GProps(); BRepGProp.VolumeProperties_s(shp, g)
        v = g.Mass() * 1e-9                                  # m^3
        m = v * rho
        c = np.array([g.CentreOfMass().X(), g.CentreOfMass().Y(), g.CentreOfMass().Z()]) * 1e-3
        Im = g.MatrixOfInertia()                              # about the part COM, per unit density, mm^5
        Ic_part = np.array([[Im.Value(i, j) for j in (1, 2, 3)] for i in (1, 2, 3)]) * 1e-15 * rho  # kg m^2, about the part's COM
        I0 += Ic_part + m * (np.dot(c, c) * np.eye(3) - np.outer(c, c))                          # shift to link origin
        M += m; first += m * c
    C = first / M
    # parallel-axis: shift from origin to combined COM
    Ic = I0 - M * (np.dot(C, C) * np.eye(3) - np.outer(C, C))
    return M, C, Ic

def export_stl(shp, path, lin=0.15, ang=0.35):
    # scale mm -> m so meshes load with scale 1 1 1
    t = gp_Trsf(); t.SetScale(gp_Pnt(0, 0, 0), 1e-3)
    s = BRepBuilderAPI_Transform(shp, t, True).Shape()
    BRepMesh_IncrementalMesh(s, lin * 1e-3, False, ang, True)
    w = StlAPI_Writer(); w.ASCIIMode = False
    assert w.Write(s, path), path

def compound(shapes):
    b = BRep_Builder(); c = TopoDS_Compound(); b.MakeCompound(c)
    for s in shapes: b.Add(c, s)
    return c

# ---------------------------------------------------------------- group parts into links
chassis_parts, motor_asms = [], {}
for path, shp in leaves:
    if len(path) == 2:
        chassis_parts.append((path[-1], shp))
    else:
        motor_asms.setdefault(path[1], []).append((path[-1], shp))

# ---- design change requested by the user: the inclined "Middle Plane" must not protrude above the front-cabin roof
#      (it would sit in the lidar's field of view). Cut everything above the roof's top face off that solid.
from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut
from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox
TRIM_MIDDLE_PLANE_TO_ROOF = True
_roof_mn, _roof_mx = bbox([sh for n, sh in chassis_parts if n == "top plane"][0])
roof_top_cad = _roof_mx[2]                                                    # mm, CAD z of the roof's top face
if TRIM_MIDDLE_PLANE_TO_ROOF:
    _cutter = BRepPrimAPI_MakeBox(gp_Pnt(-1000, -1000, roof_top_cad), gp_Pnt(1000, 1000, 1000)).Shape()
    for _i, (n, sh) in enumerate(chassis_parts):
        if n == "Middle Plane":
            _orig_v = volume(sh)
            _cut = BRepAlgoAPI_Cut(sh, _cutter); _cut.Build(); assert _cut.IsDone()
            _trimmed = _cut.Shape()
            chassis_parts[_i] = (n, _trimmed)
            print(f"Middle Plane trimmed at z={roof_top_cad:.1f} mm: volume {_orig_v:.0f} -> {volume(_trimmed):.0f} mm3, "
                  f"top now at z={bbox(_trimmed)[1][2]:.1f} mm (roof {roof_top_cad:.1f})")

DENS = dict(acrylic=1180.0, alu=2700.0, steel=7850.0, brass=8500.0, rubber=1100.0, abs=1050.0, motor=2870.0)
#  motor: effective density giving ~0.19 kg for the 37 mm gearmotor body+gearbox (typical 37D gearmotor ~0.2 kg)

# chassis colours (from STEP)
chassis_colors = {n: color_of(s) for n, s in chassis_parts}

# corner naming from the motor-assembly COM (CAD): -y = front, +x = left
corners = {}
for asm, parts in motor_asms.items():
    c = np.mean([com(s) for _, s in parts], axis=0)
    key = ("front" if c[1] < 0 else "rear") + "_" + ("left" if c[0] > 0 else "right")
    corners[key] = parts
assert set(corners) == {"front_left", "front_right", "rear_left", "rear_right"}, corners.keys()

def split_motor_asm(parts):
    motor, wheel = [], []
    for n, s in parts:
        v = volume(s)
        if n.startswith("Motor Mount"):                           motor.append((n, s, DENS["alu"]))
        elif n.startswith("MOTOR") and v > 5000:                  motor.append((n, s, DENS["motor"]))
        elif n.startswith("MOTOR") and 500 < v < 530:             motor.append((n, s, DENS["steel"]))   # bearing boss
        elif n.startswith("MOTOR"):                               wheel.append((n, s, DENS["steel"]))   # 6 mm D-shaft
        elif "coupling" in n:                                     wheel.append((n, s, DENS["brass"]))
        elif n.startswith("Cramped Robot Wheel") and v > 30000:   wheel.append((n, s, DENS["rubber"]))  # tyre
        elif n.startswith("Cramped Robot Wheel"):                 wheel.append((n, s, DENS["abs"]))     # hub
        else: raise RuntimeError(n)
    return motor, wheel

# ---------------------------------------------------------------- key dimensions (CAD, mm) from the FRONT-LEFT corner
fl_motor, fl_wheel = split_motor_asm(corners["front_left"])
tyre = [s for n, s, _ in fl_wheel if volume(s) > 30000][0]
tmn, tmx = bbox(tyre)
wheel_radius_mm = (tmx[2] - tmn[2]) / 2
wheel_width_mm = tmx[0] - tmn[0]
axle_z_mm = (tmn[2] + tmx[2]) / 2
wheel_y_offset_mm = (tmn[0] + tmx[0]) / 2                     # lateral wheel-centre (CAD x)
motor_body = max(((n, s) for n, s, _ in fl_motor if n.startswith("MOTOR")), key=lambda t: volume(t[1]))[1]
mbmn, mbmx = bbox(motor_body)
motor_face_mm = mbmx[0]                                        # outboard face of gearbox (CAD x)
mount = [s for n, s, _ in fl_motor if n.startswith("Motor Mount")][0]
mtmn, mtmx = bbox(mount)
motor_x_cad = (mtmn[1] + mtmx[1]) / 2                          # longitudinal position of motor axis (CAD y), = 65.0
motor_all = compound([s for _, s, _ in fl_motor]); mamn, mamx = bbox(motor_all)
bot = [s for n, s in chassis_parts if n.startswith("Bottom chassis")][0]; bmn, bmx = bbox(bot)
ground_z_mm = tmn[2]

wheelbase_mm = 2 * abs(motor_x_cad)
track_mm = 2 * abs(wheel_y_offset_mm)
print(f"wheel r={wheel_radius_mm:.2f} w={wheel_width_mm:.2f} axle_z={axle_z_mm:.2f} track={track_mm:.1f} wheelbase={wheelbase_mm:.1f} motor_face={motor_face_mm:.1f}")

# ---------------------------------------------------------------- link frames (CAD coords, mm)
# base_link: CAD origin = centre of the bottom face of the bottom chassis plate.
frames_cad = {"base_link": np.zeros(3)}
sign = {"front_left": (-1, +1), "front_right": (-1, -1), "rear_left": (+1, +1), "rear_right": (+1, -1)}  # (y_cad, x_cad)
for k, (sy, sx) in sign.items():
    frames_cad[f"{k}_motor_link"] = np.array([sx * motor_face_mm, sy * abs(motor_x_cad), axle_z_mm])
    frames_cad[f"{k}_wheel_link"] = np.array([sx * abs(wheel_y_offset_mm), sy * abs(motor_x_cad), axle_z_mm])

# ---------------------------------------------------------------- mass properties + meshes
# base_link
def plate_rho(n): return DENS["alu"] if n.startswith("bracket") else DENS["acrylic"]   # coloured plates = acrylic, dark L-brackets = aluminium
base_local = [(to_local(s, frames_cad["base_link"]), plate_rho(n)) for n, s in chassis_parts]
motor_parts_all = []                                   # (corner, name, shape, density) — fixed to the chassis
for k in corners:
    m, _ = split_motor_asm(corners[k])
    motor_parts_all += [(k, n, s, rho) for n, s, rho in m]
base_local += [(to_local(s, frames_cad["base_link"]), rho) for _, _, s, rho in motor_parts_all]
base_M, base_C, base_I = mass_props(base_local)
chassis_only_M = mass_props([(to_local(s, frames_cad["base_link"]), plate_rho(n)) for n, s in chassis_parts])[0]
export_stl(compound([s for s, _ in base_local]), f"{OUT}/meshes/visual/base_link.stl")

# motor + wheel: geometry is identical at all four corners; export from front-left, mirror by yaw for the right side
mot_local = [(to_local(s, frames_cad["front_left_motor_link"]), rho) for _, s, rho in fl_motor]
# wheel mesh: centre on the tyre's own axis (the CAD wheels sit 0.4 mm off the motor axis; the joint uses the motor axis)
whl_origin_cad = np.array([(tmn[0] + tmx[0]) / 2, (tmn[1] + tmx[1]) / 2, (tmn[2] + tmx[2]) / 2])
whl_local = [(to_local(s, whl_origin_cad), rho) for _, s, rho in fl_wheel]
mot_M, mot_C, mot_I = mass_props(mot_local)
whl_M, whl_C, whl_I = mass_props(whl_local)
export_stl(compound([s for s, _ in whl_local]), f"{OUT}/meshes/visual/wheel.stl", lin=0.25, ang=0.6)
# the tyre's tread pattern is made of hundreds of tiny B-spline faces -> decimate the wheel mesh for RViz
import trimesh
_w = trimesh.load(f"{OUT}/meshes/visual/wheel.stl")
_w.simplify_quadric_decimation(face_count=9000).export(f"{OUT}/meshes/visual/wheel.stl")

# sanity: all four corners have identical mass props
for k in corners:
    m, w = split_motor_asm(corners[k])
    assert abs(mass_props([(to_local(s, frames_cad[f"{k}_motor_link"]), r) for _, s, r in m])[0] - mot_M) < 1e-6
    assert abs(mass_props([(to_local(s, frames_cad[f"{k}_wheel_link"]), r) for _, s, r in w])[0] - whl_M) < 1e-6

# ---------------------------------------------------------------- collision primitives for base_link (URDF frame, m)
def box_from_bbox(mn, mx):
    c = cad2urdf((mn + mx) / 2) * 1e-3
    size_cad = (mx - mn) * 1e-3
    size = np.abs(R_cu @ size_cad)
    return c, size
coll_boxes = []   # (comment, center, size, rpy)
seen = set()
for n, s in chassis_parts:
    if n.startswith("bracket") or n == "Middle Plane": continue
    mn, mx = bbox(s); c, sz = box_from_bbox(mn, mx)
    coll_boxes.append((n, c, sz, (0, 0, 0)))
# Middle Plane: single inclined 5 mm plate -> rotated box
mp = [s for n, s in chassis_parts if n == "Middle Plane"][0]
faces = []
ex = TopExp_Explorer(mp, TopAbs_FACE)
while ex.More():
    f = TopoDS.Face(ex.Current()); ex.Next()
    g = GProp_GProps(); BRepGProp.SurfaceProperties_s(f, g)
    ad = BRepAdaptor_Surface(f)
    if ad.GetType() == GeomAbs_Plane: faces.append((g.Mass(), f, ad.Plane().Axis().Direction()))
faces.sort(key=lambda t: -t[0])
big = faces[0]
n_cad = np.array([big[2].X(), big[2].Y(), big[2].Z()]); n_cad *= np.sign(n_cad[2]) if n_cad[2] else 1
fmn, fmx = bbox(big[1])
plate_len = math.hypot(fmx[1] - fmn[1], fmx[2] - fmn[2])       # along the slope (in the CAD yz plane)
plate_w = fmx[0] - fmn[0]
mpmn, mpmx = bbox(mp)
plate_t = volume(mp) / big[0]                                  # slab thickness = volume / large-face area (5 mm)
n_u = cad2urdf(n_cad)                                          # normal in URDF frame
pitch = math.atan2(n_u[0], n_u[2])                            # Ry(pitch) maps z -> normal
mp_c = cad2urdf(com(mp)) * 1e-3
coll_boxes.append(("Middle Plane (inclined plate)", mp_c, np.array([plate_len, plate_w, plate_t]) * 1e-3, (0, pitch, 0)))

# motor assemblies -> collision boxes in the base frame (mount + gearmotor body, per corner)
for k in ("front_left", "front_right", "rear_left", "rear_right"):
    parts = [(n, sh) for kk, n, sh, _ in motor_parts_all if kk == k]
    mnt = [sh for n, sh in parts if n.startswith("Motor Mount")][0]
    body = compound([sh for n, sh in parts if n.startswith("MOTOR") and volume(sh) > 5000])
    for label, shp in ((f"{k} motor mount", mnt), (f"{k} gearmotor", body)):
        mn, mx = bbox(shp); c, sz = box_from_bbox(mn, mx)
        coll_boxes.append((label, c, sz, (0, 0, 0)))

# sensor placement (URDF frame, m)
top_mn, top_mx = bbox([sh for n, sh in chassis_parts if n == "top plane"][0])        # front-cabin roof
lidar_r, lidar_h = 0.030, 0.035                                                     # compact 2-D lidar puck
lidar_z_scan = 0.025                                                                # scan plane above the puck base
roof_z = top_mx[2] * 1e-3
lidar_xyz = np.array([cad2urdf((top_mn + top_mx) / 2)[0] * 1e-3, 0.0, roof_z])   # centre of the front-cabin roof
# clearance: nothing of the mid-plate may rise above the roof, and it must stay below the scan plane
plate_top_z = mpmx[2] * 1e-3
assert plate_top_z <= roof_z + 1e-4, f"mid-plate protrudes {plate_top_z-roof_z:.4f} m above the roof"
assert plate_top_z < roof_z + lidar_z_scan, "mid-plate reaches the lidar scan plane"
plate_front_x, plate_front_z = cad2urdf(mpmn)[0] * 1e-3, plate_top_z             # plate's front (highest) edge, URDF
imu_xyz = np.array([0.0, 0.0, bmx[2] * 1e-3])                                        # centre of the bottom-plate top face
imu_box = (0.020, 0.020, 0.005)

# ---------------------------------------------------------------- numbers for the xacro
def f(x, nd=6): return f"{x:.{nd}f}".rstrip("0").rstrip(".") if abs(x) > 1e-12 else "0"
def inertia_per_kg(I, M):
    return {k: I[i, j] / M for k, (i, j) in dict(ixx=(0, 0), ixy=(0, 1), ixz=(0, 2), iyy=(1, 1), iyz=(1, 2), izz=(2, 2)).items()}
base_ipk, whl_ipk = inertia_per_kg(base_I, base_M), inertia_per_kg(whl_I, whl_M)

wheel_x = abs(motor_x_cad) * 1e-3
wheel_y = abs(wheel_y_offset_mm) * 1e-3
motor_y = motor_face_mm * 1e-3
axle_z = axle_z_mm * 1e-3
ground_z = ground_z_mm * 1e-3
whl_r = wheel_radius_mm * 1e-3
whl_w = wheel_width_mm * 1e-3

def col(c):  # rgba string from STEP colour
    return f"{c[0]:.3f} {c[1]:.3f} {c[2]:.3f} 1.0"

summary = dict(
    wheel_radius_m=whl_r, wheel_width_m=whl_w, track_m=2 * wheel_y, wheelbase_m=2 * wheel_x, axle_height_from_base_m=axle_z,
    base_to_ground_m=-ground_z, base_mass_kg=base_M, chassis_only_mass_kg=chassis_only_M, motor_asm_mass_kg=mot_M, wheel_link_mass_kg=whl_M,
    total_mass_kg=base_M + 4 * whl_M, base_com_m=base_C.tolist(), wheel_com_m=whl_C.tolist(),
    base_inertia_kgm2=base_I.tolist(), wheel_inertia_kgm2=whl_I.tolist(), lidar_xyz_m=lidar_xyz.tolist(), imu_xyz_m=imu_xyz.tolist(), plate_front_edge_xz_m=[plate_front_x, plate_front_z], lidar_scan_z_footprint_m=roof_z + lidar_z_scan - ground_z,
    chassis_bbox_urdf_m=[cad2urdf(v).tolist() for v in bbox(compound([s for _, s in chassis_parts]))],
    middle_plane_pitch_rad=pitch, chassis_parts=[n for n, _ in chassis_parts],
)
json.dump(summary, open("/home/claude/summary.json", "w"), indent=1)
print(json.dumps({k: v for k, v in summary.items() if not isinstance(v, list)}, indent=1))

# ============================================================================ XACRO
xacro_main = f"""<?xml version="1.0"?>
<!--
  {ROBOT} — 4-wheel skid-steer mobile robot
  Generated from Main_Assembly.step (Onshape, STEP AP242) on 2026-09-23.

  Frame conventions (ROS REP-103): x forward, y left, z up.
  CAD -> URDF mapping: x_urdf = -y_cad, y_urdf = x_cad, z_urdf = z_cad  (the CAD front is -Y).
  base_link origin = centre of the BOTTOM face of the bottom chassis plate (the CAD origin).
  base_footprint sits on the ground plane, {f(-ground_z, 4)} m below base_link.

  Design change vs. the STEP: the inclined "Middle Plane" is cut off at roof height (z = {f(roof_top_cad*1e-3,3)} m in base_link)
  so it cannot protrude into the lidar's field of view. Trim the physical plate the same way, or set
  TRIM_MIDDLE_PLANE_TO_ROOF = False in scripts/build_from_step.py to restore it.

  Links: base_footprint, base_link (chassis + brackets + 4 motor mounts + 4 gearmotors, rigid),
         4 wheel links (continuous joints), lidar_link (front-cabin roof), imu_link (lower-cabin floor).

  Mesh units: metres (pre-scaled at export). Visual meshes come straight from the STEP solids;
  collision uses analytic primitives derived from the same solids' bounding boxes.

  Mass: total_mass (2.0 kg, user estimate) is distributed in proportion to the CAD-derived link masses.
  CAD-derived masses (volume x density; inertia tensors scale with mass):
    chassis plates (5 mm)                     : acrylic, 1180 kg/m3   -> {f(chassis_only_M, 3)} kg incl. brackets
    L-brackets / motor mounts                 : aluminium, 2700 kg/m3
    4 x 37 mm gearmotor                        : ~{f(mot_M - volume(mount)*1e-9*2700, 3)} kg each (typical 37D gearmotor)
    wheel (rubber tyre + ABS hub) + brass coupling + steel D-shaft : ~{f(whl_M, 3)} kg
  Motor limits assume a 37D 12 V gearmotor, ~100 rpm no-load, ~1 N·m stall at the wheel.
-->
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="{ROBOT}">

  <!-- ============================== arguments ============================== -->
  <xacro:arg name="prefix"            default=""/>
  <xacro:arg name="use_sim"           default="false"/>   <!-- add Gazebo (gz-sim) plugins + sensors -->
  <xacro:arg name="use_ros2_control"  default="false"/>   <!-- add ros2_control hardware block -->
  <xacro:arg name="mesh_pkg"          default="package://{PKG}/meshes/visual"/>

  <xacro:property name="prefix"   value="$(arg prefix)"/>
  <xacro:property name="mesh_pkg" value="$(arg mesh_pkg)"/>
  <xacro:property name="PI"       value="3.14159265358979"/>

  <!-- ============================== geometry (from CAD) ============================== -->
  <xacro:property name="wheel_radius"    value="{f(whl_r, 4)}"/>   <!-- m, tyre OD {f(2*wheel_radius_mm,1)} mm -->
  <xacro:property name="wheel_width"     value="{f(whl_w, 4)}"/>   <!-- m -->
  <xacro:property name="wheel_x"         value="{f(wheel_x, 4)}"/>   <!-- m, half wheelbase (wheelbase {f(2*wheel_x*1000,1)} mm) -->
  <xacro:property name="wheel_y"         value="{f(wheel_y, 4)}"/>   <!-- m, half track (track {f(2*wheel_y*1000,1)} mm) -->
  <xacro:property name="axle_z"          value="{f(axle_z, 4)}"/>   <!-- m, wheel axle height relative to base_link -->
  <xacro:property name="ground_offset"   value="{f(-ground_z, 4)}"/>   <!-- m, base_link height above ground -->

  <!-- ============================== sensors ============================== -->
  <xacro:property name="lidar_x"      value="{f(lidar_xyz[0],4)}"/>   <!-- m, centre of the front-cabin roof -->
  <xacro:property name="lidar_y"      value="{f(lidar_xyz[1],4)}"/>
  <xacro:property name="lidar_base_z" value="{f(lidar_xyz[2],4)}"/>   <!-- m, roof top face (puck base) -->
  <xacro:property name="lidar_radius" value="{lidar_r}"/>
  <xacro:property name="lidar_height" value="{lidar_h}"/>
  <xacro:property name="lidar_scan_z" value="{lidar_z_scan}"/>   <!-- m, scan plane above the lidar base; lidar_link sits on the scan plane -->
  <xacro:property name="imu_xyz"      value="{f(imu_xyz[0],4)} {f(imu_xyz[1],4)} {f(imu_xyz[2],4)}"/>  <!-- m, centre of the lower-cabin floor (bottom-plate top face) -->

  <!-- ============================== masses ============================== -->
  <!-- total_mass is the measured/estimated weight of the whole robot. It is distributed over the links in
       proportion to the CAD-derived masses below (volume x density), so COM positions and inertia shapes
       stay CAD-accurate while the total matches the scale. Edit total_mass, or the per-link CAD values. -->
  <xacro:property name="total_mass"      value="2.0"/>                 <!-- kg, whole robot incl. lidar + IMU -->
  <xacro:property name="base_mass_cad"   value="{f(base_M, 4)}"/>   <!-- kg, acrylic plates + alu brackets/mounts + 4 gearmotors -->
  <xacro:property name="wheel_mass_cad"  value="{f(whl_M, 4)}"/>   <!-- kg, each wheel + coupling + shaft -->
  <xacro:property name="lidar_mass_cad"  value="0.170"/>
  <xacro:property name="imu_mass_cad"    value="0.010"/>
  <xacro:property name="mass_scale"  value="${{total_mass / (base_mass_cad + 4*wheel_mass_cad + lidar_mass_cad + imu_mass_cad)}}"/>
  <xacro:property name="base_mass"   value="${{base_mass_cad  * mass_scale}}"/>
  <xacro:property name="wheel_mass"  value="${{wheel_mass_cad * mass_scale}}"/>
  <xacro:property name="lidar_mass"  value="${{lidar_mass_cad * mass_scale}}"/>
  <xacro:property name="imu_mass"    value="${{imu_mass_cad   * mass_scale}}"/>

  <!-- ============================== actuator limits ============================== -->
  <xacro:property name="wheel_max_effort"   value="1.0"/>    <!-- N·m -->
  <xacro:property name="wheel_max_velocity" value="10.5"/>   <!-- rad/s (~100 rpm) -->
  <xacro:property name="wheel_damping"      value="0.01"/>
  <xacro:property name="wheel_friction"     value="0.05"/>

  <!-- ============================== materials ============================== -->
  <material name="chassis_yellow"><color rgba="{col(chassis_colors['Bottom chassis001'])}"/></material>
  <material name="tyre_black"><color rgba="0.10 0.10 0.10 1.0"/></material>
  <material name="sensor_grey"><color rgba="0.35 0.35 0.38 1.0"/></material>

  <!-- ============================== macros ============================== -->
  <xacro:macro name="inertial_scaled" params="mass cx cy cz ixx ixy ixz iyy iyz izz">
    <!-- inertia given per kg (from CAD geometry, uniform density per part); scales with the chosen mass -->
    <inertial>
      <origin xyz="${{cx}} ${{cy}} ${{cz}}" rpy="0 0 0"/>
      <mass value="${{mass}}"/>
      <inertia ixx="${{mass*ixx}}" ixy="${{mass*ixy}}" ixz="${{mass*ixz}}"
               iyy="${{mass*iyy}}" iyz="${{mass*iyz}}" izz="${{mass*izz}}"/>
    </inertial>
  </xacro:macro>

  <xacro:macro name="box_inertial" params="mass x y z">
    <inertial>
      <mass value="${{mass}}"/>
      <inertia ixx="${{mass/12*(y*y+z*z)}}" ixy="0" ixz="0" iyy="${{mass/12*(x*x+z*z)}}" iyz="0" izz="${{mass/12*(x*x+y*y)}}"/>
    </inertial>
  </xacro:macro>

  <xacro:macro name="cylinder_inertial" params="mass r h">
    <inertial>
      <mass value="${{mass}}"/>
      <inertia ixx="${{mass/12*(3*r*r+h*h)}}" ixy="0" ixz="0" iyy="${{mass/12*(3*r*r+h*h)}}" iyz="0" izz="${{mass/2*r*r}}"/>
    </inertial>
  </xacro:macro>

  <!--
    One wheel, driven directly from base_link (the gearmotor is part of base_link).
    side_sign: +1 left / -1 right   (the right wheel is the left wheel rotated 180 deg about z: mesh yaw=pi,
                                     COM x,y and the xz/yz inertia products change sign)
    front_sign: +1 front / -1 rear
  -->
  <xacro:macro name="wheel" params="name side_sign front_sign">
    <link name="${{prefix}}${{name}}_wheel_link">
      <visual>
        <origin xyz="0 0 0" rpy="0 0 ${{0 if side_sign > 0 else PI}}"/>
        <geometry><mesh filename="${{mesh_pkg}}/wheel.stl"/></geometry>
        <material name="tyre_black"/>
      </visual>
      <collision>
        <origin xyz="0 0 0" rpy="${{PI/2}} 0 0"/>
        <geometry><cylinder radius="${{wheel_radius}}" length="${{wheel_width}}"/></geometry>
      </collision>
      <xacro:inertial_scaled mass="${{wheel_mass}}"
        cx="${{side_sign*{f(whl_C[0],6)}}}" cy="${{side_sign*{f(whl_C[1],6)}}}" cz="{f(whl_C[2],6)}"
        ixx="{f(whl_ipk['ixx'],8)}" ixy="{f(whl_ipk['ixy'],8)}" ixz="${{side_sign*{f(whl_ipk['ixz'],8)}}}"
        iyy="{f(whl_ipk['iyy'],8)}" iyz="${{side_sign*{f(whl_ipk['iyz'],8)}}}" izz="{f(whl_ipk['izz'],8)}"/>
    </link>
    <joint name="${{prefix}}${{name}}_wheel_joint" type="continuous">
      <parent link="${{prefix}}base_link"/>
      <child  link="${{prefix}}${{name}}_wheel_link"/>
      <origin xyz="${{front_sign*wheel_x}} ${{side_sign*wheel_y}} ${{axle_z}}" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
      <limit effort="${{wheel_max_effort}}" velocity="${{wheel_max_velocity}}"/>
      <dynamics damping="${{wheel_damping}}" friction="${{wheel_friction}}"/>
    </joint>
    <xacro:if value="$(arg use_sim)">
      <gazebo reference="${{prefix}}${{name}}_wheel_link">
        <mu1>1.0</mu1><mu2>1.0</mu2>
        <kp>1e6</kp><kd>100</kd>
        <minDepth>0.001</minDepth>
        <fdir1>1 0 0</fdir1>
      </gazebo>
    </xacro:if>
  </xacro:macro>

  <!-- ============================== base ============================== -->
  <link name="${{prefix}}base_footprint"/>
  <joint name="${{prefix}}base_footprint_joint" type="fixed">
    <parent link="${{prefix}}base_footprint"/>
    <child  link="${{prefix}}base_link"/>
    <origin xyz="0 0 ${{ground_offset}}" rpy="0 0 0"/>
  </joint>

  <link name="${{prefix}}base_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry><mesh filename="${{mesh_pkg}}/base_link.stl"/></geometry>
      <material name="chassis_yellow"/>
    </visual>
"""
for label, c, sz, rpy in coll_boxes:
    xacro_main += f"""    <collision><!-- {label} -->
      <origin xyz="{f(c[0],5)} {f(c[1],5)} {f(c[2],5)}" rpy="{f(rpy[0],5)} {f(rpy[1],5)} {f(rpy[2],5)}"/>
      <geometry><box size="{f(sz[0],5)} {f(sz[1],5)} {f(sz[2],5)}"/></geometry>
    </collision>
"""
xacro_main += f"""    <xacro:inertial_scaled mass="${{base_mass}}"
      cx="{f(base_C[0],6)}" cy="{f(base_C[1],6)}" cz="{f(base_C[2],6)}"
      ixx="{f(base_ipk['ixx'],8)}" ixy="{f(base_ipk['ixy'],8)}" ixz="{f(base_ipk['ixz'],8)}"
      iyy="{f(base_ipk['iyy'],8)}" iyz="{f(base_ipk['iyz'],8)}" izz="{f(base_ipk['izz'],8)}"/>
  </link>

  <!-- ============================== wheels ============================== -->
  <xacro:wheel name="front_left"  side_sign="1"  front_sign="1"/>
  <xacro:wheel name="front_right" side_sign="-1" front_sign="1"/>
  <xacro:wheel name="rear_left"   side_sign="1"  front_sign="-1"/>
  <xacro:wheel name="rear_right"  side_sign="-1" front_sign="-1"/>

  <!-- ============================== lidar: top of the front cabin ============================== -->
  <!-- lidar_link origin is on the scan plane (lidar_scan_z above the roof); the puck's base sits on the roof -->
  <link name="${{prefix}}lidar_link">
    <visual>
      <origin xyz="0 0 ${{lidar_height/2 - lidar_scan_z}}" rpy="0 0 0"/>
      <geometry><cylinder radius="${{lidar_radius}}" length="${{lidar_height}}"/></geometry>
      <material name="sensor_grey"/>
    </visual>
    <collision>
      <origin xyz="0 0 ${{lidar_height/2 - lidar_scan_z}}" rpy="0 0 0"/>
      <geometry><cylinder radius="${{lidar_radius}}" length="${{lidar_height}}"/></geometry>
    </collision>
    <xacro:cylinder_inertial mass="${{lidar_mass}}" r="${{lidar_radius}}" h="${{lidar_height}}"/>
  </link>
  <joint name="${{prefix}}lidar_joint" type="fixed">
    <parent link="${{prefix}}base_link"/>
    <child  link="${{prefix}}lidar_link"/>
    <origin xyz="${{lidar_x}} ${{lidar_y}} ${{lidar_base_z + lidar_scan_z}}" rpy="0 0 0"/>
  </joint>

  <!-- ============================== IMU: floor of the lower cabin ============================== -->
  <link name="${{prefix}}imu_link">
    <visual>
      <origin xyz="0 0 {f(imu_box[2]/2,4)}" rpy="0 0 0"/>
      <geometry><box size="{imu_box[0]} {imu_box[1]} {imu_box[2]}"/></geometry>
      <material name="sensor_grey"/>
    </visual>
    <xacro:box_inertial mass="${{imu_mass}}" x="{imu_box[0]}" y="{imu_box[1]}" z="{imu_box[2]}"/>
  </link>
  <joint name="${{prefix}}imu_joint" type="fixed">
    <parent link="${{prefix}}base_link"/>
    <child  link="${{prefix}}imu_link"/>
    <origin xyz="${{imu_xyz}}" rpy="0 0 0"/>
  </joint>

  <!-- ============================== simulation / control ============================== -->
  <xacro:if value="$(arg use_sim)">
    <xacro:include filename="{ROBOT}.gazebo.xacro"/>
  </xacro:if>
  <xacro:if value="$(arg use_ros2_control)">
    <xacro:include filename="{ROBOT}.ros2_control.xacro"/>
  </xacro:if>

</robot>
"""
open(f"{OUT}/urdf/{ROBOT}.urdf.xacro", "w").write(xacro_main)

# ---------------------------------------------------------------- gazebo include
gazebo_xacro = f'''<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Gazebo (gz-sim / Harmonic+) plugins. Included when use_sim:=true -->

  <gazebo reference="${{prefix}}base_link">
    <mu1>0.6</mu1><mu2>0.6</mu2>
  </gazebo>

  <gazebo>
    <!-- 4-wheel skid-steer via the diff-drive system: all left wheels / all right wheels -->
    <plugin filename="gz-sim-diff-drive-system" name="gz::sim::systems::DiffDrive">
      <left_joint>${{prefix}}front_left_wheel_joint</left_joint>
      <left_joint>${{prefix}}rear_left_wheel_joint</left_joint>
      <right_joint>${{prefix}}front_right_wheel_joint</right_joint>
      <right_joint>${{prefix}}rear_right_wheel_joint</right_joint>
      <wheel_separation>${{2*wheel_y}}</wheel_separation>
      <wheel_radius>${{wheel_radius}}</wheel_radius>
      <max_linear_acceleration>1.0</max_linear_acceleration>
      <min_linear_acceleration>-1.0</min_linear_acceleration>
      <max_angular_acceleration>2.0</max_angular_acceleration>
      <min_angular_acceleration>-2.0</min_angular_acceleration>
      <max_linear_velocity>${{wheel_radius*wheel_max_velocity}}</max_linear_velocity>
      <min_linear_velocity>${{-wheel_radius*wheel_max_velocity}}</min_linear_velocity>
      <topic>cmd_vel</topic>
      <odom_topic>odom</odom_topic>
      <tf_topic>tf</tf_topic>
      <frame_id>odom</frame_id>
      <child_frame_id>${{prefix}}base_footprint</child_frame_id>
      <odom_publish_frequency>50</odom_publish_frequency>
    </plugin>

    <plugin filename="gz-sim-joint-state-publisher-system" name="gz::sim::systems::JointStatePublisher">
      <topic>joint_states</topic>
      <joint_name>${{prefix}}front_left_wheel_joint</joint_name>
      <joint_name>${{prefix}}front_right_wheel_joint</joint_name>
      <joint_name>${{prefix}}rear_left_wheel_joint</joint_name>
      <joint_name>${{prefix}}rear_right_wheel_joint</joint_name>
    </plugin>

    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
  </gazebo>

  <gazebo reference="${{prefix}}imu_link">
    <sensor name="imu" type="imu">
      <always_on>true</always_on>
      <update_rate>100</update_rate>
      <topic>imu</topic>
      <gz_frame_id>${{prefix}}imu_link</gz_frame_id>
    </sensor>
  </gazebo>

  <gazebo reference="${{prefix}}lidar_link">
    <sensor name="lidar" type="gpu_lidar">
      <always_on>true</always_on>
      <update_rate>10</update_rate>
      <visualize>true</visualize>
      <topic>scan</topic>
      <gz_frame_id>${{prefix}}lidar_link</gz_frame_id>
      <lidar>
        <scan>
          <horizontal>
            <samples>720</samples>
            <resolution>1</resolution>
            <min_angle>${{-PI}}</min_angle>
            <max_angle>${{PI}}</max_angle>
          </horizontal>
        </scan>
        <range>
          <min>0.15</min>
          <max>12.0</max>
          <resolution>0.01</resolution>
        </range>
        <noise><type>gaussian</type><mean>0.0</mean><stddev>0.01</stddev></noise>
      </lidar>
    </sensor>
  </gazebo>
</robot>
'''
open(f"{OUT}/urdf/{ROBOT}.gazebo.xacro", "w").write(gazebo_xacro)

ros2_control_xacro = f'''<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- ros2_control: swap the hardware plugin for your motor-driver interface on real hardware -->
  <ros2_control name="${{prefix}}{ROBOT}_system" type="system">
    <hardware>
      <xacro:if value="$(arg use_sim)">
        <plugin>gz_ros2_control/GazeboSimSystem</plugin>
      </xacro:if>
      <xacro:unless value="$(arg use_sim)">
        <plugin>mock_components/GenericSystem</plugin>
        <param name="calculate_dynamics">true</param>
      </xacro:unless>
    </hardware>
    <xacro:macro name="wheel_iface" params="joint">
      <joint name="${{joint}}">
        <command_interface name="velocity">
          <param name="min">${{-wheel_max_velocity}}</param>
          <param name="max">${{wheel_max_velocity}}</param>
        </command_interface>
        <state_interface name="position"/>
        <state_interface name="velocity"/>
      </joint>
    </xacro:macro>
    <xacro:wheel_iface joint="${{prefix}}front_left_wheel_joint"/>
    <xacro:wheel_iface joint="${{prefix}}front_right_wheel_joint"/>
    <xacro:wheel_iface joint="${{prefix}}rear_left_wheel_joint"/>
    <xacro:wheel_iface joint="${{prefix}}rear_right_wheel_joint"/>
  </ros2_control>
</robot>
'''
open(f"{OUT}/urdf/{ROBOT}.ros2_control.xacro", "w").write(ros2_control_xacro)

controllers_yaml = f'''controller_manager:
  ros__parameters:
    update_rate: 50
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster
    diff_drive_controller:
      type: diff_drive_controller/DiffDriveController

diff_drive_controller:
  ros__parameters:
    left_wheel_names:  [front_left_wheel_joint, rear_left_wheel_joint]
    right_wheel_names: [front_right_wheel_joint, rear_right_wheel_joint]
    wheel_separation: {f(2*wheel_y,4)}
    wheel_radius: {f(whl_r,4)}
    wheels_per_side: 2
    base_frame_id: base_footprint
    odom_frame_id: odom
    publish_rate: 50.0
    enable_odom_tf: true
    cmd_vel_timeout: 0.5
    use_stamped_vel: false
    linear.x.max_velocity: {f(whl_r*10.5,3)}
    linear.x.min_velocity: {f(-whl_r*10.5,3)}
    angular.z.max_velocity: {f(whl_r*10.5*2/(2*wheel_y),3)}
    angular.z.min_velocity: {f(-whl_r*10.5*2/(2*wheel_y),3)}
'''
open(f"{OUT}/config/controllers.yaml", "w").write(controllers_yaml)

# ---------------------------------------------------------------- package boilerplate
open(f"{OUT}/package.xml", "w").write(f'''<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>{PKG}</name>
  <version>0.1.0</version>
  <description>URDF/xacro description of the {ROBOT} 4-wheel skid-steer platform, generated from Main_Assembly.step</description>
  <maintainer email="user@example.com">maintainer</maintainer>
  <license>Apache-2.0</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <exec_depend>xacro</exec_depend>
  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>joint_state_publisher_gui</exec_depend>
  <exec_depend>rviz2</exec_depend>
  <export><build_type>ament_cmake</build_type></export>
</package>
''')
open(f"{OUT}/CMakeLists.txt", "w").write(f'''cmake_minimum_required(VERSION 3.8)
project({PKG})
find_package(ament_cmake REQUIRED)
install(DIRECTORY urdf meshes launch rviz config DESTINATION share/${{PROJECT_NAME}})
ament_package()
''')
open(f"{OUT}/launch/display.launch.py", "w").write(f'''from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = FindPackageShare("{PKG}")
    use_sim = LaunchConfiguration("use_sim")
    robot_description = ParameterValue(
        Command([
            FindExecutable(name="xacro"), " ",
            PathJoinSubstitution([pkg, "urdf", "{ROBOT}.urdf.xacro"]),
            " use_sim:=", use_sim,
        ]),
        value_type=str,
    )
    return LaunchDescription([
        DeclareLaunchArgument("use_sim", default_value="false"),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{{"robot_description": robot_description}}]),
        Node(package="joint_state_publisher_gui", executable="joint_state_publisher_gui"),
        Node(package="rviz2", executable="rviz2",
             arguments=["-d", PathJoinSubstitution([pkg, "rviz", "display.rviz"])]),
    ])
''')
open(f"{OUT}/rviz/display.rviz", "w").write('''Panels:
  - Class: rviz_common/Displays
    Name: Displays
Visualization Manager:
  Displays:
    - Class: rviz_default_plugins/Grid
      Name: Grid
      Value: true
    - Class: rviz_default_plugins/RobotModel
      Name: RobotModel
      Description Topic:
        Value: /robot_description
      Value: true
  Global Options:
    Fixed Frame: base_footprint
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 1.0
      Pitch: 0.5
      Yaw: 0.8
''')
print("written", OUT)
