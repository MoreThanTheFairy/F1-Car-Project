import bpy
import bmesh
import math
from mathutils import Vector, Matrix, Euler
import os

# ------------------------- USER SETTINGS -------------------------

INPUT_STL = r"/mnt/data/Chasis V1 (2).stl"   # <-- change if needed
OUTPUT_DIR = r"/mnt/data/chassis_outputs"    # output folder will be created if missing

# Units / scale
ASSUME_INPUT_MM = True         # STL is already in millimeters
UNIT_SCALE_FACTOR = 1.0        # If your STL is meters, set to 1000.0 to convert to mm

# Roll cage adjustment
TARGET_ROLLCAGE_HEIGHT_MM = 1400.0
ROLL_CAGE_Z_THRESHOLD_MM = 800.0  # vertices with Z >= this are considered roll-cage to be lifted
BASE_PLANE_OFFSET_MM = 0.0        # if the base should be at 0, leave as 0. Else set a known base Z.

# Wall thickness (heuristic via Solidify)
MIN_WALL_THICKNESS_MM = 2.0
APPLY_SOLIDIFY = True             # set False to skip
APPLY_TO_OBJECT_NAME_FILTER = ""  # if you want to restrict to certain objects containing this string; empty = all imported
# NOTE: This uses Blender's Solidify (Even Thickness + High Quality Normals).
# It can't locally upsize OD only at thin joints, but will create >=2mm shell where none exists.

# Rendering
IMG_RES = (2200, 1800)
RENDER_TRANSPARENT_BG = False  # keeps background opaque
LINE_OVERLAY = True            # enable cavity/outline for clarity

# Export modified STL
EXPORT_MODIFIED_STL = True
OUTPUT_STL = os.path.join(OUTPUT_DIR, "Chasis_V1_modified_rollcage_1400mm.stl")

# ------------------------- UTILITIES -------------------------

def clean_scene():
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete(use_global=False)
    for block in bpy.data.meshes:
        if block.users == 0:
            bpy.data.meshes.remove(block)

def ensure_output_dir(path):
    if not os.path.isdir(path):
        os.makedirs(path, exist_ok=True)

def import_stl(stl_path, scale_factor=1.0):
    bpy.ops.import_mesh.stl(filepath=stl_path)
    imported = [obj for obj in bpy.context.selected_objects if obj.type == 'MESH']
    for obj in imported:
        obj.scale = (scale_factor, scale_factor, scale_factor)
        obj.name = f"Imported_{obj.name}"
        obj.data.name = f"{obj.name}_Mesh"
    bpy.context.view_layer.update()
    return imported

def get_combined_bounds(objs):
    mins = Vector((float('inf'), float('inf'), float('inf')))
    maxs = Vector((float('-inf'), float('-inf'), float('-inf')))
    for o in objs:
        for v in o.bound_box:
            # v is in local space; convert to world
            wv = o.matrix_world @ Vector(v)
            mins.x = min(mins.x, wv.x)
            mins.y = min(mins.y, wv.y)
            mins.z = min(mins.z, wv.z)
            maxs.x = max(maxs.x, wv.x)
            maxs.y = max(maxs.y, wv.y)
            maxs.z = max(maxs.z, wv.z)
    return mins, maxs

def move_vertices_above_threshold(objs, z_thresh_world, dz):
    # Lift only vertices whose world Z >= threshold (preserve joint geometry below)
    for obj in objs:
        if obj.type != 'MESH':
            continue
        me = obj.data
        bm = bmesh.new()
        bm.from_mesh(me)
        bm.verts.ensure_lookup_table()
        inv = obj.matrix_world.inverted()
        for v in bm.verts:
            w = obj.matrix_world @ v.co
            if w.z >= z_thresh_world:
                w.z += dz
                v.co = inv @ w
        bm.to_mesh(me)
        bm.free()
    bpy.context.view_layer.update()

def add_axes_with_ticks(length=2000.0, tick_every=200.0, text_every=400.0, line_width=0.0025):
    """
    Adds X, Y, Z axes as thin cylinders with tick marks and numeric labels in mm.
    """
    collection = bpy.data.collections.new("Axes")
    bpy.context.scene.collection.children.link(collection)

    def add_cylinder_line(start, end, radius=1.5, name="axis"):
        direction = end - start
        length = direction.length
        if length == 0: return None
        mid = (start + end) * 0.5
        # Create cylinder
        bpy.ops.mesh.primitive_cylinder_add(vertices=24, radius=radius, depth=length, location=mid)
        cyl = bpy.context.active_object
        cyl.name = name
        # Rotate cylinder to align with direction
        up = Vector((0,0,1))
        rot_axis = up.cross(direction)
        if rot_axis.length > 0:
            rot_axis.normalize()
            angle = up.angle(direction)
            cyl.rotation_mode = 'AXIS_ANGLE'
            cyl.rotation_axis_angle = (angle, rot_axis.x, rot_axis.y, rot_axis.z)
        collection.objects.link(cyl)
        bpy.context.scene.collection.objects.unlink(cyl)
        return cyl

    def add_text(label, loc, rot=(0,0,0), size=20.0):
        bpy.ops.object.text_add(location=loc, rotation=rot)
        t = bpy.context.active_object
        t.data.body = label
        t.data.size = size
        t.data.extrude = 0.5
        t.name = f"label_{label}"
        collection.objects.link(t)
        bpy.context.scene.collection.objects.unlink(t)
        return t

    # Axes along world origin
    half = length/2
    # X
    add_cylinder_line(Vector((-half,0,0)), Vector((half,0,0)), radius=1.5, name="Axis_X")
    # Y
    add_cylinder_line(Vector((0,-half,0)), Vector((0,half,0)), radius=1.5, name="Axis_Y")
    # Z
    add_cylinder_line(Vector((0,0,0)), Vector((0,0,length)), radius=1.5, name="Axis_Z")

    # Ticks and labels
    tick_len = 25.0
    for axis, dir_vec, right_vec, name in [
        ('X', Vector((1,0,0)), Vector((0,1,0)), 'X'),
        ('Y', Vector((0,1,0)), Vector((1,0,0)), 'Y'),
        ('Z', Vector((0,0,1)), Vector((1,0,0)), 'Z'),
    ]:
        steps = int(length / tick_every) + 1
        for i in range(steps):
            d = -half + i * tick_every
            if axis == 'X':
                p = Vector((d, 0, 0))
                n = right_vec
            elif axis == 'Y':
                p = Vector((0, d, 0))
                n = right_vec
            else:
                p = Vector((0, 0, d))
                n = right_vec
            # small tick as a cylinder
            add_cylinder_line(p - n * tick_len/2, p + n * tick_len/2, radius=1.2, name=f"Tick_{axis}_{i}")
            # text every 'text_every'
            if (abs(d) % text_every) < 1e-6 or i == 0 or i == steps-1:
                lbl = f"{int(d + (0 if axis!='Z' else 0))} mm"
                # offset text slightly
                text_loc = p + n * (tick_len + 12.0)
                rot = (math.radians(90), 0, 0) if axis in ('X','Y') else (0,0,0)
                add_text(lbl, text_loc, rot=rot, size=20.0)

def setup_world_and_view():
    scene = bpy.context.scene
    scene.render.engine = 'CYCLES'  # good quality
    scene.cycles.samples = 64
    scene.render.resolution_x = IMG_RES[0]
    scene.render.resolution_y = IMG_RES[1]
    scene.render.film_transparent = RENDER_TRANSPARENT_BG

    # Light
    bpy.ops.object.light_add(type='AREA', location=(1200, -1200, 1200))
    light = bpy.context.active_object
    light.data.energy = 3500
    light.data.size = 1000

    bpy.ops.object.light_add(type='SUN', location=(-800, 800, 1600))
    sun = bpy.context.active_object
    sun.data.energy = 2.0

    # Camera placeholder
    if "RenderCam" not in bpy.data.objects:
        bpy.ops.object.camera_add(location=(0, -3000, 900))
        cam = bpy.context.active_object
        cam.name = "RenderCam"
        scene.camera = cam

    # Viewport overlay (line art) for clarity via Freestyle not used; use cavity/curvature in Eevee?
    # We'll keep Cycles and rely on lighting + smooth shading.

def make_materials():
    # Solid material
    mat_solid = bpy.data.materials.new("Mat_Solid")
    mat_solid.use_nodes = True
    bsdf = mat_solid.node_tree.nodes.get("Principled BSDF")
    bsdf.inputs["Roughness"].default_value = 0.4
    bsdf.inputs["Metallic"].default_value = 0.0
    # Semi-transparent
    mat_trans = bpy.data.materials.new("Mat_Transparent")
    mat_trans.use_nodes = True
    nt = mat_trans.node_tree
    bsdf2 = nt.nodes.get("Principled BSDF")
    bsdf2.inputs["Transmission"].default_value = 0.6
    bsdf2.inputs["Alpha"].default_value = 0.35
    bsdf2.inputs["Roughness"].default_value = 0.2
    # Enable alpha blending
    mat_trans.blend_method = 'BLEND'
    mat_trans.shadow_method = 'HASHED'
    return mat_solid, mat_trans

def apply_material(obj, mat):
    if obj.type != 'MESH': return
    if obj.data.materials:
        obj.data.materials[0] = mat
    else:
        obj.data.materials.append(mat)

def apply_solidify(obj, thickness_mm=2.0):
    # Add solidify with even thickness
    m = obj.modifiers.new(name="MinWall_Solidify", type='SOLIDIFY')
    m.thickness = thickness_mm
    m.offset = 1.0            # push outward
    m.use_even_offset = True
    m.use_quality_normals = True
    m.rim = True
    m.nonmanifold_boundary_mode = 'NONE'  # leave alone
    # apply the modifier to bake geometry
    bpy.context.view_layer.objects.active = obj
    bpy.ops.object.modifier_apply(modifier=m.name)

def set_smooth_shading(objs):
    for obj in objs:
        if obj.type == 'MESH':
            bpy.context.view_layer.objects.active = obj
            bpy.ops.object.shade_smooth()
            if obj.data and hasattr(obj.data, "use_auto_smooth"):
                obj.data.use_auto_smooth = True
                obj.data.auto_smooth_angle = math.radians(40)

def position_camera_for_view(view_name, bounds_min, bounds_max, padding=400.0):
    """
    Set camera to a canonical position: front/back/left/right/top/bottom/iso.
    """
    cx = (bounds_min.x + bounds_max.x) * 0.5
    cy = (bounds_min.y + bounds_max.y) * 0.5
    cz = (bounds_min.z + bounds_max.z) * 0.5
    size = (bounds_max - bounds_min).length
    dist = size * 1.2 + padding

    cam = bpy.data.objects["RenderCam"]
    if view_name == "front":
        cam.location = (cx, bounds_min.y - dist, cz)
        cam.rotation_euler = Euler((math.radians(90), 0, 0), 'XYZ')
    elif view_name == "back":
        cam.location = (cx, bounds_max.y + dist, cz)
        cam.rotation_euler = Euler((math.radians(-90), 0, math.radians(180)), 'XYZ')
    elif view_name == "left":
        cam.location = (bounds_min.x - dist, cy, cz)
        cam.rotation_euler = Euler((0, math.radians(90), math.radians(90)), 'XYZ')
    elif view_name == "right":
        cam.location = (bounds_max.x + dist, cy, cz)
        cam.rotation_euler = Euler((0, math.radians(-90), math.radians(-90)), 'XYZ')
    elif view_name == "top":
        cam.location = (cx, cy, bounds_max.z + dist)
        cam.rotation_euler = Euler((0, 0, 0), 'XYZ')
        cam.rotation_euler[0] = 0  # look down
        cam.rotation_euler[0] = 0
        cam.rotation_euler = Euler((0, 0, 0), 'XYZ')
        cam.rotation_euler[0] = 0
        cam.rotation_euler = Euler((0, 0, 0), 'XYZ')
        cam.rotation_euler[0] = 0
        cam.rotation_euler = Euler((0, 0, 0), 'XYZ')
        cam.rotation_euler.x = 0
        cam.rotation_euler.y = 0
        cam.rotation_euler.z = 0
        # point down with Track To
        cam.rotation_euler = Euler((math.radians(0), math.radians(0), math.radians(0)), 'XYZ')
        cam.location = (cx, cy, bounds_max.z + dist)
        cam.rotation_euler = Euler((math.radians(0), math.radians(0), math.radians(0)), 'XYZ')
        # Instead force -Z view by pointing at center:
        cam.rotation_euler = Euler((math.radians(90), 0, 0), 'XYZ')
    elif view_name == "underside":
        cam.location = (cx, cy, bounds_min.z - dist)
        cam.rotation_euler = Euler((math.radians(-90), 0, math.radians(180)), 'XYZ')
    elif view_name == "iso":
        cam.location = (bounds_max.x + dist*0.8, bounds_min.y - dist*0.8, bounds_max.z + dist*0.6)
        cam.rotation_euler = Euler((math.radians(60), 0, math.radians(45)), 'XYZ')

    # Set lens
    cam.data.lens = 85
    bpy.context.scene.camera = cam

def render_set(objs, mat, tag, bounds_min, bounds_max):
    # apply material
    for o in objs:
        apply_material(o, mat)
    bpy.context.view_layer.update()

    views = ["front","back","left","right","top","underside","iso"]
    for v in views:
        position_camera_for_view(v, bounds_min, bounds_max)
        out = os.path.join(OUTPUT_DIR, f"{tag}_{v}.png")
        bpy.context.scene.render.filepath = out
        bpy.ops.render.render(write_still=True)

# ------------------------- MAIN -------------------------

def main():
    clean_scene()
    ensure_output_dir(OUTPUT_DIR)

    # Import
    scale_factor = (1.0 if ASSUME_INPUT_MM else UNIT_SCALE_FACTOR)
    imported_objs = import_stl(INPUT_STL, scale_factor=scale_factor)

    # Set smooth shading for nicer renders
    set_smooth_shading(imported_objs)

    # Compute bounds and base
    bmin, bmax = get_combined_bounds(imported_objs)
    base_z = bmin.z
    if BASE_PLANE_OFFSET_MM != 0.0:
        # shift whole model so base is at desired offset
        dz = BASE_PLANE_OFFSET_MM - base_z
        for o in imported_objs:
            o.location.z += dz
        bpy.context.view_layer.update()
        bmin, bmax = get_combined_bounds(imported_objs)
        base_z = bmin.z

    # Current total height
    current_height = bmax.z - base_z

    # Compute required lift for roll-cage region
    delta_h = TARGET_ROLLCAGE_HEIGHT_MM - current_height
    if abs(delta_h) > 1e-6:
        # Lift vertices whose Z >= threshold
        z_thresh_world = base_z + ROLL_CAGE_Z_THRESHOLD_MM
        move_vertices_above_threshold(imported_objs, z_thresh_world, delta_h)

    bpy.context.view_layer.update()
    bmin, bmax = get_combined_bounds(imported_objs)

    # Heuristic minimum wall thickness via Solidify
    if APPLY_SOLIDIFY:
        for o in imported_objs:
            if o.type != 'MESH': 
                continue
            if APPLY_TO_OBJECT_NAME_FILTER and APPLY_TO_OBJECT_NAME_FILTER not in o.name:
                continue
            apply_solidify(o, thickness_mm=MIN_WALL_THICKNESS_MM)

    # Add axis lines + labels (length chosen to cover model bbox with margin)
    diag = (bmax - bmin).length
    add_axes_with_ticks(length=max(2000.0, diag*1.5),
                        tick_every=200.0,
                        text_every=400.0)

    # World and camera
    setup_world_and_view()
    mat_solid, mat_trans = make_materials()

    # Recompute bounds after ops
    bmin, bmax = get_combined_bounds(imported_objs)

    # Render SOLID
    render_set(imported_objs, mat_solid, tag="solid", bounds_min=bmin, bounds_max=bmax)

    # Render TRANSPARENT
    render_set(imported_objs, mat_trans, tag="transparent", bounds_min=bmin, bounds_max=bmax)

    # Optional STL export
    if EXPORT_MODIFIED_STL:
        # Join all imported objects into one and export
        for o in imported_objs:
            o.select_set(True)
        bpy.context.view_layer.objects.active = imported_objs[0]
        bpy.ops.object.join()
        joined = bpy.context.view_layer.objects.active
        bpy.ops.export_mesh.stl(filepath=OUTPUT_STL, use_selection=True, ascii=False)
        # Unselect after export
        joined.select_set(False)

    print("Done.")

if __name__ == "__main__":
    main()
