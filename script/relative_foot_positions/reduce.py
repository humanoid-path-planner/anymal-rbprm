import bpy
import glob
import os

# this script is tested with blender 2.82
# WARNING !! this script will erase your scene
#

# change those parameters according to your needs
TARGET_NUM_FACES = 24.0
FOLDER_PATH = (
    "/media/data/dev/linux/hpp/src/anymal-rbprm/script/relative_foot_positions/"
)
OUTPUT_PATH = FOLDER_PATH + "output/"

EXTENSION = ".obj"


def decimate(obj):
    nFaces = len(obj.data.polygons)
    heuristic_ratio = TARGET_NUM_FACES / float(nFaces)
    bpy.ops.mesh.decimate(ratio=heuristic_ratio)
    return


def load_obj(file):
    # bpy.ops.import_scene.obj(filepath=FOLDER_PATH+file, axis_forward='X', axis_up='Z')
    bpy.ops.import_scene.obj(filepath=FOLDER_PATH + file)

    obj = bpy.data.objects[-1]

    # api change in 2.82
    # bpy.context.scene.objects.active = obj
    bpy.context.view_layer.objects.active = obj
    bpy.ops.object.editmode_toggle()
    bpy.ops.mesh.delete(type="EDGE_FACE")
    bpy.ops.mesh.select_mode(type="VERT")
    bpy.ops.mesh.select_all(action="SELECT")
    bpy.ops.mesh.convex_hull()
    decimate(obj)

    # to export first extract filename
    idx = file.index(EXTENSION)
    obj.name = file[:idx] + "_reduced"
    bpy.ops.export_scene.obj(
        filepath=OUTPUT_PATH + obj.name + EXTENSION,
        check_existing=True,
        filter_glob="*.obj;*.mtl",
        use_selection=True,
        use_animation=False,
        use_mesh_modifiers=True,
        use_edges=True,
        use_smooth_groups=False,
        use_smooth_groups_bitflags=False,
        use_normals=True,
        use_uvs=True,
        use_materials=False,
        use_triangles=False,
        use_nurbs=False,
        use_vertex_groups=False,
        use_blen_objects=True,
        group_by_object=False,
        group_by_material=False,
        keep_vertex_order=False,
        # ~ global_scale=1.0, path_mode='AUTO', axis_forward='X', axis_up='Z')
        global_scale=1.0,
        path_mode="AUTO",
    )

    # delete all objects
    bpy.ops.object.delete()


bpy.ops.object.select_all(action="SELECT")
bpy.ops.object.delete()

os.chdir(FOLDER_PATH)
directory = os.path.dirname(OUTPUT_PATH)
if not os.path.exists(directory):
    os.makedirs(directory)

# clear the scene !

for file in glob.glob("*.obj"):
    load_obj(file)
