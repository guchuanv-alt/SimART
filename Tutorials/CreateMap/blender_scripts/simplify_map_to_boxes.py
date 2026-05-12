import bpy
from mathutils import Matrix, Vector


# True: process only currently selected objects.
# False: process all Mesh objects in the current scene.
USE_SELECTION_ONLY = True

# Generated box proxies are stored in this collection.
OUTPUT_COLLECTION_NAME = "LOD_Buildings"

# Prefix for generated proxy object names.
PROXY_PREFIX = "LOD_Box_"

# How to handle original objects.
HIDE_ORIGINAL = True
DELETE_ORIGINAL = False

# Whether to assign one shared gray material to generated boxes.
CREATE_GRAY_MATERIAL = True

# Whether to ignore very small objects.
FILTER_SMALL_OBJECTS = True
MIN_SIZE_X = 1.0
MIN_SIZE_Y = 1.0
MIN_SIZE_Z = 2.0


def log(message):
    print(f"[LOD-BOX] {message}")


def ensure_collection(name):
    collection = bpy.data.collections.get(name)
    if collection is None:
        collection = bpy.data.collections.new(name)
        bpy.context.scene.collection.children.link(collection)
    return collection


def get_or_create_gray_material():
    name = "LOD_Box_Material"
    material = bpy.data.materials.get(name)
    if material is None:
        material = bpy.data.materials.new(name=name)
        material.use_nodes = True
        bsdf = material.node_tree.nodes.get("Principled BSDF")
        if bsdf:
            bsdf.inputs["Base Color"].default_value = (0.72, 0.72, 0.72, 1.0)
            bsdf.inputs["Roughness"].default_value = 0.9
    return material


def assign_material(obj, material):
    if obj.type != "MESH":
        return

    if len(obj.data.materials) == 0:
        obj.data.materials.append(material)
    else:
        obj.data.materials[0] = material


def move_to_collection(obj, target_collection):
    for collection in list(obj.users_collection):
        collection.objects.unlink(obj)
    target_collection.objects.link(obj)


def get_world_size(obj):
    depsgraph = bpy.context.evaluated_depsgraph_get()
    obj_eval = obj.evaluated_get(depsgraph)

    corners_local = [Vector(corner) for corner in obj_eval.bound_box]
    corners_world = [obj_eval.matrix_world @ vertex for vertex in corners_local]

    min_v = Vector((
        min(vertex.x for vertex in corners_world),
        min(vertex.y for vertex in corners_world),
        min(vertex.z for vertex in corners_world),
    ))
    max_v = Vector((
        max(vertex.x for vertex in corners_world),
        max(vertex.y for vertex in corners_world),
        max(vertex.z for vertex in corners_world),
    ))
    return max_v - min_v


def get_candidates():
    if USE_SELECTION_ONLY:
        objects = [obj for obj in bpy.context.selected_objects if obj.type == "MESH"]
    else:
        objects = [obj for obj in bpy.context.scene.objects if obj.type == "MESH"]

    results = []
    for obj in objects:
        if obj.name.startswith(PROXY_PREFIX):
            continue

        if FILTER_SMALL_OBJECTS:
            size = get_world_size(obj)
            if size.x < MIN_SIZE_X or size.y < MIN_SIZE_Y or size.z < MIN_SIZE_Z:
                continue

        results.append(obj)

    return results


def create_box_proxy(src_obj, output_collection, material=None):
    depsgraph = bpy.context.evaluated_depsgraph_get()
    obj_eval = src_obj.evaluated_get(depsgraph)

    corners = [Vector(corner) for corner in obj_eval.bound_box]
    local_min = Vector((
        min(vertex.x for vertex in corners),
        min(vertex.y for vertex in corners),
        min(vertex.z for vertex in corners),
    ))
    local_max = Vector((
        max(vertex.x for vertex in corners),
        max(vertex.y for vertex in corners),
        max(vertex.z for vertex in corners),
    ))

    local_size = local_max - local_min
    local_center = (local_min + local_max) * 0.5

    if local_size.x <= 1e-6 or local_size.y <= 1e-6 or local_size.z <= 1e-6:
        log(f"Skip degenerate object: {src_obj.name}")
        return None

    bpy.ops.mesh.primitive_cube_add(size=2.0, location=(0, 0, 0))
    cube = bpy.context.active_object
    cube.name = f"{PROXY_PREFIX}{src_obj.name}"

    scale_matrix = Matrix.Diagonal((
        local_size.x / 2.0,
        local_size.y / 2.0,
        local_size.z / 2.0,
        1.0,
    ))

    cube.matrix_world = obj_eval.matrix_world @ Matrix.Translation(local_center) @ scale_matrix
    move_to_collection(cube, output_collection)

    if material is not None:
        assign_material(cube, material)

    return cube


def process_original(obj):
    if DELETE_ORIGINAL:
        bpy.data.objects.remove(obj, do_unlink=True)
    elif HIDE_ORIGINAL:
        obj.hide_set(True)
        obj.hide_render = True


def main():
    output_collection = ensure_collection(OUTPUT_COLLECTION_NAME)
    material = get_or_create_gray_material() if CREATE_GRAY_MATERIAL else None

    candidates = get_candidates()
    log(f"Candidate objects: {len(candidates)}")

    created = 0
    for obj in candidates:
        proxy = create_box_proxy(obj, output_collection, material)
        if proxy is not None:
            process_original(obj)
            created += 1

    bpy.context.view_layer.update()
    log(f"Created proxies: {created}")
    log("Done.")


if __name__ == "__main__":
    main()
