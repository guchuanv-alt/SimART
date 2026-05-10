# Blender简化地图代码

## 1.地图简化代码

选中需要简化的对象（建议是建筑物），在“脚本”中运行此代码。会将对象替换为一个长宽高相等的立方体，同时隐藏原对象。处理完后校准无误可删除原对象

```python
import bpy
from mathutils import Vector, Matrix

# =========================
# 配置
# =========================

# True: 只处理当前选中的对象（推荐）
# False: 处理当前场景里所有 Mesh 对象
USE_SELECTION_ONLY = True

# 生成的方块楼放到这个集合里
OUTPUT_COLLECTION_NAME = "LOD_Buildings"

# 新对象名前缀
PROXY_PREFIX = "LOD_Box_"

# 原对象如何处理
HIDE_ORIGINAL = True
DELETE_ORIGINAL = False   # 如果设为 True，会直接删除原建筑

# 是否给新方块统一材质
CREATE_GRAY_MATERIAL = True

# 是否忽略太小的对象
FILTER_SMALL_OBJECTS = True
MIN_SIZE_X = 1.0
MIN_SIZE_Y = 1.0
MIN_SIZE_Z = 2.0


# =========================
# 工具函数
# =========================

def log(msg):
    print(f"[LOD-BOX] {msg}")

def ensure_collection(name):
    col = bpy.data.collections.get(name)
    if col is None:
        col = bpy.data.collections.new(name)
        bpy.context.scene.collection.children.link(col)
    return col

def get_or_create_gray_material():
    name = "LOD_Box_Material"
    mat = bpy.data.materials.get(name)
    if mat is None:
        mat = bpy.data.materials.new(name=name)
        mat.use_nodes = True
        bsdf = mat.node_tree.nodes.get("Principled BSDF")
        if bsdf:
            bsdf.inputs["Base Color"].default_value = (0.72, 0.72, 0.72, 1.0)
            bsdf.inputs["Roughness"].default_value = 0.9
    return mat

def assign_material(obj, mat):
    if obj.type != 'MESH':
        return
    if len(obj.data.materials) == 0:
        obj.data.materials.append(mat)
    else:
        obj.data.materials[0] = mat

def move_to_collection(obj, target_collection):
    for c in list(obj.users_collection):
        c.objects.unlink(obj)
    target_collection.objects.link(obj)

def get_world_size(obj):
    depsgraph = bpy.context.evaluated_depsgraph_get()
    obj_eval = obj.evaluated_get(depsgraph)

    corners_local = [Vector(corner) for corner in obj_eval.bound_box]
    corners_world = [obj_eval.matrix_world @ v for v in corners_local]

    min_v = Vector((
        min(v.x for v in corners_world),
        min(v.y for v in corners_world),
        min(v.z for v in corners_world),
    ))
    max_v = Vector((
        max(v.x for v in corners_world),
        max(v.y for v in corners_world),
        max(v.z for v in corners_world),
    ))
    return max_v - min_v

def get_candidates():
    if USE_SELECTION_ONLY:
        objs = [obj for obj in bpy.context.selected_objects if obj.type == 'MESH']
    else:
        objs = [obj for obj in bpy.context.scene.objects if obj.type == 'MESH']

    results = []
    for obj in objs:
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

    # 局部包围盒
    corners = [Vector(corner) for corner in obj_eval.bound_box]
    local_min = Vector((
        min(v.x for v in corners),
        min(v.y for v in corners),
        min(v.z for v in corners),
    ))
    local_max = Vector((
        max(v.x for v in corners),
        max(v.y for v in corners),
        max(v.z for v in corners),
    ))

    local_size = local_max - local_min
    local_center = (local_min + local_max) * 0.5

    if local_size.x <= 1e-6 or local_size.y <= 1e-6 or local_size.z <= 1e-6:
        log(f"Skip degenerate object: {src_obj.name}")
        return None

    # 创建默认立方体
    bpy.ops.mesh.primitive_cube_add(size=2.0, location=(0, 0, 0))
    cube = bpy.context.active_object
    cube.name = f"{PROXY_PREFIX}{src_obj.name}"

    # 默认 cube 尺寸是 2，所以这里缩放用 size/2
    scale_mat = Matrix.Diagonal((
        local_size.x / 2.0,
        local_size.y / 2.0,
        local_size.z / 2.0,
        1.0
    ))

    # 保留原对象的旋转/缩放/平移，并把方块放到原对象局部包围盒中心
    cube.matrix_world = obj_eval.matrix_world @ Matrix.Translation(local_center) @ scale_mat

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
    out_col = ensure_collection(OUTPUT_COLLECTION_NAME)
    mat = get_or_create_gray_material() if CREATE_GRAY_MATERIAL else None

    candidates = get_candidates()
    log(f"Candidate objects: {len(candidates)}")

    created = 0
    for obj in candidates:
        proxy = create_box_proxy(obj, out_col, mat)
        if proxy is not None:
            process_original(obj)
            created += 1

    bpy.context.view_layer.update()
    log(f"Created proxies: {created}")
    log("Done.")

if __name__ == "__main__":
    main()
```

## 2.材质修改代码

选中需要修改材质的对象，在“脚本”中运行此代码，会将选中的对象的所有材质槽都换为“itu_concrete”。如果不换为itu_concrete，则将代码中的target_name改为其他名称。若无此材质，则会创建一个此名字的材质并应用。

```python
import bpy

target_name = "itu_concrete"

target_mat = bpy.data.materials.get(target_name)
if target_mat is None:
    target_mat = bpy.data.materials.new(name=target_name)

count = 0

for obj in bpy.context.selected_objects:
    if obj.data and hasattr(obj.data, "materials"):
        mats = obj.data.materials
        if len(mats) == 0:
            mats.append(target_mat)
        else:
            for i in range(len(mats)):
                mats[i] = target_mat
        count += 1

print(f"[OK] 已处理 {count} 个选中物体，所有材质槽替换为 {target_name}")
```

## 3.场景总面数统计代码

在“脚本”中运行此代码，可将场景中所有对象总面数统计出来，并生成Mesh_Statistics_Report统计报告。

```python
import bpy

REPORT_NAME = "Mesh_Statistics_Report"
TOP_N = 50  # 报告里显示三角面数最高的前 N 个对象

def format_int(n):
    return f"{n:,}"

def get_evaluated_mesh_stats(obj, depsgraph):
    obj_eval = obj.evaluated_get(depsgraph)
    me = obj_eval.to_mesh()
    if me is None:
        return None

    me.calc_loop_triangles()

    stats = {
        "vertices": len(me.vertices),
        "edges": len(me.edges),
        "faces": len(me.polygons),
        "triangles": len(me.loop_triangles),
    }

    obj_eval.to_mesh_clear()
    return stats

def get_or_create_report_text(name):
    txt = bpy.data.texts.get(name)
    if txt is None:
        txt = bpy.data.texts.new(name)
    else:
        txt.clear()
    return txt

def main():
    depsgraph = bpy.context.evaluated_depsgraph_get()
    scene = bpy.context.scene

    mesh_objects = [obj for obj in scene.objects if obj.type == 'MESH']
    selected_mesh_objects = [obj for obj in bpy.context.selected_objects if obj.type == 'MESH']

    total_vertices = 0
    total_edges = 0
    total_faces = 0
    total_triangles = 0

    per_object = []

    for obj in mesh_objects:
        stats = get_evaluated_mesh_stats(obj, depsgraph)
        if stats is None:
            continue

        total_vertices += stats["vertices"]
        total_edges += stats["edges"]
        total_faces += stats["faces"]
        total_triangles += stats["triangles"]

        per_object.append({
            "name": obj.name,
            "vertices": stats["vertices"],
            "edges": stats["edges"],
            "faces": stats["faces"],
            "triangles": stats["triangles"],
        })

    selected_vertices = 0
    selected_edges = 0
    selected_faces = 0
    selected_triangles = 0

    for obj in selected_mesh_objects:
        stats = get_evaluated_mesh_stats(obj, depsgraph)
        if stats is None:
            continue
        selected_vertices += stats["vertices"]
        selected_edges += stats["edges"]
        selected_faces += stats["faces"]
        selected_triangles += stats["triangles"]

    per_object.sort(key=lambda x: x["triangles"], reverse=True)

    lines = []
    lines.append("Mesh Statistics Report")
    lines.append("=" * 80)
    lines.append(f"Scene: {scene.name}")
    lines.append("")
    lines.append("[Scene Total]")
    lines.append(f"Mesh objects : {format_int(len(mesh_objects))}")
    lines.append(f"Vertices     : {format_int(total_vertices)}")
    lines.append(f"Edges        : {format_int(total_edges)}")
    lines.append(f"Faces        : {format_int(total_faces)}")
    lines.append(f"Triangles    : {format_int(total_triangles)}")
    lines.append("")

    lines.append("[Selected Objects Total]")
    lines.append(f"Selected mesh objects : {format_int(len(selected_mesh_objects))}")
    lines.append(f"Vertices              : {format_int(selected_vertices)}")
    lines.append(f"Edges                 : {format_int(selected_edges)}")
    lines.append(f"Faces                 : {format_int(selected_faces)}")
    lines.append(f"Triangles             : {format_int(selected_triangles)}")
    lines.append("")

    lines.append(f"[Top {min(TOP_N, len(per_object))} Objects by Triangles]")
    lines.append("-" * 80)
    lines.append(f"{'Rank':>4} | {'Triangles':>12} | {'Faces':>12} | {'Name'}")
    lines.append("-" * 80)

    for i, item in enumerate(per_object[:TOP_N], start=1):
        lines.append(
            f"{i:>4} | {format_int(item['triangles']):>12} | "
            f"{format_int(item['faces']):>12} | {item['name']}"
        )

    lines.append("")
    lines.append("[Note]")
    lines.append("These numbers are evaluated mesh stats (modifiers included if visible).")

    report_text = get_or_create_report_text(REPORT_NAME)
    report_text.write("\n".join(lines))

    # 顺手把总三角面数也写到 Scene 自定义属性里，方便以后脚本读取
    scene["mesh_report_total_triangles"] = total_triangles
    scene["mesh_report_total_faces"] = total_faces

    print(f"Report generated: {REPORT_NAME}")

if __name__ == "__main__":
    main()
```

## 4.清理无效法线代码

此代码会清理掉场景中所有元素的无效法线。但是在某些情况下可能不起作用

```python
import bpy
import bmesh

MERGE_DIST = 1e-4   # 不够就改成 1e-4，再不够 1e-3（从小往大）
DEGEN_DIST = 1e-12  # 退化阈值（很小）

def fix_mesh_data(me, merge_dist=MERGE_DIST):
    bm = bmesh.new()
    bm.from_mesh(me)

    # 1) 合并重合点（等价 Merge by Distance）
    bmesh.ops.remove_doubles(bm, verts=bm.verts, dist=merge_dist)

    # 2) 溶解退化几何（0长度边/0面积面）
    bmesh.ops.dissolve_degenerate(bm, edges=bm.edges, dist=DEGEN_DIST)

    # 3) 删除松散元素（孤立点/边）
    loose_verts = [v for v in bm.verts if not v.link_edges and not v.link_faces]
    if loose_verts:
        bmesh.ops.delete(bm, geom=loose_verts, context='VERTS')

    loose_edges = [e for e in bm.edges if not e.link_faces]
    if loose_edges:
        bmesh.ops.delete(bm, geom=loose_edges, context='EDGES')

    # 4) 重算法线（面法线一致性）
    if bm.faces:
        bmesh.ops.recalc_face_normals(bm, faces=bm.faces)

    bm.normal_update()
    bm.to_mesh(me)
    bm.free()

    # 5) 校验并清理自定义数据（尽量清掉坏的 customdata / normals）
    me.validate(clean_customdata=True)
    me.update()

def main():
    fixed = 0
    skipped_linked = 0

    # 只遍历当前场景对象（避免别的场景数据干扰）
    for obj in bpy.context.scene.objects:
        if obj.type != 'MESH':
            continue

        # 外链库对象无法直接改数据：需要先 Make Local
        if obj.library is not None or (obj.data and obj.data.library is not None):
            skipped_linked += 1
            continue

        fix_mesh_data(obj.data)
        fixed += 1

    print(f"[OK] fixed mesh objects: {fixed}, skipped linked meshes: {skipped_linked}")
    if skipped_linked:
        print("有外链对象被跳过：请先 Object > Relations > Make Local > All，然后再运行一次脚本。")

main()
```

## 5.修改材质颜色代码（fbx版）

在blender脚本里面运行此脚本，可以将下面的itu材质的预览颜色更换为对应的RGB值。这个脚本同样也会在更换完颜色后自动导出fbx场景文件。因为如果手动在blender里面导出fbx文件，所有的颜色又会被blender给强制覆盖为灰色。故在此脚本中直接导出，以绕过blender的强制覆盖机制。

```python
import bpy

colors = {
    "itu_metal": (0.35, 0.35, 0.35, 1.0),
    "itu_wood": (0.55, 0.28, 0.10, 1.0),
    "itu_plasterboard": (0.78, 0.78, 0.72, 1.0),
    "itu_glass": (0.20, 0.65, 0.85, 1.0),
    "itu_very_dry_ground": (0.42, 0.38, 0.30, 1.0),
    "itu_concrete": (0.55, 0.55, 0.52, 1.0),
    "itu_medium_dry_ground": (0.30, 0.36, 0.30, 1.0),
    "itu_brick": (0.55, 0.22, 0.13, 1.0),
    "itu_marble": (0.85, 0.85, 0.82, 1.0),
}

for mat in bpy.data.materials:
    base_name = mat.name.split(".")[0]
    if base_name not in colors:
        continue

    color = colors[base_name]

    # Blender 材质自身颜色
    mat.diffuse_color = color

    # 关闭复杂节点，强制变成普通材质颜色，便于 FBX 导出器写入
    mat.use_nodes = False

print("Material colors before export:")
for mat in bpy.data.materials:
    print(mat.name, tuple(round(v, 3) for v in mat.diffuse_color))

bpy.ops.export_scene.fbx(
    filepath="/home/guchuan/GuChuan/GUI_worlds/BigCitySample/BigCitySample_color_exported.fbx",
    use_selection=False,
    object_types={'MESH'},
    global_scale=1,
    apply_unit_scale=False,
    apply_scale_options='FBX_SCALE_UNITS',
    axis_forward='Y',
    axis_up='Z',
    use_space_transform=True,
    bake_space_transform=False,
    path_mode='AUTO'
)

print("Export finished.")
```

## 6.检查材质颜色是否正确修改代码

```python
import bpy

for mat in bpy.data.materials:
    print(mat.name, tuple(round(v, 3) for v in mat.diffuse_color))
```

这个脚本会输出所有材质的RGB值以供检查。