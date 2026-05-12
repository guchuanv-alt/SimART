import bpy


REPORT_NAME = "Mesh_Statistics_Report"
TOP_N = 50


def format_int(value):
    return f"{value:,}"


def get_evaluated_mesh_stats(obj, depsgraph):
    obj_eval = obj.evaluated_get(depsgraph)
    mesh = obj_eval.to_mesh()
    if mesh is None:
        return None

    mesh.calc_loop_triangles()

    stats = {
        "vertices": len(mesh.vertices),
        "edges": len(mesh.edges),
        "faces": len(mesh.polygons),
        "triangles": len(mesh.loop_triangles),
    }

    obj_eval.to_mesh_clear()
    return stats


def get_or_create_report_text(name):
    text = bpy.data.texts.get(name)
    if text is None:
        text = bpy.data.texts.new(name)
    else:
        text.clear()
    return text


def main():
    depsgraph = bpy.context.evaluated_depsgraph_get()
    scene = bpy.context.scene

    mesh_objects = [obj for obj in scene.objects if obj.type == "MESH"]
    selected_mesh_objects = [obj for obj in bpy.context.selected_objects if obj.type == "MESH"]

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

    per_object.sort(key=lambda item: item["triangles"], reverse=True)

    lines = [
        "Mesh Statistics Report",
        "=" * 80,
        f"Scene: {scene.name}",
        "",
        "[Scene Total]",
        f"Mesh objects : {format_int(len(mesh_objects))}",
        f"Vertices     : {format_int(total_vertices)}",
        f"Edges        : {format_int(total_edges)}",
        f"Faces        : {format_int(total_faces)}",
        f"Triangles    : {format_int(total_triangles)}",
        "",
        "[Selected Objects Total]",
        f"Selected mesh objects : {format_int(len(selected_mesh_objects))}",
        f"Vertices              : {format_int(selected_vertices)}",
        f"Edges                 : {format_int(selected_edges)}",
        f"Faces                 : {format_int(selected_faces)}",
        f"Triangles             : {format_int(selected_triangles)}",
        "",
        f"[Top {min(TOP_N, len(per_object))} Objects by Triangles]",
        "-" * 80,
        f"{'Rank':>4} | {'Triangles':>12} | {'Faces':>12} | {'Name'}",
        "-" * 80,
    ]

    for index, item in enumerate(per_object[:TOP_N], start=1):
        lines.append(
            f"{index:>4} | {format_int(item['triangles']):>12} | "
            f"{format_int(item['faces']):>12} | {item['name']}"
        )

    lines.extend([
        "",
        "[Note]",
        "These numbers are evaluated mesh stats, including visible modifiers.",
    ])

    report_text = get_or_create_report_text(REPORT_NAME)
    report_text.write("\n".join(lines))

    scene["mesh_report_total_triangles"] = total_triangles
    scene["mesh_report_total_faces"] = total_faces

    print(f"Report generated: {REPORT_NAME}")


if __name__ == "__main__":
    main()
