import bmesh
import bpy


MERGE_DIST = 1e-4
DEGEN_DIST = 1e-12


def fix_mesh_data(mesh, merge_dist=MERGE_DIST):
    bm = bmesh.new()
    bm.from_mesh(mesh)

    bmesh.ops.remove_doubles(bm, verts=bm.verts, dist=merge_dist)
    bmesh.ops.dissolve_degenerate(bm, edges=bm.edges, dist=DEGEN_DIST)

    loose_verts = [vertex for vertex in bm.verts if not vertex.link_edges and not vertex.link_faces]
    if loose_verts:
        bmesh.ops.delete(bm, geom=loose_verts, context="VERTS")

    loose_edges = [edge for edge in bm.edges if not edge.link_faces]
    if loose_edges:
        bmesh.ops.delete(bm, geom=loose_edges, context="EDGES")

    if bm.faces:
        bmesh.ops.recalc_face_normals(bm, faces=bm.faces)

    bm.normal_update()
    bm.to_mesh(mesh)
    bm.free()

    mesh.validate(clean_customdata=True)
    mesh.update()


def main():
    fixed = 0
    skipped_linked = 0

    for obj in bpy.context.scene.objects:
        if obj.type != "MESH":
            continue

        if obj.library is not None or (obj.data and obj.data.library is not None):
            skipped_linked += 1
            continue

        fix_mesh_data(obj.data)
        fixed += 1

    print(f"[OK] fixed mesh objects: {fixed}, skipped linked meshes: {skipped_linked}")
    if skipped_linked:
        print("Some linked objects were skipped. Use Object > Relations > Make Local > All, then run the script again.")


if __name__ == "__main__":
    main()
