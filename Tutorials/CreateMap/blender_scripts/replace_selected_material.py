import bpy


TARGET_MATERIAL_NAME = "itu_concrete"


def get_or_create_material(name):
    material = bpy.data.materials.get(name)
    if material is None:
        material = bpy.data.materials.new(name=name)
    return material


def replace_selected_materials(target_material):
    count = 0

    for obj in bpy.context.selected_objects:
        if not obj.data or not hasattr(obj.data, "materials"):
            continue

        materials = obj.data.materials
        if len(materials) == 0:
            materials.append(target_material)
        else:
            for index in range(len(materials)):
                materials[index] = target_material
        count += 1

    return count


def main():
    target_material = get_or_create_material(TARGET_MATERIAL_NAME)
    count = replace_selected_materials(target_material)
    print(f"[OK] Processed {count} selected objects. Material set to {TARGET_MATERIAL_NAME}")


if __name__ == "__main__":
    main()
