#
# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
import drjit as dr
import mitsuba as mi
import polyscope as ps
import polyscope.imgui as psim

from .dlpack_utils import pointer_from_dlpack


def get_array_ptr(arr: dr.ArrayBase) -> tuple[int, int, int, int]:
    value = arr.array if dr.is_tensor_v(arr) else arr
    if hasattr(value, "data_"):
        ptr = value.data_()
    else:
        ptr = pointer_from_dlpack(value)

    return (
        # arr_ptr
        ptr,
        # arr_shape
        dr.shape(arr),
        # arr_dtype
        None,
        # arr_nbytes
        get_array_size_bytes(arr),
    )


def get_array_size_bytes(arr: dr.ArrayBase) -> int:
    assert dr.is_tensor_v(arr) or dr.depth_v(arr) == 1
    match dr.type_v(arr):
        case dr.VarType.UInt8:
            ts = 1
        case dr.VarType.Float16:
            ts = 2
        case dr.VarType.Float32:
            ts = 4
        case dr.VarType.Float64:
            ts = 8
        case _:
            raise ValueError(f"Unsupported array type: {dr.type_v(arr)}")
    return ts * dr.width(arr)


def memcpy_2d_to_array_async(dst_ptr, src_ptr, width, height):
    return dr.cuda.memcpy_2d_to_array_async(
        dst_ptr, src_ptr, src_pitch=width, height=height, from_host=False
    )


def set_polyscope_device_interop_funcs():
    ps_device_func_dict = {
        "map_resource_and_get_array": lambda handle: dr.cuda.map_graphics_resource_array(
            handle
        ),
        "map_resource_and_get_pointer": lambda handle: dr.cuda.map_graphics_resource_ptr(
            handle
        ),
        "unmap_resource": lambda handle: dr.cuda.unmap_graphics_resource(handle),
        "register_gl_buffer": lambda native_id: dr.cuda.register_gl_buffer(native_id),
        "register_gl_image_2d": lambda native_id: dr.cuda.register_gl_texture(
            native_id
        ),
        "unregister_resource": lambda handle: dr.cuda.unregister_cuda_resource(handle),
        "get_array_ptr": get_array_ptr,
        "memcpy_2d": memcpy_2d_to_array_async,
    }

    ps.set_device_interop_funcs(ps_device_func_dict)


def supports_direct_update_from_device() -> bool:
    return "cuda" in mi.variant()


def set_custom_imgui_style():
    def style_cb():
        """
        Theme is "Bess Dark" from @shivang51, released under MIT license:
        https://github.com/shivang51/bess/blob/a74d78e78ee4678b03582181905e00c1094c3d18/src/Bess/src/settings/themes.cpp
        Includes minor modifications.
        """
        style = psim.GetStyle()

        style.WindowRounding = 3.0
        style.FrameRounding = 3.0
        style.GrabRounding = 3.0
        style.TabRounding = 3.0
        style.PopupRounding = 3.0
        style.ScrollbarRounding = 3.0
        style.WindowPadding = (8, 8)
        style.FramePadding = (6, 4)
        style.ItemSpacing = (8, 6)
        style.PopupBorderSize = 0.0

        style.ScaleAllSizes(ps.get_ui_scale())

        # Primary background
        style.Colors[psim.ImGuiCol_WindowBg] = (0.07, 0.07, 0.09, 1.00)
        style.Colors[psim.ImGuiCol_MenuBarBg] = (0.12, 0.12, 0.15, 1.00)

        style.Colors[psim.ImGuiCol_PopupBg] = (0.18, 0.18, 0.22, 1.00)

        # Headers
        style.Colors[psim.ImGuiCol_Header] = (0.18, 0.18, 0.22, 1.00)
        style.Colors[psim.ImGuiCol_HeaderHovered] = (0.30, 0.30, 0.40, 1.00)
        style.Colors[psim.ImGuiCol_HeaderActive] = (0.25, 0.25, 0.35, 1.00)

        # Buttons
        style.Colors[psim.ImGuiCol_Button] = (0.20, 0.22, 0.27, 1.00)
        style.Colors[psim.ImGuiCol_ButtonHovered] = (0.30, 0.32, 0.40, 1.00)
        style.Colors[psim.ImGuiCol_ButtonActive] = (0.35, 0.38, 0.50, 1.00)

        # Frame BG
        style.Colors[psim.ImGuiCol_FrameBg] = (0.15, 0.15, 0.18, 1.00)
        style.Colors[psim.ImGuiCol_FrameBgHovered] = (0.22, 0.22, 0.27, 1.00)
        style.Colors[psim.ImGuiCol_FrameBgActive] = (0.25, 0.25, 0.30, 1.00)

        # Tabs
        style.Colors[psim.ImGuiCol_Tab] = (0.18, 0.18, 0.22, 1.00)
        style.Colors[psim.ImGuiCol_TabHovered] = (0.35, 0.35, 0.50, 1.00)
        style.Colors[psim.ImGuiCol_TabUnfocused] = (0.13, 0.13, 0.17, 1.00)
        style.Colors[psim.ImGuiCol_TabUnfocusedActive] = (0.20, 0.20, 0.25, 1.00)

        # Title
        style.Colors[psim.ImGuiCol_TitleBg] = (0.12, 0.12, 0.15, 1.00)
        style.Colors[psim.ImGuiCol_TitleBgActive] = (0.15, 0.15, 0.20, 1.00)
        style.Colors[psim.ImGuiCol_TitleBgCollapsed] = (0.10, 0.10, 0.12, 1.00)

        # Borders
        style.Colors[psim.ImGuiCol_Border] = (0.20, 0.20, 0.25, 0.50)
        style.Colors[psim.ImGuiCol_BorderShadow] = (0.00, 0.00, 0.00, 0.00)

        # Text
        style.Colors[psim.ImGuiCol_Text] = (0.90, 0.90, 0.95, 1.00)
        style.Colors[psim.ImGuiCol_TextDisabled] = (0.50, 0.50, 0.55, 1.00)

        # Highlights
        style.Colors[psim.ImGuiCol_CheckMark] = (0.50, 0.70, 1.00, 1.00)
        style.Colors[psim.ImGuiCol_SliderGrab] = (0.50, 0.70, 1.00, 1.00)
        style.Colors[psim.ImGuiCol_SliderGrabActive] = (0.60, 0.80, 1.00, 1.00)
        style.Colors[psim.ImGuiCol_ResizeGrip] = (0.50, 0.70, 1.00, 0.50)
        style.Colors[psim.ImGuiCol_ResizeGripHovered] = (0.60, 0.80, 1.00, 0.75)
        style.Colors[psim.ImGuiCol_ResizeGripActive] = (0.70, 0.90, 1.00, 1.00)

        # Scrollbar
        style.Colors[psim.ImGuiCol_ScrollbarBg] = (0.10, 0.10, 0.12, 1.00)
        style.Colors[psim.ImGuiCol_ScrollbarGrab] = (0.30, 0.30, 0.35, 1.00)
        style.Colors[psim.ImGuiCol_ScrollbarGrabHovered] = (0.40, 0.40, 0.50, 1.00)
        style.Colors[psim.ImGuiCol_ScrollbarGrabActive] = (0.45, 0.45, 0.55, 1.00)

    ps.set_configure_imgui_style_callback(style_cb)
