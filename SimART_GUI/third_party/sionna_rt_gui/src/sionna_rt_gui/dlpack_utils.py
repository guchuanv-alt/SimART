#
# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
from __future__ import annotations

import ctypes
from ctypes import pythonapi

import drjit as dr
import mitsuba as mi


# Define the DLPack tensor structure according to the DLPack specification
class DLPackTensor(ctypes.Structure):
    _fields_ = [
        ("data", ctypes.c_void_p),  # Pointer to the tensor data
        ("device", ctypes.c_int),  # Device type (CPU, GPU, etc.)
        ("device_id", ctypes.c_int),  # Device ID
        ("ndim", ctypes.c_int),  # Number of dimensions
        ("dtype", ctypes.c_uint8),  # Data type code
        ("bits", ctypes.c_uint8),  # Bits per element
        ("lanes", ctypes.c_uint16),  # Number of lanes for vector types
        ("shape", ctypes.POINTER(ctypes.c_int64)),  # Shape array
        ("strides", ctypes.POINTER(ctypes.c_int64)),  # Strides array
        ("byte_offset", ctypes.c_uint64),  # Byte offset in the buffer
    ]


def pointer_from_dlpack(tensor: mi.TensorXf | dr.ArrayBase) -> int:
    """
    Given a DrJit tensor, extract the underlying data pointer through
    the DLPack interface.

    This function will no longer be needed once DrJit exposes a direct `data_()` method.
    """
    dr.eval(tensor)
    capsule = tensor.__dlpack__()

    # Configure the PyCapsule_GetPointer function from Python's C API
    pythonapi.PyCapsule_GetPointer.argtypes = [ctypes.py_object, ctypes.c_char_p]
    pythonapi.PyCapsule_GetPointer.restype = ctypes.c_void_p

    # Extract the DLPack tensor pointer from the capsule
    # The capsule name for DLPack is typically "dltensor" but we pass None to accept any
    tensor_ptr_value = pythonapi.PyCapsule_GetPointer(capsule, b"dltensor")
    if not tensor_ptr_value:
        raise RuntimeError("Failed to get pointer from capsule")

    # Cast the void pointer to our DLPackTensor structure
    tensor_ptr = ctypes.cast(tensor_ptr_value, ctypes.POINTER(DLPackTensor))

    # Return the data pointer as an integer
    return tensor_ptr.contents.data
