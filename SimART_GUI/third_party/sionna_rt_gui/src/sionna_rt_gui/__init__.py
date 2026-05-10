#
# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
from importlib.resources import files
import os

SOURCE_DIR = os.path.realpath(os.path.dirname(files(__package__)))
DATA_DIR = os.path.join(SOURCE_DIR, "sionna_rt_gui", "data")
SCENES_DIR = os.path.join(DATA_DIR, "scenes")
CONFIGS_DIR = os.path.join(DATA_DIR, "configs", "sionna_rt_gui")
DEFAULT_CONFIG_PATH = os.path.join(CONFIGS_DIR, "base.yaml")

__version__ = (0, 1, 1)

from . import gui
from .gui import SionnaRtGui
from .reload import AppHolder
