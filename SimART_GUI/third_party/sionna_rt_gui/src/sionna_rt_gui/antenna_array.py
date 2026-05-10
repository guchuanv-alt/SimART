#
# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
from __future__ import annotations

import polyscope.imgui as psim
from sionna import rt
from sionna.rt.antenna_pattern import (
    antenna_pattern_registry,
    polarization_registry,
)

from .config import AntennaArrayConfig


def _antenna_array_gui(
    name: str, cfg: AntennaArrayConfig, array: rt.AntennaArray
) -> tuple[rt.AntennaArray, bool]:
    needs_update = False

    if psim.TreeNode(name):
        # psim.Text(f"Array size: {dr.width(array.normalized_positions)}\n")

        changed, cfg.pattern_i = psim.Combo(
            f"Pattern##{name}", cfg.pattern_i, antenna_pattern_registry.list()
        )
        needs_update |= changed
        changed, cfg.polarization_i = psim.Combo(
            f"Polarization##{name}",
            cfg.polarization_i,
            polarization_registry.list(),
        )
        needs_update |= changed

        changed, (cfg.num_rows, cfg.num_cols) = psim.InputInt2(
            f"Rows, cols##{name}", (cfg.num_rows, cfg.num_cols)
        )
        needs_update |= changed
        changed, (cfg.vertical_spacing, cfg.horizontal_spacing) = psim.InputFloat2(
            f"V/H spacing##{name}", (cfg.vertical_spacing, cfg.horizontal_spacing)
        )
        needs_update |= changed

        psim.Spacing()
        psim.TreePop()

    if needs_update:
        array = cfg.create()
    return array, needs_update


def antenna_array_gui(gui: "SionnaRtGui"):
    gui.scene._tx_array, tx_changed = _antenna_array_gui(
        "TX array", gui.cfg.tx_array, gui.scene._tx_array
    )
    gui.scene._rx_array, rx_changed = _antenna_array_gui(
        "RX array", gui.cfg.rx_array, gui.scene._rx_array
    )

    if tx_changed:
        # Note: receivers don't affect radio maps.
        gui.reset_radio_map()

    if (tx_changed or rx_changed) and gui.cfg.paths.auto_update:
        gui.update_paths(clear_first=True, show=True)
