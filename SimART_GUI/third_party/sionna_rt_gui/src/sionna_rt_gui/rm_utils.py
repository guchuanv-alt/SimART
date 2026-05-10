#
# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
from __future__ import annotations

import io

import drjit as dr
import mitsuba as mi
import numpy as np
import matplotlib.colors
import matplotlib.pyplot as plt
import matplotlib.patheffects as path_effects


class Normalize:
    """
    Minimal DrJit alternative to matplotlib.colors.Normalize.
    """

    def __init__(self, vmin: float, vmax: float):
        self.vmin = vmin
        self.vmax = vmax

    def __call__(self, value: mi.TensorXf) -> mi.TensorXf:
        return (value - self.vmin) / (self.vmax - self.vmin)


class DrColormap:
    def __init__(self, color_map: matplotlib.colors.Colormap):
        if not isinstance(color_map, matplotlib.colors.ListedColormap):
            raise NotImplementedError(
                f"color_map must be a matplotlib.colors.ListedColormap, found {type(color_map)}"
            )
        color_map._init()
        self.N = color_map.N
        self.lut = mi.Float(color_map._lut.ravel())
        self.idx_under = color_map._i_under
        self.idx_over = color_map._i_over

    def __call__(self, value: mi.TensorXf) -> mi.TensorXf:
        sh = value.shape
        value = value.array
        is_under = value < 0.0
        is_over = value >= 1.0
        quantized = dr.select(
            is_under,
            self.idx_under,
            dr.select(is_over, self.idx_over, mi.UInt32(value * self.N)),
        )
        # Note: indices of out-of-range values are mapped to valid indices, so we do want
        # to gather even those entries.
        colors = dr.gather(mi.Vector4f, self.lut, quantized)
        return mi.TensorXf(
            dr.ravel(colors),
            shape=(*sh, 4),
        )


def radio_map_texture(
    rm_values: mi.TensorXf,
    db_scale: bool = True,
    rm_cmap: str | callable | None = None,
    vmin: float | None = None,
    vmax: float | None = None,
    premultiply_alpha: bool = True,
) -> tuple[np.ndarray, np.ndarray]:
    # Leave zero-valued regions as transparent
    valid = rm_values > 0.0
    opacity = mi.TensorXf(valid)

    # Color mapping of real values
    rm_values, normalizer, color_map = radio_map_color_mapping(
        rm_values, db_scale=db_scale, cmap=rm_cmap, vmin=vmin, vmax=vmax
    )
    texture = color_map(normalizer(rm_values))
    # Eliminate alpha channel
    texture = texture[..., :3]
    # Colors from the color map are gamma-compressed, go back to linear
    texture = dr.srgb_to_linear(texture)

    if premultiply_alpha:
        # Pre-multiply alpha to avoid fringe
        texture *= opacity[..., None]

    return texture, opacity


def radio_map_color_mapping(
    radio_map: mi.TensorXf,
    db_scale: bool = True,
    cmap: str | callable | None = None,
    vmin: float | None = None,
    vmax: float | None = None,
):
    """
    Prepare a Matplotlib color maps and normalizing helper based on the
    requested value scale to be displayed.
    Also applies the dB scaling to a copy of the radio map, if requested.

    Note that if vmin or vmax are not provided, they will be computed from
    the data, which requires a reduction.
    """
    valid = (radio_map > 0.0) & (dr.isfinite(radio_map))
    if db_scale:
        radio_map = dr.select(
            valid,
            10.0 * dr.log(radio_map) / dr.log(10.0),
            radio_map,
        )

    if (vmin is None) or (vmax is None):
        any_valid = dr.any(valid)
    if vmin is None:
        vmin = dr.min(radio_map[valid]) if any_valid else 0
    if vmax is None:
        vmax = dr.min(radio_map[valid]) if any_valid else 0

    normalizer = Normalize(vmin=vmin, vmax=vmax)

    # Make sure that invalid values are outside the color map range.
    radio_map = dr.select(valid, radio_map, vmin - 1)

    if cmap is None:
        color_map = matplotlib.colormaps.get_cmap("viridis")
    elif isinstance(cmap, str):
        color_map = matplotlib.colormaps.get_cmap(cmap)
    else:
        raise TypeError(f"Unsupported `cmap` type: {type(cmap)}")

    color_map = DrColormap(color_map)
    return radio_map, normalizer, color_map


def radio_map_colorbar_to_image(
    cmap: str, vmin: float, vmax: float, dpi: int = 100
) -> np.ndarray:
    """
    Returns a numpy array of the colorbar image in linear floating point RGBA format,
    with premultiplied alpha.
    """

    fig = plt.figure(figsize=(6, 0.2), dpi=dpi)
    ax = fig.add_axes([0.01, 0.01, 0.98, 0.8])

    _ = matplotlib.colorbar.Colorbar(
        ax=ax,
        cmap=cmap,
        norm=matplotlib.colors.Normalize(vmin=vmin, vmax=vmax),
        orientation="horizontal",
    )

    # Adjust text color, size, and add outline
    ax.tick_params(labelcolor="black", grid_color="black", which="both", labelsize=14)
    for spine in ax.spines.values():
        spine.set_color("white")
    for label in ax.get_xticklabels():
        label.set_path_effects(
            [
                path_effects.withStroke(linewidth=4, foreground="white"),
            ]
        )

    # Add dB unit label
    unit_text = ax.text(
        1.02,
        0.5,
        "dB",
        transform=ax.transAxes,
        fontsize=14,
        color="black",
        verticalalignment="center",
        horizontalalignment="left",
    )
    unit_text.set_path_effects(
        [path_effects.withStroke(linewidth=3, foreground="white")]
    )

    renderer = fig.canvas.get_renderer()
    tight_bbox = fig.get_tightbbox(renderer)

    with io.BytesIO() as buff:
        fig.savefig(
            buff, format="raw", bbox_inches="tight", pad_inches=0.1, transparent=True
        )
        buff.seek(0)
        data = np.frombuffer(buff.getvalue(), dtype=np.uint8)
    plt.close(fig)

    w, h = (tight_bbox.width + 0.2) * dpi, (tight_bbox.height + 0.2) * dpi
    data = data.reshape((int(h), int(w), 4)).copy()

    data = (data / 255.0).astype(np.float32)
    data[..., :3] *= data[..., 3:4]
    return data
