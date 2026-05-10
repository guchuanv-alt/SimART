#
# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
import gc

import drjit as dr


def drjit_cleanup():
    gc.collect()
    gc.collect()

    dr.kernel_history_clear()
    dr.flush_malloc_cache()
    # dr.detail.malloc_clear_statistics()
    dr.detail.clear_registry()
    dr.flush_kernel_cache()

    dr.sync_thread()
    # Note: there are several flags we care about preserving, so we don't reset them here.
    # dr.set_flag(dr.JitFlag.Default, True)
