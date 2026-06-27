#!/usr/bin/env bash
set -uo pipefail

REPO_ROOT="/home/ubuntu2004/catkin_ws/src/SimART"
PY="/home/ubuntu2004/miniconda3/envs/SimART/bin/python"
SCRIPT="SimART_GUI/scripts/sionna_ckm_generator.py"
BS="SimART_sample_maps/BigCitySample/config/bs_list_simplified.json"
SCENE_DIR="SimART_sample_maps/BigCitySample/BigCitySample_strict_review_sionna"
OUT="SimART_sample_maps/BigCitySample/output/ckm_sensitivity"
LOG_DIR="${OUT}/logs"

cd "$REPO_ROOT" || exit 1
mkdir -p "$LOG_DIR"
export MPLCONFIGDIR=/tmp

run_one() {
  local label="$1"
  local xml="$2"
  local log="${LOG_DIR}/${label}.log"

  {
    echo "========== START ${label} =========="
    date
    echo "scene=${SCENE_DIR}/${xml}"
    echo "output=${OUT}/${label}"

    "$PY" "$SCRIPT" \
      --scene-path "${SCENE_DIR}/${xml}" \
      --bs-list-json "$BS" \
      --metric power_db \
      --metric path_loss_db \
      --metric tau_std_ns \
      --metric sys_sinr_eff_db \
      --metric best_bs_rate_bpshz \
      --enable-sys-integration true \
      --enable-beamforming false \
      --x-min -100 --x-max 100 \
      --y-min -100 --y-max 100 \
      --z-fixed 20 \
      --resolution-m 5 \
      --output-dir "$OUT" \
      --output-root-dir "${OUT}/${label}" \
      --fc-hz 3500000000 \
      --mi-variant llvm_ad_mono_polarized \
      --tx-array-num-rows 8 \
      --tx-array-num-cols 8 \
      --rx-array-num-rows 1 \
      --rx-array-num-cols 1 \
      --max-depth 1 \
      --samples-per-src 256 \
      --render-scene-overlay false

    local status=$?
    echo "exit_code=${status}"
    if [ "$status" -eq 0 ]; then
      echo "========== DONE ${label} =========="
    else
      echo "========== FAILED ${label} =========="
    fi
    date
    return "$status"
  } 2>&1 | tee "$log"

  return "${PIPESTATUS[0]}"
}

{
  echo "========== RUN ALL START =========="
  date

  run_one 00_baseline BigCitySample_sensitivity_00_baseline_concrete_placeholder.xml || exit 1
  run_one 01_small_low BigCitySample_sensitivity_01_small_low_interaction.xml || exit 1
  run_one 02_small_medium BigCitySample_sensitivity_02_small_medium_interaction.xml || exit 1
  run_one 03_small_high BigCitySample_sensitivity_03_small_high_interaction.xml || exit 1

  echo "========== ALL DONE =========="
  date
} 2>&1 | tee "${LOG_DIR}/run_all.log"
