#!/usr/bin/env bash
set -uo pipefail

REPO_ROOT="${REPO_ROOT:-/home/ubuntu2004/catkin_ws/src/SimART}"
PROCESSOR_PY="${PROCESSOR_PY:-/usr/bin/python3}"
SIONNA_PY="${SIONNA_PY:-/home/ubuntu2004/miniconda3/envs/SimART/bin/python}"
SCRIPT="${SCRIPT:-SimART_GUI/scripts/reprocess_rosbag_with_sionna.py}"

INPUT_BAG="${INPUT_BAG:-SimART_sample_rosbags/BigCitySample_circular_flight.bag}"
POSE_TOPIC="${POSE_TOPIC:-/airsim_node/PX4/odom_local_ned}"
RF_TOPIC="${RF_TOPIC:-/airsim_gui_UErealtime/rf_observations}"
BS_LIST="${BS_LIST:-SimART_sample_maps/BigCitySample/config/bs_list_simplified.json}"
SCENE_DIR="${SCENE_DIR:-SimART_sample_maps/BigCitySample/BigCitySample_strict_review_sionna}"

OUT_DIR="${OUT_DIR:-SimART_sample_maps/BigCitySample/output/reflection_sensitivity_bags}"
LOG_DIR="${OUT_DIR}/logs"

# About 101 s / 2 s ~= 51 fixed pose samples on the current BigCity rosbag.
INTERVAL_S="${INTERVAL_S:-2}"

# Higher than the previous quick run. With specular/diffuse enabled this allows
# multi-bounce NLOS paths instead of mostly LOS-only observations.
MAX_DEPTH="${MAX_DEPTH:-4}"
SAMPLES_PER_SRC="${SAMPLES_PER_SRC:-64}"
MAX_PATHS_PER_SRC="${MAX_PATHS_PER_SRC:-128}"
WORKER_TIMEOUT_S="${WORKER_TIMEOUT_S:-3600}"
MI_VARIANT="${MI_VARIANT:-llvm_ad_mono_polarized}"

LOS="${LOS:-true}"
SPECULAR_REFLECTION="${SPECULAR_REFLECTION:-true}"
DIFFUSE_REFLECTION="${DIFFUSE_REFLECTION:-true}"
REFRACTION="${REFRACTION:-false}"
DIFFRACTION="${DIFFRACTION:-false}"
EDGE_DIFFRACTION="${EDGE_DIFFRACTION:-false}"
DIFFRACTION_LIT_REGION="${DIFFRACTION_LIT_REGION:-false}"

cd "$REPO_ROOT" || exit 1

if [ -f /opt/ros/noetic/setup.bash ]; then
  # shellcheck disable=SC1091
  source /opt/ros/noetic/setup.bash
fi
if [ -f /home/ubuntu2004/catkin_ws/devel/setup.bash ]; then
  # shellcheck disable=SC1091
  source /home/ubuntu2004/catkin_ws/devel/setup.bash
fi

mkdir -p "$LOG_DIR"
export MPLCONFIGDIR="${MPLCONFIGDIR:-/tmp}"

CONFIG_FILE="${OUT_DIR}/run_config.txt"
MANIFEST_FILE="${OUT_DIR}/bag_manifest.csv"

write_run_config() {
  {
    echo "BigCity reflection sensitivity bag run"
    echo "created_at=$(date --iso-8601=seconds)"
    echo "input_bag=${INPUT_BAG}"
    echo "pose_topic=${POSE_TOPIC}"
    echo "rf_topic=${RF_TOPIC}"
    echo "bs_list=${BS_LIST}"
    echo "scene_dir=${SCENE_DIR}"
    echo "out_dir=${OUT_DIR}"
    echo "interval_s=${INTERVAL_S}"
    echo "max_depth=${MAX_DEPTH}"
    echo "samples_per_src=${SAMPLES_PER_SRC}"
    echo "max_paths_per_src=${MAX_PATHS_PER_SRC}"
    echo "mi_variant=${MI_VARIANT}"
    echo "los=${LOS}"
    echo "specular_reflection=${SPECULAR_REFLECTION}"
    echo "diffuse_reflection=${DIFFUSE_REFLECTION}"
    echo "refraction=${REFRACTION}"
    echo "diffraction=${DIFFRACTION}"
    echo "edge_diffraction=${EDGE_DIFFRACTION}"
    echo "diffraction_lit_region=${DIFFRACTION_LIT_REGION}"
    echo "worker_timeout_s=${WORKER_TIMEOUT_S}"
  } > "$CONFIG_FILE"

  {
    echo "label,xml,output_bag,log"
  } > "$MANIFEST_FILE"
}

run_one() {
  local label="$1"
  local xml="$2"
  local output_bag="${OUT_DIR}/${label}_reflection.bag"
  local log="${LOG_DIR}/${label}.log"

  echo "${label},${SCENE_DIR}/${xml},${output_bag},${log}" >> "$MANIFEST_FILE"

  {
    echo "========== START ${label} =========="
    date
    echo "input_bag=${INPUT_BAG}"
    echo "scene=${SCENE_DIR}/${xml}"
    echo "output_bag=${output_bag}"
    echo "interval_s=${INTERVAL_S}"
    echo "max_depth=${MAX_DEPTH}"
    echo "samples_per_src=${SAMPLES_PER_SRC}"
    echo "max_paths_per_src=${MAX_PATHS_PER_SRC}"
    echo "los=${LOS}"
    echo "specular_reflection=${SPECULAR_REFLECTION}"
    echo "diffuse_reflection=${DIFFUSE_REFLECTION}"
    echo "refraction=${REFRACTION}"
    echo "diffraction=${DIFFRACTION}"

    "$PROCESSOR_PY" "$SCRIPT" \
      --input-bag "$INPUT_BAG" \
      --output-bag "$output_bag" \
      --pose-topic "$POSE_TOPIC" \
      --rf-topic "$RF_TOPIC" \
      --scene-path "${SCENE_DIR}/${xml}" \
      --bs-list-json "$BS_LIST" \
      --interval-s "$INTERVAL_S" \
      --sionna-python "$SIONNA_PY" \
      --mi-variant "$MI_VARIANT" \
      --tx-array-num-rows 8 \
      --tx-array-num-cols 8 \
      --rx-array-num-rows 1 \
      --rx-array-num-cols 1 \
      --max-depth "$MAX_DEPTH" \
      --samples-per-src "$SAMPLES_PER_SRC" \
      --max-num-paths-per-src "$MAX_PATHS_PER_SRC" \
      --synthetic-array true \
      --merge-shapes false \
      --enable-sys-integration false \
      --enable-beamforming false \
      --los "$LOS" \
      --specular-reflection "$SPECULAR_REFLECTION" \
      --diffuse-reflection "$DIFFUSE_REFLECTION" \
      --refraction "$REFRACTION" \
      --diffraction "$DIFFRACTION" \
      --edge-diffraction "$EDGE_DIFFRACTION" \
      --diffraction-lit-region "$DIFFRACTION_LIT_REGION" \
      --output-frame-key 3d \
      --worker-timeout-s "$WORKER_TIMEOUT_S" \
      --write-rf true \
      --write-sys false \
      --write-beam false

    local status=$?
    echo "exit_code=${status}"
    if [ "$status" -eq 0 ]; then
      echo "========== DONE ${label} =========="
      rosbag info "$output_bag" | grep -E "duration|size|messages:|/airsim_gui_UErealtime/rf_observations" || true
    else
      echo "========== FAILED ${label} =========="
    fi
    date
    return "$status"
  } 2>&1 | tee "$log"

  return "${PIPESTATUS[0]}"
}

write_run_config

{
  echo "========== BIGCITY REFLECTION SENSITIVITY RUN START =========="
  date
  echo "This is rosbag fixed-pose re-simulation, not CKM."
  echo "Outputs are separated from the previous CSV run:"
  echo "  ${OUT_DIR}"
  echo "The run writes exactly four output bag names and overwrites them on rerun."
  echo
  cat "$CONFIG_FILE"
  echo

  run_one 00_baseline BigCitySample_sensitivity_00_baseline_concrete_placeholder.xml || exit 1
  run_one 01_small_low BigCitySample_sensitivity_01_small_low_interaction.xml || exit 1
  run_one 02_small_medium BigCitySample_sensitivity_02_small_medium_interaction.xml || exit 1
  run_one 03_small_high BigCitySample_sensitivity_03_small_high_interaction.xml || exit 1

  echo "========== BIGCITY REFLECTION SENSITIVITY ALL DONE =========="
  echo "bags:"
  ls -lh "${OUT_DIR}"/*.bag
  echo "manifest=${MANIFEST_FILE}"
  echo "config=${CONFIG_FILE}"
  date
} 2>&1 | tee "${LOG_DIR}/run_all.log"
