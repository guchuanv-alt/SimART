#!/usr/bin/env bash
set -uo pipefail

REPO_ROOT="${REPO_ROOT:-/home/ubuntu2004/catkin_ws/src/SimART}"
PROCESSOR_PY="${PROCESSOR_PY:-/usr/bin/python3}"
SIONNA_PY="${SIONNA_PY:-/home/ubuntu2004/miniconda3/envs/SimART/bin/python}"
SCRIPT="SimART_GUI/scripts/reprocess_rosbag_with_sionna.py"

INPUT_BAG="${INPUT_BAG:-SimART_sample_rosbags/BigCitySample_circular_flight.bag}"
POSE_TOPIC="${POSE_TOPIC:-/airsim_node/PX4/odom_local_ned}"
RF_TOPIC="${RF_TOPIC:-/airsim_gui_UErealtime/rf_observations}"
BS_LIST="${BS_LIST:-SimART_sample_maps/BigCitySample/config/bs_list_simplified.json}"
SCENE_DIR="${SCENE_DIR:-SimART_sample_maps/BigCitySample/BigCitySample_strict_review_sionna}"
OUT_DIR="${OUT_DIR:-SimART_sample_maps/BigCitySample/output/fixed_pose_sensitivity}"
LOG_DIR="${OUT_DIR}/logs"

INTERVAL_S="${INTERVAL_S:-20}"
MI_VARIANT="${MI_VARIANT:-llvm_ad_mono_polarized}"
SAMPLES_PER_SRC="${SAMPLES_PER_SRC:-64}"
MAX_DEPTH="${MAX_DEPTH:-1}"
MAX_PATHS_PER_SRC="${MAX_PATHS_PER_SRC:-64}"
WORKER_TIMEOUT_S="${WORKER_TIMEOUT_S:-600}"

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

run_one() {
  local label="$1"
  local xml="$2"
  local output_bag="${OUT_DIR}/${label}_fixed_pose.bag"
  local log="${LOG_DIR}/${label}.log"

  {
    echo "========== START ${label} =========="
    date
    echo "input_bag=${INPUT_BAG}"
    echo "scene=${SCENE_DIR}/${xml}"
    echo "output_bag=${output_bag}"
    echo "interval_s=${INTERVAL_S}"
    echo "samples_per_src=${SAMPLES_PER_SRC}"

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
      --los true \
      --specular-reflection true \
      --diffuse-reflection false \
      --refraction false \
      --diffraction false \
      --edge-diffraction false \
      --diffraction-lit-region false \
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

{
  echo "========== FIXED-POSE SENSITIVITY RUN START =========="
  date
  echo "This is rosbag fixed-pose re-simulation, not CKM."

  run_one 00_baseline BigCitySample_sensitivity_00_baseline_concrete_placeholder.xml || exit 1
  run_one 01_small_low BigCitySample_sensitivity_01_small_low_interaction.xml || exit 1
  run_one 02_small_medium BigCitySample_sensitivity_02_small_medium_interaction.xml || exit 1
  run_one 03_small_high BigCitySample_sensitivity_03_small_high_interaction.xml || exit 1

  echo "========== FIXED-POSE SENSITIVITY ALL DONE =========="
  date
} 2>&1 | tee "${LOG_DIR}/run_all.log"
