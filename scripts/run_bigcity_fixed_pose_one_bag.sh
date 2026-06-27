#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="${REPO_ROOT:-/home/ubuntu2004/catkin_ws/src/SimART}"
cd "$REPO_ROOT"

source /opt/ros/noetic/setup.bash
source /home/ubuntu2004/catkin_ws/devel/setup.bash

PROCESSOR_PY="${PROCESSOR_PY:-/usr/bin/python3}"
SIONNA_PY="${SIONNA_PY:-/home/ubuntu2004/miniconda3/envs/SimART/bin/python}"
SCRIPT="${SCRIPT:-SimART_GUI/scripts/reprocess_rosbag_with_sionna.py}"

INPUT_BAG="${INPUT_BAG:-SimART_sample_rosbags/BigCitySample_circular_flight.bag}"
POSE_TOPIC="${POSE_TOPIC:-/airsim_node/PX4/odom_local_ned}"
RF_TOPIC="${RF_TOPIC:-/airsim_gui_UErealtime/rf_observations}"
BS_LIST="${BS_LIST:-SimART_sample_maps/BigCitySample/config/bs_list_simplified.json}"
SCENE_DIR="${SCENE_DIR:-SimART_sample_maps/BigCitySample/BigCitySample_strict_review_sionna}"
OUT_DIR="${OUT_DIR:-SimART_sample_maps/BigCitySample/output/fixed_pose_one_bag}"

LABEL="${LABEL:-03_small_high}"
XML="${XML:-}"
case "$LABEL" in
  00_baseline)
    XML="${XML:-BigCitySample_sensitivity_00_baseline_concrete_placeholder.xml}"
    ;;
  01_small_low)
    XML="${XML:-BigCitySample_sensitivity_01_small_low_interaction.xml}"
    ;;
  02_small_medium)
    XML="${XML:-BigCitySample_sensitivity_02_small_medium_interaction.xml}"
    ;;
  03_small_high)
    XML="${XML:-BigCitySample_sensitivity_03_small_high_interaction.xml}"
    ;;
  *)
    if [ -z "$XML" ]; then
      echo "Unknown LABEL=${LABEL}; set XML=YourScene.xml too." >&2
      exit 2
    fi
    ;;
esac

INTERVAL_S="${INTERVAL_S:-10}"
MAX_DEPTH="${MAX_DEPTH:-2}"
SAMPLES_PER_SRC="${SAMPLES_PER_SRC:-64}"
MAX_PATHS_PER_SRC="${MAX_PATHS_PER_SRC:-64}"
WORKER_TIMEOUT_S="${WORKER_TIMEOUT_S:-900}"
MI_VARIANT="${MI_VARIANT:-llvm_ad_mono_polarized}"

mkdir -p "$OUT_DIR"
export MPLCONFIGDIR="${MPLCONFIGDIR:-/tmp}"

OUTPUT_BAG="${OUT_DIR}/${LABEL}_fixed_pose.bag"
LOG_FILE="${OUT_DIR}/${LABEL}_fixed_pose.log"

{
  echo "========== SINGLE FIXED-POSE BAG RUN START =========="
  date
  echo "This is rosbag fixed-pose re-simulation, not CKM."
  echo "label=${LABEL}"
  echo "scene=${SCENE_DIR}/${XML}"
  echo "output_bag=${OUTPUT_BAG}"
  echo "interval_s=${INTERVAL_S}"
  echo "max_depth=${MAX_DEPTH}"
  echo "samples_per_src=${SAMPLES_PER_SRC}"

  "$PROCESSOR_PY" "$SCRIPT" \
    --input-bag "$INPUT_BAG" \
    --output-bag "$OUTPUT_BAG" \
    --pose-topic "$POSE_TOPIC" \
    --rf-topic "$RF_TOPIC" \
    --scene-path "${SCENE_DIR}/${XML}" \
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

  echo "========== SINGLE FIXED-POSE BAG RUN DONE =========="
  rosbag info "$OUTPUT_BAG" | grep -E "duration|size|messages:|/airsim_gui_UErealtime/rf_observations" || true
  date
} 2>&1 | tee "$LOG_FILE"
