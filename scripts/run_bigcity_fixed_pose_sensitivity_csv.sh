#!/usr/bin/env bash
set -euo pipefail

cd /home/ubuntu2004/catkin_ws/src/SimART

source /opt/ros/noetic/setup.bash
source /home/ubuntu2004/catkin_ws/devel/setup.bash

export MPLCONFIGDIR="${MPLCONFIGDIR:-/tmp}"

OUT_DIR="${OUT_DIR:-SimART_sample_maps/BigCitySample/output/fixed_pose_sensitivity_csv}"
INTERVAL_S="${INTERVAL_S:-10}"
MAX_DEPTH="${MAX_DEPTH:-2}"
SAMPLES_PER_SRC="${SAMPLES_PER_SRC:-64}"
MAX_NUM_PATHS_PER_SRC="${MAX_NUM_PATHS_PER_SRC:-64}"
WORKER_TIMEOUT_S="${WORKER_TIMEOUT_S:-900}"
RESUME="${RESUME:-true}"

mkdir -p "${OUT_DIR}/logs"
LOG_FILE="${OUT_DIR}/logs/run_$(date +%Y%m%d_%H%M%S).log"
exec > >(tee -a "${LOG_FILE}") 2>&1

echo "========== FIXED-POSE CSV SENSITIVITY RUN START =========="
date
echo "This is rosbag fixed-pose re-simulation, not CKM."
echo "Output directory: ${OUT_DIR}"
echo "Log file: ${LOG_FILE}"
echo "interval_s=${INTERVAL_S}"
echo "max_depth=${MAX_DEPTH}"
echo "samples_per_src=${SAMPLES_PER_SRC}"
echo "max_num_paths_per_src=${MAX_NUM_PATHS_PER_SRC}"
echo "resume=${RESUME}"

/usr/bin/python3 scripts/bigcity_fixed_pose_sensitivity_csv.py \
  --out-dir "${OUT_DIR}" \
  --resume "${RESUME}" \
  --interval-s "${INTERVAL_S}" \
  --max-depth "${MAX_DEPTH}" \
  --samples-per-src "${SAMPLES_PER_SRC}" \
  --max-num-paths-per-src "${MAX_NUM_PATHS_PER_SRC}" \
  --worker-timeout-s "${WORKER_TIMEOUT_S}"

echo "========== FIXED-POSE CSV SENSITIVITY RUN DONE =========="
date
echo "Summary: ${OUT_DIR}/rf_summary.csv"
echo "Deltas:  ${OUT_DIR}/rf_deltas_vs_baseline.csv"
echo "Anchors: ${OUT_DIR}/rf_anchor_summary.csv"
