#!/usr/bin/env bash
set -e

REPO="guchuanv-alt/SimART"
ASSET_NAME="BigCitySample_circular_flight.bag"

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

BAG_DIR="${PROJECT_ROOT}/SimART_sample_rosbags"
BAG_PATH="${BAG_DIR}/${ASSET_NAME}"

DOWNLOAD_URL="https://github.com/${REPO}/releases/latest/download/${ASSET_NAME}"

mkdir -p "${BAG_DIR}"

echo "Project root: ${PROJECT_ROOT}"
echo "Downloading latest sample rosbag..."
echo "${DOWNLOAD_URL}"
echo "Output: ${BAG_PATH}"

if command -v curl >/dev/null 2>&1; then
    curl -L --fail -C - -o "${BAG_PATH}" "${DOWNLOAD_URL}"
elif command -v wget >/dev/null 2>&1; then
    wget -c -O "${BAG_PATH}" "${DOWNLOAD_URL}"
else
    echo "Error: curl or wget is required."
    exit 1
fi

echo "Done."
echo "Sample rosbag is available at:"
echo "${BAG_PATH}"