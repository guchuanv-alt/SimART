#!/usr/bin/env bash
set -e

REPO="guchuanv-alt/SimART"
ASSET_NAME="SimART_sample_maps.zip"

# This script lives at the project root, so PROJECT_ROOT is the script directory.
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

DOWNLOAD_URL="https://github.com/${REPO}/releases/latest/download/${ASSET_NAME}"
ZIP_PATH="${PROJECT_ROOT}/${ASSET_NAME}"

echo "Project root: ${PROJECT_ROOT}"
echo "Downloading latest sample maps..."
echo "${DOWNLOAD_URL}"

if command -v curl >/dev/null 2>&1; then
    curl -L --fail -o "${ZIP_PATH}" "${DOWNLOAD_URL}"
elif command -v wget >/dev/null 2>&1; then
    wget -O "${ZIP_PATH}" "${DOWNLOAD_URL}"
else
    echo "Error: curl or wget is required."
    exit 1
fi

echo "Extracting sample maps..."
unzip -o "${ZIP_PATH}" -d "${PROJECT_ROOT}"

echo "Done."
echo "Sample maps are available at:"
echo "${PROJECT_ROOT}/SimART_sample_maps"
