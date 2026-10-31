#!/usr/bin/env bash
set -euo pipefail

script_directory="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repository_root="$(cd -- "$script_directory/.." && pwd)"
gui_directory="$repository_root/reference/sharpa-manus-sdk/client/CalibrationGUI"
gui_executable="$gui_directory/CalibrationGUI.out"
sdk_library_directory="$gui_directory/../ManusSDK/lib"
calibration_directory="${1:-$repository_root/calibration/metaglove}"

if [[ ! -d "$gui_directory" ]]; then
    echo "CalibrationGUI directory not found: $gui_directory" >&2
    exit 1
fi

gui_needs_build=false
if [[ ! -x "$gui_executable" ]]; then
    gui_needs_build=true
elif find "$gui_directory" -maxdepth 1 \
    \( -name '*.cpp' -o -name '*.hpp' -o -name 'Makefile' \) \
    -newer "$gui_executable" -print -quit | grep -q .; then
    gui_needs_build=true
fi

if [[ "$gui_needs_build" == true ]]; then
    echo "CalibrationGUI is missing or out of date; building it now..."
    (
        cd "$gui_directory"
        ./build.sh
    )
fi

if [[ -e "$calibration_directory/Calibration_left.mcal" ]] ||
   [[ -e "$calibration_directory/Calibration_right.mcal" ]]; then
    echo "Calibration output already exists in: $calibration_directory" >&2
    echo "Pass a different output directory to avoid overwriting it:" >&2
    echo "  $0 calibration/metaglove-\$(date +%Y%m%d-%H%M%S)" >&2
    exit 1
fi

mkdir -p "$calibration_directory"
calibration_directory="$(cd "$calibration_directory" && pwd)"

echo "Calibration files will be saved in:"
echo "  $calibration_directory"
echo
echo "Close any running manus_driver before continuing."
echo "In the GUI, calibrate Left Glove first, then Right Glove."
echo

cd "$calibration_directory"
__GLX_VENDOR_LIBRARY_NAME=mesa \
LIBGL_ALWAYS_SOFTWARE=1 \
LD_LIBRARY_PATH="$sdk_library_directory${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" \
    "$gui_executable"

echo
echo "Calibration GUI exited. Expected output files:"
echo "  $calibration_directory/Calibration_left.mcal"
echo "  $calibration_directory/Calibration_right.mcal"
