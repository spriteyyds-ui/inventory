#!/bin/bash
# init_camera_controls.sh
# Set v4l2 parameters for left and right C100 cameras after they start.
# Called from inventory_system.launch.py via TimerAction.

set +e  # do not exit on individual command failures

RIGHT_DEVICE="$1"
LEFT_DEVICE="$2"

# Confirmed C100 v4l2 control values for both cameras.
CONTROLS=(
    "white_balance_automatic=1"
    "auto_exposure=1"
    "exposure_dynamic_framerate=0"
    "exposure_time_absolute=35"
    "brightness=40"
    "contrast=40"
    "sharpness=3"
    "gain=0"
    "backlight_compensation=0"
    "gamma=100"
    "power_line_frequency=1"
)

TAG="[init_camera_controls]"

# Brief wait for device node to appear (up to 5 seconds)
wait_for_device() {
    local dev="$1"
    local label="$2"
    local retries=0
    while [ ! -e "$dev" ] && [ $retries -lt 10 ]; do
        echo "${TAG} Waiting for ${label} device ${dev} ..."
        sleep 0.5
        retries=$((retries + 1))
    done
    if [ ! -e "$dev" ]; then
        echo "${TAG} WARNING: ${label} device ${dev} not found after waiting, skipping."
        return 1
    fi
    return 0
}

set_camera_params() {
    local device="$1"
    local label="$2"
    local ctrl=""
    local ctrl_name=""

    if ! wait_for_device "$device" "$label"; then
        return 0
    fi

    echo "${TAG} Setting ${label} v4l2 controls on ${device} ..."

    for ctrl in "${CONTROLS[@]}"; do
        v4l2-ctl --device="$device" --set-ctrl="$ctrl" 2>/dev/null
        if [ $? -ne 0 ]; then
            echo "${TAG} WARNING: Failed to set ${ctrl} on ${label}"
        fi
    done

    echo "${TAG} ${label} readback:"
    for ctrl in "${CONTROLS[@]}"; do
        ctrl_name="${ctrl%%=*}"
        readback=$(v4l2-ctl --device="$device" --get-ctrl="$ctrl_name" 2>/dev/null)
        if [ $? -eq 0 ]; then
            echo "${TAG}   ${label}: ${readback}"
        else
            echo "${TAG} WARNING: Failed to read ${ctrl_name} on ${label}"
        fi
    done

    echo "${TAG} ${label} initialization complete."
}

set_camera_params "$RIGHT_DEVICE" "Right Camera"
set_camera_params "$LEFT_DEVICE" "Left Camera"

echo "${TAG} All camera controls initialized."
