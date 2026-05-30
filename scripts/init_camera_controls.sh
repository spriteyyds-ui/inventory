#!/bin/bash
# init_camera_controls.sh
# Set v4l2 parameters for left and right C100 cameras after they start.
# Called from inventory_system.launch.py via TimerAction.

set +e  # do not exit on individual command failures

RIGHT_DEVICE="$1"
LEFT_DEVICE="$2"

# v4l2 control values
AUTO_EXPOSURE=1        # 1 = Manual Mode
BRIGHTNESS=50
BACKLIGHT_COMPENSATION=0
GAIN=0
EXPOSURE_TIME_ABSOLUTE=55

CONTROLS="auto_exposure=${AUTO_EXPOSURE} brightness=${BRIGHTNESS} backlight_compensation=${BACKLIGHT_COMPENSATION} gain=${GAIN} exposure_time_absolute=${EXPOSURE_TIME_ABSOLUTE}"

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

    if ! wait_for_device "$device" "$label"; then
        return 0
    fi

    echo "${TAG} Setting ${label} v4l2 controls on ${device} ..."

    for ctrl in $CONTROLS; do
        v4l2-ctl --device="$device" --set-ctrl="$ctrl" 2>/dev/null
        if [ $? -ne 0 ]; then
            echo "${TAG} WARNING: Failed to set ${ctrl} on ${label}"
        fi
    done

    echo "${TAG} ${label} readback:"
    v4l2-ctl --device="$device" --list-ctrls-menus 2>/dev/null \
        | grep -E "auto_exposure|brightness|backlight_compensation|gain|exposure_time_absolute" \
        | while IFS= read -r line; do
            echo "${TAG}   ${label}: ${line}"
        done

    echo "${TAG} ${label} initialization complete."
}

set_camera_params "$RIGHT_DEVICE" "Right Camera"
set_camera_params "$LEFT_DEVICE" "Left Camera"

echo "${TAG} All camera controls initialized."