
#!/bin/bash
set -e

# ======================================================================
#  flash.sh ? Upload Arduino firmware to Base + Arm controllers
# ======================================================================

BASE_PORT=/dev/ttyACM0
ARM_PORT=/dev/ttyACM1

BASE_SKETCH=~/rover_ws/src/rover_base/arduino/base_controller/base_controller.ino
ARM_SKETCH=~/rover_ws/src/rover_arm/arduino/arm_controller/arm_controller.ino

echo -e "\033[0;32m=== Flashing Arduino Firmware ===\033[0m"

# Verify arduino-cli
if ! command -v arduino-cli &>/dev/null; then
    echo -e "\033[0;31marduino-cli is not installed.\033[0m"
    echo "Install with:"
    echo "  curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh"
    exit 1
fi

echo "Updating core index..."
arduino-cli core update-index

echo "Installing AVR core if needed..."
arduino-cli core install arduino:avr

# ---------------------- Flash Base Mega ----------------------
echo -e "\n\033[0;32mFlashing Base Arduino Mega...\033[0m"
arduino-cli compile --fqbn arduino:avr:mega "$BASE_SKETCH"
arduino-cli upload -p "$BASE_PORT" --fqbn arduino:avr:mega "$BASE_SKETCH"

# ---------------------- Flash Arm Mega ----------------------
echo -e "\n\033[0;32mFlashing Arm Arduino Mega...\033[0m"
arduino-cli compile --fqbn arduino:avr:mega "$ARM_SKETCH"
arduino-cli upload -p "$ARM_PORT" --fqbn arduino:avr:mega "$ARM_SKETCH"

echo -e "\n\033[0;32m=== Firmware Upload Complete ===\033[0m"
