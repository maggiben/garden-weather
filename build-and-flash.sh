#!/bin/bash

# Define variables
ENV_NAME="nodemcu-32s"
RASPBERRY_PI_USER="bmaggi"
RASPBERRY_PI_IP="zero.local"
REMOTE_PATH="/garden-weather/.pio/build/$ENV_NAME"
REMOTE_PORT="/dev/ttyUSB0"

# Compile the binary
~/.platformio/penv/bin/pio run -e $ENV_NAME

if [ $? -eq 0 ]; then
    echo -n "Build succeeded. Do you want to flash the device? (y/n): "
    read -r user_input
    if [[ "$user_input" == "y" || "$user_input" == "Y" ]]; then
        ./flash.sh
    else
        echo "Flash aborted by user."
    fi
else
    echo "Build process failed"
fi
