#!/bin/bash
echo "Starting serial capture..."
pio device monitor --port /dev/cu.usbserial-53220403591 --baud 115200 | tee serial_output.log 