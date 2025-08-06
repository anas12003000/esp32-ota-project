#!/bin/bash
esptool.py --chip esp32 --port /dev/ttyACM0 --baud 921600 \
  --before default_reset --after hard_reset write_flash -z --flash_mode dio --flash_freq 40m --flash_size detect \
  0x1000 .pio/build/ttgo-lora32-v1/bootloader.bin \
  0x8000 .pio/build/ttgo-lora32-v1/partitions.bin \
  0x10000 .pio/build/ttgo-lora32-v1/firmware.bin
