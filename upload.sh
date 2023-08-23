#!/bin/bash
#python -m esptool --chip esp8266 erase_flash
pio run --target buildfs -v -e nodemcuv2
pio run --target uploadfs -e nodemcuv2
pio run --target upload -v -e nodemcuv2
pio run -e nodemcuv2 -t monitor
