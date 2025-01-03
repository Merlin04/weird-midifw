#!/bin/sh

./build.sh
teensy_loader_cli --mcu=TEENSY41 -w -r ./weird-midifw.hex