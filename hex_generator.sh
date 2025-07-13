#!/usr/bin/bash

readonly COMP_PATH="/d/Git_Folders/MC-Project/avr-gcc-14.1.0-x64-windows/bin"
readonly SRC_FILE="$1"
readonly BASE=${SRC_FILE%".c"}
readonly ELF_FILE="$BASE".elf

"$COMP_PATH"/avr-gcc.exe -mmcu=atmega32 -Os -Wall -o "$ELF_FILE" "$SRC_FILE"
"$COMP_PATH"/avr-objcopy -O ihex -R .eeprom "$ELF_FILE" "$BASE".hex
rm "$ELF_FILE"

# -Os -DF_CPU=1000000UL