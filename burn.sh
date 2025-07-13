#!/usr/bin/bash

BURNER_PATH="/d/Git_Folders/MC-Project/avrdude"

$BURNER_PATH/avrdude.exe -C$BURNER_PATH/avrdude.conf -v -pm32 -cusbasp -B 10 -U flash:w:"$1":i