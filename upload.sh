#!/bin/bash

elffile=$1
hexfile=${elffile%.*}.hex
avr-size $elffile
avr-objcopy -O ihex $elffile $hexfile
pymcuprog erase
pymcuprog write  -f $hexfile

socat /dev/ttyACM0,rawer,b115200 STDOUT | ../defmt/target/release/defmt-print -e $elffile