#!/bin/sh
if [ $# -eq 0 ]
  then
    echo "Please provide the hex"
    exit 1
fi
avrdude -p m2560 -c stk500v2 -P /dev/ttyACM0 -b 115200 -F -D -U flash:w:$1
