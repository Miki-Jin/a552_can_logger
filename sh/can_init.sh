#!/bin/bash
#@Echo off
#@Echo on
BITRATE_DEF=1000000


if [ $# = 0 ]; then
    sudo ifconfig can0 down
    sudo ip link set can0 type can bitrate $BITRATE_DEF
    sudo ifconfig can0 up
    echo "Complete !"
else
    sudo ifconfig can0 down
    sudo ip link set can0 type can bitrate $1
    sudo ifconfig can0 up
    echo "Complete !"
fi
