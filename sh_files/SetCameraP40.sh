#!/bin/bash

ros2 topic pub --once /$1/setcamera std_msgs/msg/UInt8MultiArray "{data: [$2,$3,$4,111]}"

