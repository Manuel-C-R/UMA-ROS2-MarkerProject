#!/bin/bash

ros2 topic pub --once /$1/reset_aruco/ID std_msgs/msg/UInt8 "{data: 111}"

