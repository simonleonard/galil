#!/bin/bash

ros2 topic pub -1 /forward_position_controller/commands std_msgs/msg/Float64MultiArray  "{data: [$1,$2,$3],layout: {dim:[], data_offset: 1"}}
