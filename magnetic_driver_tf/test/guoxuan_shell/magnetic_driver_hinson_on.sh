#!/bin/bash

rostopic pub -1 /task_switch std_msgs/Header -- '{seq: 1, stamp: 0, frame_id: magnetic_driver_hinson}'
