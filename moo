#!/bin/bash
(roscore) &
(rosrun stage_ros stageros willow-four-erratics.world) &
./nodes/potential_fields.py
