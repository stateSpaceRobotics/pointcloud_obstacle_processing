#!/bin/bash
(gnome-terminal -e "bash -c \"roscore\"") &
cp test.world ./testers.world
for x in `seq 1 $1`; do
	dist=$((5-$x))
	echo "minibot( pose [-7.0 $dist.0 0 0.0] name \"minibot\" color \"blue\")" >> testers.world
done
cp /dev/null obstacleLoc.txt
for x in `seq 1 $2`; do
	randX=$(((RANDOM%4)-2))	
	randY=$(((RANDOM%12)-6))
	echo $randX $randY >> obstacleLoc.txt
	echo "obstacle( pose [$randX.0 $randY.0 0 0.0] name \"obstacle\" color \"red\")" >> testers.world
done
(gnome-terminal -e "bash -c \"rosrun stage_ros stageros testers.world\"") &
cd nodes
./logansPotential.py
cd ..
