#!/bin/bash
dir="/home/pithreeone/catkin_ws/src/navigation/navigation_run/script"



# gnome-terminal --working-directory=$dir -- bash -c './basic.sh run1'
# gnome-terminal --working-directory=$dir -- bash -c './basic.sh hub1'
# gnome-terminal --working-directory=$dir -- bash -c './basic.sh run2'
# gnome-terminal --working-directory=$dir -- bash -c './basic.sh hub2'

gnome-terminal --working-directory=$dir -- bash -c './basic.sh cam_a'
gnome-terminal --working-directory=$dir -- bash -c './basic.sh cam_b'
gnome-terminal --working-directory=$dir -- bash -c './basic.sh cam_c'
gnome-terminal --working-directory=$dir -- bash -c './basic.sh cam_central'