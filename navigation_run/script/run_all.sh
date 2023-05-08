#!/usr/bin/bash
dir="$HOME/navi_ws/src/navigation/navigation_run/script"

# 2 Pandas (Robots)
if [[ $1="b" ]]
then
    gnome-terminal --working-directory=$dir -- bash -c './basic.sh run1b'
    gnome-terminal --working-directory=$dir -- bash -c './basic.sh hub1'
    gnome-terminal --working-directory=$dir -- bash -c './basic.sh run2b'
    gnome-terminal --working-directory=$dir -- bash -c './basic.sh hub2'
elif [[ $1="g" ]]
then
    gnome-terminal --working-directory=$dir -- bash -c './basic.sh run1g'
    gnome-terminal --working-directory=$dir -- bash -c './basic.sh hub1'
    gnome-terminal --working-directory=$dir -- bash -c './basic.sh run2g'
    gnome-terminal --working-directory=$dir -- bash -c './basic.sh hub2'
else
    echo "Please specify b/g side!"
fi

# 3 PIs + 1 Panda (Side)
gnome-terminal --working-directory=$dir -- bash -c './basic.sh cam_a'
gnome-terminal --working-directory=$dir -- bash -c './basic.sh cam_b'
gnome-terminal --working-directory=$dir -- bash -c './basic.sh cam_c'
gnome-terminal --working-directory=$dir -- bash -c './basic.sh cam_central'