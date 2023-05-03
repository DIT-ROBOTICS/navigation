#!/bin/bash -i

hostname=$1

case $1 in
    "p1" )
        hostname="192.168.50.118"
        echo "Change ROS Master to 118!"
    ;;
    "p2" )
        hostname="192.168.50.174"
        echo "Change ROS Master to 174!"
    ;;
    "192.168.50."* )
        echo "Change ROS Master to ${hostname}!"
    ;;
    * )
        echo "IP Format may be wrong! Try again!"
        exit
    ;;
esac

sed -i "/^[^#]/s%ROS_MASTER_URI=.*%ROS_MASTER_URI=http://${hostname}:11311%g" ~/.bashrc

source ~/.bashrc