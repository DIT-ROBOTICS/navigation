#!/bin/bash -i
#Activate script using ". ch.sh [alias/IP]" or ". ch[tab] [alias/IP]"

#$1 means first argument in command [. ch.sh] [IP]
#                                      $0      $1
hostname=$1

MY_IP=$(hostname -I)
MY_IP=${MY_IP%?} #delete the last space

#Assign IP to hostname
case $1 in
    "p1" )
        hostname="192.168.50.118"
        echo "[${MY_IP}] Change ROS Master to ${hostname}!"
    ;;
    "p2" )
        hostname="192.168.50.174"
        echo "[${MY_IP}] Change ROS Master to ${hostname}!"
    ;;
    "o1" )
        hostname="192.168.50.210"
        echo "[${MY_IP}] Change ROS Master to ${hostname}!"
    ;;
    "o2" )
        hostname="192.168.50.238"
        echo "[${MY_IP}] Change ROS Master to ${hostname}!"
    ;;
    # "alias" )
    #     hostname="192.168.50.xxx"
    #     echo "[${MY_IP}] Change ROS Master to ${hostname}!"
    # ;;
    "192.168.50."* )
        echo "[${MY_IP}] Change ROS Master to ${hostname}!"
    ;;
    * )
        echo "[${MY_IP}] IP Format may be wrong: ${hostname}! Try again!"
        exit
    ;;
esac

# Change .bashrc but skip #commented lines
sed -i "/^[^#]/s%ROS_MASTER_URI=.*%ROS_MASTER_URI=http://${hostname}:11311%g" ~/.bashrc

# Source .bashrc(At the first line, you need to add "-i", which means open "interactive mode".)
source ~/.bashrc