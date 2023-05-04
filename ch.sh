#!/bin/bash -i
#Activate script using ". ch.sh [alias/IP]" or ". ch[tab] [alias/IP]"

#$1 means first argument in command [.] [ch.sh] [IP]
#                                 source  $0     $1
hostname=$1

MY_IP=$(hostname -I) #get my ip
MY_IP=${MY_IP%?} #delete the last space
MASTER=`env | grep ROS_MASTER` #see current master

# hostname=$(echo "$hostname" | tr -d '[:space:]')
# hostname=$(echo "$hostname" | tr -cd '[:print:]')

function checkIP(){
    IFS='.' read -ra octets <<< "$hostname"
    # Check if each octet is a number between 0 and 255
    for octet in "${octets[@]}"
    do
        if [ $octet -lt 0 ] || [ $octet -gt 255 ]
        then
            echo "[${MY_IP}] One of octets is out of range: '${octet}'! Try again!"
            return 1
        else
            continue
        fi
    done
    return 0
}

#Assign IP to hostname
case $1 in
    m )
        echo "[env | grep ROS_MASTER] ${MASTER}"
        return
    ;;
    my )
        hostname=${MY_IP}
        echo "[${MY_IP}] Change ROS Master back to mine, ${hostname}!"
    ;;
    p1 )
        hostname="192.168.50.118"
        echo "[${MY_IP}] Change ROS Master to ${hostname}!"
    ;;
    p2 )
        hostname="192.168.50.174"
        echo "[${MY_IP}] Change ROS Master to ${hostname}!"
    ;;
    o1 )
        hostname="192.168.50.210"
        echo "[${MY_IP}] Change ROS Master to ${hostname}!"
    ;;
    o2 )
        hostname="192.168.50.238"
        echo "[${MY_IP}] Change ROS Master to ${hostname}!"
    ;;
    # alias )
    #     hostname="192.168.50.xxx"
    #     echo "[${MY_IP}] Change ROS Master to ${hostname}!"
    # ;;
    [0-9]*.[0-9]*.[0-9]*.[0-9]* )
        if checkIP
        then
            echo "[${MY_IP}] Change ROS Master to ${hostname}!"
        else
            # echo "problem detected!"
            return
        fi
    ;;
    [Ii][Pp] | --help | "")
        echo "usage: . ch.sh {m(look up current master) | my(change back to your IP) | IP | alias}"
        return
    ;;
    * )
        echo "[${MY_IP}] Please enter IP or alias, not '${hostname}'! Try again!"
        return
    ;;
esac

# Change .bashrc but skip #commented lines
sed -i "/^[^#]/s%ROS_MASTER_URI=.*%ROS_MASTER_URI=http://${hostname}:11311%g" ~/.bashrc

# Source .bashrc(At the first line, you need to add "-i", which means opening "interactive mode".)
source ~/.bashrc