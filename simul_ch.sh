#!/bin/bash

#Hosts you want to connect(must have been ssh-copy-ided so no passwords)
USERNAME=ubuntu
HOSTS=""192.168.50.171" "192.168.50.147" "192.168.50.140""

#Do master changing script, so they must have ch.sh under ~/
SCRIPT="source ~/ch.sh $1"

#For same hostname, otherwise copy paste the whole script
for HOSTNAME in ${HOSTS} ; do
    ssh -l ${USERNAME} ${HOSTNAME} "${SCRIPT}"
done