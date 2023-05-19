#!/usr/bin/bash

DIR() {
    SOURCE="${BASH_SOURCE[0]}"
    # While $SOURCE is a symlink, resolve it
    while [ -h "$SOURCE" ]
    do
        dir="$( cd -P "$( dirname "$SOURCE" )" && pwd )"
        SOURCE="$( readlink "$SOURCE" )"
        # If $SOURCE was a relative symlink (so no "/" as prefix, need to resolve it relative to the symlink base directory
        [[ $SOURCE != /* ]] && SOURCE="$DIR/$SOURCE"
    done
    dir="$( cd -P "$( dirname "$SOURCE" )" && pwd )"
    # echo "$DIR"
}

DIR

# Open roscore
# gnome-terminal --working-directory=$dir -- bash -c './basic.sh roscore'
# sleep 5

# 2 Pandas (Robots)
if [[ $1 = "b" ]]
then
    if [[ $2 = "1" ]]
    then
        gnome-terminal --working-directory=$dir -- bash -c './basic.sh run1b1'
        sleep 5
        gnome-terminal --working-directory=$dir -- bash -c './basic.sh run2b1'
    elif [[ $2 = "2" ]]
    then
        gnome-terminal --working-directory=$dir -- bash -c './basic.sh run1b2'
        sleep 5
        gnome-terminal --working-directory=$dir -- bash -c './basic.sh run2b2' 
    fi
elif [[ $1 = "g" ]]
then
    if [[ $2 = "1" ]]
    then
        gnome-terminal --working-directory=$dir -- bash -c './basic.sh run1g1'
        sleep 5
        gnome-terminal --working-directory=$dir -- bash -c './basic.sh run2g1'
    elif [[ $2 = "2" ]]
    then
        gnome-terminal --working-directory=$dir -- bash -c './basic.sh run1g2'
        sleep 5
        gnome-terminal --working-directory=$dir -- bash -c './basic.sh run2g2' 
    fi
fi




# gnome-terminal --working-directory=$dir -- bash -c './basic.sh hub2'

# 3 PIs + 1 Panda (Side)
gnome-terminal --working-directory=$dir -- bash -c './basic.sh cam_a'
gnome-terminal --working-directory=$dir -- bash -c './basic.sh cam_b'
gnome-terminal --working-directory=$dir -- bash -c './basic.sh cam_c'
gnome-terminal --working-directory=$dir -- bash -c './basic.sh cam_central'