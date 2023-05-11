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
    gnome-terminal --working-directory=$dir -- bash -c './basic.sh run1b'
    gnome-terminal --working-directory=$dir -- bash -c './basic.sh run2b'
elif [[ $1 = "g" ]]
then
    gnome-terminal --working-directory=$dir -- bash -c './basic.sh run1g'
    gnome-terminal --working-directory=$dir -- bash -c './basic.sh run2g'
else
    gnome-terminal --working-directory=$dir -- bash -c './basic.sh run1b'
    gnome-terminal --working-directory=$dir -- bash -c './basic.sh run2b'
fi

# gnome-terminal --working-directory=$dir -- bash -c './basic.sh hub2'

# 3 PIs + 1 Panda (Side)
gnome-terminal --working-directory=$dir -- bash -c './basic.sh cam_a'
gnome-terminal --working-directory=$dir -- bash -c './basic.sh cam_b'
gnome-terminal --working-directory=$dir -- bash -c './basic.sh cam_c'
gnome-terminal --working-directory=$dir -- bash -c './basic.sh cam_central'