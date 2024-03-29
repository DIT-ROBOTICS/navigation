#!/usr/bin/expect -f

# Set variables
set which [lindex $argv 0]
set user "ubuntu"
set user_central "ditrobotics"
set host_1 "192.168.50.118"
set host_2 "192.168.50.228"
set host_cam_a "192.168.50.171"
set host_cam_b "192.168.50.147"
set host_cam_c "192.168.50.140"
set host_cam_central "192.168.50.44"
set loginpass "ditrobotics"
set cmd_prompt "]#|~]?"

# Set timeout, delay (unit: seconds)
set timeout 3
set delay1 0
set delay2 2


if {$which == "roscore"} {
  # Open roscore
  spawn ssh $user@$host_1
  sleep $delay1
  expect "continue connecting" {send "yes\r"}
  expect "password:" {send "$loginpass\r"}

  sleep $delay2
  send "byobu -S roscore-session\r"
  sleep $delay2
  send "roscore\r"

} elseif {$which == "run1b1"} {
  spawn ssh $user@$host_1
  sleep $delay1
  expect "continue connecting" {send "yes\r"}
  expect "password:" {send "$loginpass\r"}

  sleep $delay2
  send "byobu\r"
  sleep $delay2
  send "roslaunch navigation_run run_robot1.launch side:=0 basketBot:=1\r"

} elseif {$which == "run1g1"} {
  spawn ssh $user@$host_1
  sleep $delay1
  expect "continue connecting" {send "yes\r"}
  expect "password:" {send "$loginpass\r"}

  sleep $delay2
  send "byobu\r"
  sleep $delay2
  send "roslaunch navigation_run run_robot1.launch side:=1 basketBot:=1\r"

} elseif {$which == "run2b1"} {
  spawn ssh $user@$host_2
  sleep $delay1
  expect "continue connecting" {send "yes\r"}
  expect "password:" {send "$loginpass\r"}

  sleep $delay2
  send "byobu\r"
  sleep $delay2
  send "roslaunch navigation_run run_robot2.launch side:=0 basketBot:=1\r"

} elseif {$which == "run2g1"} {
  spawn ssh $user@$host_2
  sleep $delay1
  expect "continue connecting" {send "yes\r"}
  expect "password:" {send "$loginpass\r"}

  sleep $delay2
  send "byobu\r"
  sleep $delay2
  send "roslaunch navigation_run run_robot2.launch side:=1 basketBot:=1\r"

} elseif {$which == "run1b2"} {
  spawn ssh $user@$host_1
  sleep $delay1
  expect "continue connecting" {send "yes\r"}
  expect "password:" {send "$loginpass\r"}

  sleep $delay2
  send "byobu\r"
  sleep $delay2
  send "roslaunch navigation_run run_robot1.launch side:=0 basketBot:=2\r"

} elseif {$which == "run1g2"} {
  spawn ssh $user@$host_1
  sleep $delay1
  expect "continue connecting" {send "yes\r"}
  expect "password:" {send "$loginpass\r"}

  sleep $delay2
  send "byobu\r"
  sleep $delay2
  send "roslaunch navigation_run run_robot1.launch side:=1 basketBot:=2\r"

} elseif {$which == "run2b2"} {
  spawn ssh $user@$host_2
  sleep $delay1
  expect "continue connecting" {send "yes\r"}
  expect "password:" {send "$loginpass\r"}

  sleep $delay2
  send "byobu\r"
  sleep $delay2
  send "roslaunch navigation_run run_robot2.launch side:=0 basketBot:=2\r"

} elseif {$which == "run2g2"} {
  spawn ssh $user@$host_2
  sleep $delay1
  expect "continue connecting" {send "yes\r"}
  expect "password:" {send "$loginpass\r"}

  sleep $delay2
  send "byobu\r"
  sleep $delay2
  send "roslaunch navigation_run run_robot2.launch side:=1 basketBot:=2\r"

} elseif {$which == "cam_a"} {
  spawn ssh $user@$host_cam_a
  sleep $delay1
  expect "continue connecting" {send "yes\r"}
  expect "password:" {send "$loginpass\r"}

  sleep $delay2
  send "byobu\r"
  sleep $delay2
  send "roslaunch aruco_ros cam_a.launch \r"

} elseif {$which == "cam_b"} {
  spawn ssh $user@$host_cam_b
  sleep $delay1
  expect "continue connecting" {send "yes\r"}
  expect "password:" {send "$loginpass\r"}

  sleep $delay2
  send "byobu\r"
  sleep $delay2
  send "roslaunch aruco_ros cam_b.launch \r"

} elseif {$which == "cam_c"} {
  spawn ssh $user@$host_cam_c
  sleep $delay1
  expect "continue connecting" {send "yes\r"}
  expect "password:" {send "$loginpass\r"}

  sleep $delay2
  send "byobu\r"
  sleep $delay2
  send "roslaunch mklc side.launch \r"

} elseif {$which == "cam_central"} {
  spawn ssh $user_central@$host_cam_central
  sleep $delay1
  expect "continue connecting" {send "yes\r"}
  expect "password:" {send "$loginpass\r"}
  
  sleep $delay2
  send "byobu\r"
  sleep $delay2
  send "roslaunch mklc double.launch\r"

} else {
  echo "Something went wrong!"
  exit 1
}

interact

# Seperated hub.launch
  # elseif {$which == "hub2"} {
  #   spawn ssh $user@$host_2
  #   sleep $delay1
  #   expect "continue connecting" {send "yes\r"}
  #   expect "password:" {send "$loginpass\r"}
  #   sleep $delay2
  #   send "byobu\r"
  #   sleep $delay2
  #   send "roslaunch navigation_run hub2.launch\r"
  #   # expect {
  #   #     -re "$ " {
  #   #       # send "roslaunch navigation_run hub2.launch\r"
  #   #       send "exit \r"
  #   #     }
  #   #   }
  # } 
# /Seperated hub.launch
