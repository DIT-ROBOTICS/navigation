#!/bin/expect

#设置变量
set which [lindex $argv 0]
set user "ubuntu"
set host "192.168.50.118"
set loginpass "ditrobotics"
set cmd_prompt "]#|~]?"

spawn ssh $user@$host
#设置超时时间，单位是秒
set timeout 3

if {$which == "hub"} {
  expect {
      -re $cmd_prompt {
        send "roslaunch navigation_run hub1.launch \r"
        send "exit \r"
      }
    }
} elseif {$which == "run"} {
  expect {
      -re $cmd_prompt {
        send "roslaunch navigation_run run_robot1.launch \r"
        send "exit \r"
      }
    }
} else {
  exit 1
}

interact