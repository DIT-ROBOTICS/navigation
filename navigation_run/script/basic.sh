#!/usr/bin/expect -f

#Set variables
set which [lindex $argv 0]
set user "ubuntu"
set user_central "ditrobotics"
set host_1 "192.168.50.118"
set host_2 "192.168.50.174"
set host_cam_a "192.168.50.171"
set host_cam_b "192.168.50.147"
set host_cam_c "192.168.50.140"
set host_cam_central "192.168.50.44"
set loginpass "ditrobotics"
set cmd_prompt "]#|~]?"

#Set timeout, unit: seconds
set timeout 3

if {$which == "hub1"} {
  spawn ssh $user@$host_1
  sleep 2
  expect "Are you sure you want to continue connecting (yes/no/\[fingerprint\])?" {send "yes\r"}
  expect "password:" {send "$loginpass\r"}
  expect {
      -re $cmd_prompt {
        send "roslaunch navigation_run hub1.launch\r"
        send "exit \r"
      }
    }
} elseif {$which == "run1b"} {
  spawn ssh $user@$host_1
  sleep 2
  expect "Are you sure you want to continue connecting (yes/no/\[fingerprint\])?" {send "yes\r"}
  expect "password:" {send "$loginpass\r"}
  expect {
      -re $cmd_prompt {
        send "roslaunch navigation_run run_robot1.launch _side:=b\r"
        send "exit \r"
      }
    }
} elseif {$which == "run1g"} {
  spawn ssh $user@$host_1
  sleep 2
  expect "Are you sure you want to continue connecting (yes/no/\[fingerprint\])?" {send "yes\r"}
  expect "password:" {send "$loginpass\r"}
  expect {
      -re $cmd_prompt {
        send "roslaunch navigation_run run_robot1.launch _side:=g\r"
        send "exit \r"
      }
    }
} elseif {$which == "hub2"} {
  spawn ssh $user@$host_2
  sleep 2
  expect "Are you sure you want to continue connecting (yes/no/\[fingerprint\])?" {send "yes\r"}
  expect "password:" {send "$loginpass\r"}
  expect {
      -re "$ " {
        send "roslaunch navigation_run hub2.launch\r"
        send "exit \r"
      }
    }
} elseif {$which == "run2b"} {
  spawn ssh $user@$host_2
  sleep 2
  expect "Are you sure you want to continue connecting (yes/no/\[fingerprint\])?" {send "yes\r"}
  expect "password:" {send "$loginpass\r"}
  expect {
      -re $cmd_prompt {
        send "roslaunch navigation_run run_robot2.launch _side:=b\r"
        send "exit \r"
      }
    }
} elseif {$which == "run2g"} {
  spawn ssh $user@$host_2
  sleep 2
  expect "Are you sure you want to continue connecting (yes/no/\[fingerprint\])?" {send "yes\r"}
  expect "password:" {send "$loginpass\r"}
  expect {
      -re $cmd_prompt {
        send "roslaunch navigation_run run_robot2.launch _side:=g\r"
        send "exit \r"
      }
    }
} elseif {$which == "cam_a"} {
  spawn ssh $user@$host_cam_a
  sleep 2
  expect "Are you sure you want to continue connecting (yes/no/\[fingerprint\])?" {send "yes\r"}
  expect "password:" {send "$loginpass\r"}
  expect {
      -re $cmd_prompt {
        send "roslaunch aruco_ros cam_a.launch \r"
        # send "exit \r"
      }
    }
} elseif {$which == "cam_b"} {
  spawn ssh $user@$host_cam_b
  sleep 2
  expect "Are you sure you want to continue connecting (yes/no/\[fingerprint\])?" {send "yes\r"}
  expect "password:" {send "$loginpass\r"}
  expect {
      -re $cmd_prompt {
        send "roslaunch aruco_ros cam_b.launch \r"
        # send "exit \r"
      }
    }
} elseif {$which == "cam_c"} {
  spawn ssh $user@$host_cam_c
  sleep 2
  expect "Are you sure you want to continue connecting (yes/no/\[fingerprint\])?" {send "yes\r"}
  expect "password:" {send "$loginpass\r"}
  expect {
      -re $cmd_prompt {
        send "roslaunch mklc side.launch \r"
        # send "exit \r"
      }
    }
} elseif {$which == "cam_central"} {
  spawn ssh $user_central@$host_cam_central
  sleep 2
  expect "Are you sure you want to continue connecting (yes/no/\[fingerprint\])?" {send "yes\r"}
  expect "password:" {send "$loginpass\r"}
  expect {
      -re $cmd_prompt {
        send "roslaunch mklc double.launch\r"
        # send "exit \r"
      }
    }
} else {
  echo "Something went wrong!"
  exit 1
}

interact
