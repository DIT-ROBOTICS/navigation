#!/usr/bin/env python
import rospy
import roslaunch

if __name__ == '__main__':
    rospy.init_node('sim_two_and_rival')
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch_file = roslaunch.rlutil.resolve_launch_arguments(['navigation_run', 'sim_two_and_rival.launch'])[0]
    launch = roslaunch.parent.ROSLaunchParent(uuid, [launch_file])
    launch.start()
    rospy.spin()
    launch.shutdown()