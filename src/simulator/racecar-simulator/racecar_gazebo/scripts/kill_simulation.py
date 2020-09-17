#!/usr/bin/python
import rospy


if __name__ == "__main__":
    rospy.init_node("kill_simulation")
    args = rospy.myargv()[1:]
    timeout = int(args[0])
    rospy.sleep(2)
    rospy.sleep(timeout)
    rospy.logwarn("Timeout: Next Experiment")
    rospy.signal_shutdown("Ending current experiment")
