#!/usr/bin/env python3

import rospy
from perchingDrone.msg import core_command, hook_command, hook_status


if __name__ == '__main__':
    rospy.init_node("controller")

    status_topic = rospy.Publisher('status_topic', hook_status, queue_size=5)

    rate = rospy.Rate(1)

    msg = hook_status()
    msg.hook_open = True
    msg.core_open = False

    while not rospy.is_shutdown():
        msg.hook_open = not msg.hook_open
        msg.core_open = not msg.core_open

        status_topic.publish(msg)

        rate.sleep()