#!/usr/bin/env python3

import rospy
from perchingDrone.msg import core_command, hook_command, hook_status
import threading
from std_msgs.msg import String

def sendMessageToArduino (message):
    pub = rospy.Publisher('arduinocommands', String, queue_size=10)
    pub.publish(message)


def status_sender():
    global status_changed
    status_topic = rospy.Publisher('status_topic', hook_status, queue_size=5)

    while not rospy.is_shutdown():
        if status_changed:
            msg = hook_status()
            msg.hook_open = is_hook_open
            msg.core_open = is_core_open
            status_topic.publish(msg)
            status_changed = False


def listen_hook_commander():
    hook_topic = rospy.Subscriber('hook_commander', hook_command, hook_message)
    rospy.spin()


def hook_message(msg: hook_command):
    if msg.close == msg.open:
        return

    if msg.close is True and msg.open is False:
        close_hook()
    if msg.open is True and msg.close is False:
        open_hook()


def close_hook():
    global is_hook_open
    global status_changed
    # send arduino command
    sendMessageToArduino('C103')


    print('close hook')  # remove this later
    is_hook_open = False
    status_changed = True


def open_hook():
    global is_hook_open
    global status_changed
    # send arduino command
    sendMessageToArduino('C104')

    print('open hook')  # remove this later
    is_hook_open = True
    status_changed = True


def listen_core_commander():
    core_topic = rospy.Subscriber('core_commander', core_command, core_message)
    rospy.spin()


def core_message(msg: core_command):
    if msg.close == msg.open:
        return

    if msg.close is True and msg.open is False:
        close_core()
    if msg.open is True and msg.close is False:
        open_core()


def close_core():
    global is_core_open
    global status_changed
    # send arduino command

    print('close core')  # remove this later
    is_core_open = False
    status_changed = True


def open_core():
    global is_core_open
    global status_changed
    # send arduino command

    print('open core')  # remove this later
    is_core_open = True
    status_changed = True


if __name__ == '__main__':
    rospy.init_node("controller")

    is_hook_open = True
    is_core_open = True
    status_changed = True

    hook_thread = threading.Thread(target=listen_hook_commander)
    hook_thread.start()

    core_thread = threading.Thread(target=listen_core_commander)
    core_thread.start()

    # sending status to gui thread
    status_sender()
