#!/usr/bin/env python

import tkinter as tk
import rospy
from perchingDrone.msg import core_command, hook_command, hook_status
import threading


def open_core():
    # send rospy message
    msg = core_command()
    msg.open = True
    msg.close = False

    core_commander.publish(msg)
    print('open core')


def open_hook():
    # send rospy message
    msg = hook_command()
    msg.open = True
    msg.close = False

    hook_commander.publish(msg)
    print('open hook')


def close_core():
    # send rospy message
    msg = core_command()
    msg.open = False
    msg.close = True

    core_commander.publish(msg)
    print('close core')


def close_hook():
    # send rospy message
    msg = hook_command()
    msg.open = False
    msg.close = True

    hook_commander.publish(msg)
    print('close hook')


def setup_window(root, frame):
    core_title = tk.Label(frame, text='Core')
    hook_title = tk.Label(frame, text='Hook')
    core_title.grid(row=0, column=0, padx=15, pady=5)
    hook_title.grid(row=0, column=1, padx=15, pady=5)

    global core_status
    global hook_status
    core_status = tk.Label(frame, text='Open', fg='green', pady=5)
    hook_status = tk.Label(frame, text='Open', fg='green', pady=5)
    core_status.grid(row=1, column=0)
    hook_status.grid(row=1, column=1)

    core_open = tk.Button(frame, text='Open', command=open_core, pady=10, padx=10)
    hook_open = tk.Button(frame, text='Open', command=open_hook, pady=10, padx=10)
    core_open.grid(row=2, column=0)
    hook_open.grid(row=2, column=1)

    core_close = tk.Button(frame, text='Close', command=close_core, pady=10, padx=10)
    hook_close = tk.Button(frame, text='Close', command=close_hook, pady=10, padx=10)
    core_close.grid(row=3, column=0)
    hook_close.grid(row=3, column=1)


def status_updater(message):
    # read rospy message
    if message.hook_open:
        hook_status.config(text='Open', fg='green')
    else:
        hook_status.config(text='Closed', fg='red')

    if message.core_open:
        core_status.config(text='Open', fg='green')
    else:
        core_status.config(text='Closed', fg='red')


def app():
    root = tk.Tk()
    root.title('Hook Controller')
    root.geometry('200x200')

    frame = tk.Frame(root)
    frame.pack()

    setup_window(root, frame)

    root.mainloop()


def status_listener():
    status_topic = rospy.Subscriber('status_topic', hook_status, status_updater)
    rospy.spin()


if __name__ == '__main__':
    rospy.init_node('GUI')

    # listener topic
    update_thread = threading.Thread(target=status_listener)
    update_thread.start()

    # publisher topics
    hook_commander = rospy.Publisher('hook_commander', hook_command, queue_size=5)
    core_commander = rospy.Publisher('core_commander', core_command, queue_size=5)

    # start the gui
    app()
