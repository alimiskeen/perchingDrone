#!/usr/bin/env python3

import tkinter as tk
import rospy


def open_core():
    # send rospy message
    print('open core')


def open_hook():
    # send rospy message
    print('open hook')


def close_core():
    # send rospy message
    print('close core')


def close_hook():
    # send rospy message
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


def status_updater():
    # read rospy message

    if True:
        core_status.config(text='Closed', fg='red')
        hook_status.config(text='Closed', fg='red')
    else:
        core_status.config(text='Open', fg='green')
        hook_status.config(text='Open', fg='green')


def app():
    root = tk.Tk()
    root.title('Hook Controller')
    root.geometry('200x200')

    frame = tk.Frame(root)
    frame.pack()

    setup_window(root, frame)

    root.mainloop()


if __name__ == '__main__':
    rospy.init_node('GUI', anonymous=True)

    # link updater to the topic



    app()
