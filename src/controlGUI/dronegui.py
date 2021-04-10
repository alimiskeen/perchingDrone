#!/usr/bin/env python3

import tkinter as tk
import rospy
import threading
from perchingDrone.msg import drone_commands, drone_status

drone_command_topic = rospy.Publisher('drone_command_topic', drone_commands, queue_size=10)

root = tk.Tk()
root.title('Drone Controller')
root.geometry('1000x1000')

frame = tk.Frame(root)
frame.pack()

# data from the drone
bat_title = tk.Label(frame, text='Battery = 0.0v')
bat_title.grid(row=0, column=0, padx=10, pady=5)

armable_title = tk.Label(frame, text='Not Armable')
armable_title.grid(row=1, column=0, padx=10, pady=5)

armed_title = tk.Label(frame, text='Not Armed')
armed_title.grid(row=2, column=0, padx=10, pady=5)

system_status_title = tk.Label(frame, text='Status: Unknown')
system_status_title.grid(row=3, column=0, padx=10, pady=5)

mode_title = tk.Label(frame, text='Mode: Unknown')
mode_title.grid(row=4, column=0, padx=10, pady=5)

fgps_title = tk.Label(frame, text='Fixed GPS: 0')
fgps_title.grid(row=5, column=0, padx=10, pady=5)

fgps_title = tk.Label(frame, text='Fixed GPS: 0')
fgps_title.grid(row=6, column=0, padx=10, pady=5)

agps_title = tk.Label(frame, text='Available GPS: 0')
agps_title.grid(row=7, column=0, padx=10, pady=5)

gnd_speed_title = tk.Label(frame, text='GND speed: 0.0')
gnd_speed_title.grid(row=8, column=0, padx=10, pady=5)

air_speed_title = tk.Label(frame, text='AIR speed: 0.0')
air_speed_title.grid(row=9, column=0, padx=10, pady=5)

local_north_title = tk.Label(frame, text='north: 0.0')
local_north_title.grid(row=10, column=0, padx=10, pady=1)
local_east_title = tk.Label(frame, text='east: 0.0')
local_east_title.grid(row=11, column=0, padx=10, pady=1)
local_down_title = tk.Label(frame, text='down: 0.0')
local_down_title.grid(row=12, column=0, padx=10, pady=1)

heading_title = tk.Label(frame, text='heading: 0')
heading_title.grid(row=13, column=0, padx=10, pady=1)

velocities_title = tk.Label(frame, text='v[x, y, z]: [0.0,0.0,0.0]')
velocities_title.grid(row=14, column=0, padx=10, pady=1)


# def update_status(data: drone_status):
#     bat_title.config(text=f'Battery = {data.bat_voltage:.2f}v')
#
#     if data.armable:
#         armable_title.config(text='Armable')
#     else:
#         armable_title.config(text='Not Armable')
#
#     if data.armed:
#         armed_title.config(text='Armed')
#     else:
#         armed_title.config(text='Not Armed')
#
#     system_status_title.config(text=data.system_status)
#     mode_title.config(text=data.mode)
#     fgps_title.config(text=f'Fixed GPS: {data.fixed_gps}')
#     agps_title.config(text=f'Available GPS: {data.available_gps}')
#     gnd_speed_title.config(text=f'GND speed: {data.gnd_speed:.2f}')
#     air_speed_title.config(text=f'AIR speed: {data.air_speed:.2f}')
#     local_north_title.config(text=f'north: {data.local_north:.2f}')
#     local_east_title.config(text=f'east: {data.local_east:.2f}')
#     local_down_title.config(text=f'down: {data.local_down:.2f}')
#     heading_title.config(text=f'heading: {data.heading}')
#     velocities_title.config(text=f'v[x, y, z]: [{data.x},{data.y},{data.z}]')


# def drone_status_listener():
#     drone_status_topic = rospy.Subscriber('drone_status_topic', drone_status, update_status)
#     rospy.spin()


# command buttons
def setup_buttons():
    takeoff_but = tk.Button(frame, text='Take Off', command=take_off, pady=10, padx=5)
    takeoff_but.grid(row=1, column=1, padx=20, pady=1)

    land_but = tk.Button(frame, text='Land', command=land, pady=10, padx=5)
    land_but.grid(row=3, column=1, padx=20, pady=1)

    yawleft_but = tk.Button(frame, text='↩', command=yaw_left, pady=10, padx=5)
    yawleft_but.grid(row=1, column=2, padx=1, pady=1)

    yawright_but = tk.Button(frame, text='↪', command=yaw_right, pady=10, padx=5)
    yawright_but.grid(row=1, column=4, padx=1, pady=1)

    forward_but = tk.Button(frame, text='↑', command=fwd, pady=10, padx=5)
    forward_but.grid(row=1, column=3, padx=1, pady=1)

    backwards_but = tk.Button(frame, text='↓', command=bwd, pady=10, padx=5)
    backwards_but.grid(row=3, column=3, padx=1, pady=1)

    leftward_but = tk.Button(frame, text='←', command=left, pady=10, padx=5)
    leftward_but.grid(row=2, column=2, padx=1, pady=1)

    rightward_but = tk.Button(frame, text='→', command=right, pady=10, padx=5)
    rightward_but.grid(row=2, column=4, padx=1, pady=1)

    arm_but = tk.Button(frame, text='Arm', command=arm, pady=10, padx=5)
    arm_but.grid(row=2, column=3, padx=1, pady=1)

    rise_but = tk.Button(frame, text='↗', command=rise, pady=10, padx=5)
    rise_but.grid(row=1, column=5, padx=1, pady=1)

    low_but = tk.Button(frame, text='↘', command=low, pady=10, padx=5)
    low_but.grid(row=3, column=5, padx=1, pady=1)

    mode_title = tk.Label(frame, text='Mode: ')
    mode_title.grid(row=4, column=1, padx=10, pady=5)
    mode_entry = tk.Entry(frame)  # TODO: add a command
    mode_entry.grid(row=4, column=2, padx=10, pady=5, columnspan=4)
    mode_entry.insert(0, 'GUIDED')

    airspeed_title = tk.Label(frame, text='Airspeed: ')
    airspeed_title.grid(row=5, column=1, padx=10, pady=5)
    airspeed_entry = tk.Entry(frame)  # TODO: add a command
    airspeed_entry.grid(row=5, column=2, padx=10, pady=5, columnspan=4)
    airspeed_entry.insert(0, '0.5')


def take_off():
    msg = drone_commands()
    default_msg(msg)
    msg.takeoff = True
    drone_command_topic.publish(msg)


def land():
    msg = drone_commands()
    default_msg(msg)
    msg.land = True
    drone_command_topic.publish(msg)


def yaw_left():
    msg = drone_commands()
    default_msg(msg)
    msg.angle = -5
    drone_command_topic.publish(msg)


def yaw_right():
    msg = drone_commands()
    default_msg(msg)
    msg.angle = 5
    drone_command_topic.publish(msg)


def fwd():
    msg = drone_commands()
    default_msg(msg)
    msg.forward = .2
    drone_command_topic.publish(msg)


def bwd():
    msg = drone_commands()
    default_msg(msg)
    msg.forward = -.2
    drone_command_topic.publish(msg)


def left():
    msg = drone_commands()
    default_msg(msg)
    msg.leftward = .2
    drone_command_topic.publish(msg)


def right():
    msg = drone_commands()
    default_msg(msg)
    msg.angle = -.2
    drone_command_topic.publish(msg)


def rise():
    msg = drone_commands()
    default_msg(msg)
    msg.rise = .2
    drone_command_topic.publish(msg)


def low():
    msg = drone_commands()
    default_msg(msg)
    msg.rise = -.2
    drone_command_topic.publish(msg)


def arm():
    msg = drone_commands()
    default_msg(msg)
    msg.arm = True
    drone_command_topic.publish(msg)


def default_msg(msg: drone_commands):
    msg.mode = 'GUIDED'
    msg.arm = False
    msg.airspeed = 0.5
    msg.takeoff = False
    msg.land = False
    msg.forward = 0.0
    msg.leftward = 0.0
    msg.rise = 0.0
    msg.angle = 0


if __name__ == '__main__':
    rospy.init_node('controlsGUI')

    setup_buttons()

    # status_recieving = threading.Thread(target=drone_status_listener)
    # status_recieving.start()

    root.mainloop()
