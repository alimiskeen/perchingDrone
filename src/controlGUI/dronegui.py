#!/usr/bin/env python3

import tkinter as tk
import rospy
import threading
from perchingDrone.msg import drone_commands, drone_status

root = tk.Tk()
root.title('Hook Controller')
root.geometry('200x200')

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


def passing():
    pass


#command buttons

takeoff_but = tk.Button(frame, text='Take Off', command=passing, pady=10, padx=5)
takeoff_but.grid(row=1, column=1, padx=20, pady=1)

land_but = tk.Button(frame, text='Land', command=passing, pady=10, padx=5)
land_but.grid(row=3, column=1, padx=20, pady=1)

yawleft_but = tk.Button(frame, text='↩', command=passing, pady=10, padx=5)
yawleft_but.grid(row=1, column=2, padx=1, pady=1)

yawright_but = tk.Button(frame, text='↪', command=passing, pady=10, padx=5)
yawright_but.grid(row=1, column=4, padx=1, pady=1)

forward_but = tk.Button(frame, text='↑', command=passing, pady=10, padx=5)
forward_but.grid(row=1, column=3, padx=1, pady=1)

backwards_but = tk.Button(frame, text='↓', command=passing, pady=10, padx=5)
backwards_but.grid(row=3, column=3, padx=1, pady=1)

leftward_but = tk.Button(frame, text='←', command=passing, pady=10, padx=5)
leftward_but.grid(row=2, column=2, padx=1, pady=1)

rightward_but = tk.Button(frame, text='→', command=passing, pady=10, padx=5)
rightward_but.grid(row=2, column=4, padx=1, pady=1)

arm_but = tk.Button(frame, text='Arm', command=passing, pady=10, padx=5)
arm_but.grid(row=2, column=3, padx=1, pady=1)

rise_but = tk.Button(frame, text='↗', command=passing, pady=10, padx=5)
rise_but.grid(row=1, column=5, padx=1, pady=1)

low_but = tk.Button(frame, text='↘', command=passing, pady=10, padx=5)
low_but.grid(row=3, column=5, padx=1, pady=1)

mode_title = tk.Label(frame, text='Mode: ')
mode_title.grid(row=4, column=1, padx=10, pady=5)
mode_entry = tk.Entry(frame) # TODO: add a command
mode_entry.grid(row=4, column=2, padx=10, pady=5, columnspan=4)
mode_entry.insert(0, 'GUIDED')

airspeed_title = tk.Label(frame, text='Airspeed: ')
airspeed_title.grid(row=5, column=1, padx=10, pady=5)
airspeed_entry = tk.Entry(frame) # TODO: add a command
airspeed_entry.grid(row=5, column=2, padx=10, pady=5, columnspan=4)
airspeed_entry.insert(0, '0.5')








root.mainloop()
