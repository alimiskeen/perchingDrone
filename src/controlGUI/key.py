#!/usr/bin/env python3

import time
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
from pymavlink import mavutil
import os
import rospy

# - Importing Tkinter: sudo apt-get install python-tk
import tkinter as tk

# -- Connect to the vehicle
print('Connecting...')
connection_string = '/dev/ttyACM0'
try:
    vehicle = connect(connection_string, wait_ready=True)
    print(f"connected to vehicle on: {connection_string}")
except:
    print("running the shell to authorize USB connection")
    os.system('sudo ./usbpermission.sh')
    time.sleep(3)
    vehicle = connect(connection_string, wait_ready=True)

# -- Setup the commanded flying speed
gnd_speed = .3  # [m/s]


def sendMessageToArduino (message):
    pub = rospy.Publisher('arduinocommands', String, queue_size=10)
    pub.publish(message)

# -- Define arm and takeoff
def arm_and_takeoff(altitude):
    while not vehicle.is_armable:
        print("waiting to be armable")
        time.sleep(1)

    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed: time.sleep(1)

    print("Taking Off")
    vehicle.simple_takeoff(altitude)

    while True:
        v_alt = vehicle.location.global_relative_frame.alt
        print(">> Altitude = %.1f m" % v_alt)
        if v_alt >= altitude - 1.0:
            print("Target altitude reached")
            break
        time.sleep(1)


# -- Define the function for sending mavlink velocity command in body frame
def set_velocity_body(vehicle, vx, vy, vz):
    """ Remember: vz is positive downward!!!
    http://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html

    Bitmask to indicate which dimensions should be ignored by the vehicle 
    (a value of 0b0000000000000000 or 0b0000001000000000 indicates that 
    none of the setpoint dimensions should be ignored). Mapping: 
    bit 1: x,  bit 2: y,  bit 3: z, 
    bit 4: vx, bit 5: vy, bit 6: vz, 
    bit 7: ax, bit 8: ay, bit 9:


    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,
        0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000111111000111,  # -- BITMASK -> Consider only the velocities
        0, 0, 0,  # -- POSITION
        vx, vy, vz,  # -- VELOCITY
        0, 0, 0,  # -- ACCELERATIONS
        0, 0)
    vehicle.send_mavlink(msg)
    vehicle.flush()


def land(do):
    if do:  # land if do is true
        vehicle.mode = VehicleMode('Land')
        time.sleep(1)


def close_hook():
    sendMessageToArduino('C104')
    print('close hook')  # remove this later



def open_hook():
    sendMessageToArduino('C103')
    print('open hook')  # remove this later


def condition_yaw(heading, relative=False):
    if relative:
        is_relative = 1  # yaw relative to direction of travel
    else:
        is_relative = 0  # yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # command
        0,  # confirmation
        heading,  # param 1, yaw in degrees
        0,  # param 2, yaw speed deg/s
        1,  # param 3, direction -1 ccw, 1 cw
        is_relative,  # param 4, relative offset 1, absolute angle 0
        0, 0, 0)  # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)


def yaw(angle):
    if angle != 0:
        condition_yaw(angle, relative=True)


# -- Key event function
def key(event):
    if event.char == event.keysym:  # -- standard keys
        if event.keysym == 'r':
            print("r pressed >> Set the vehicle to RTL")
            vehicle.mode = VehicleMode("RTL")

    else:  # -- non standard keys
        if event.keysym == 'Up':
            set_velocity_body(vehicle, gnd_speed, 0, 0)
        elif event.keysym == 'Down':
            set_velocity_body(vehicle, -gnd_speed, 0, 0)
        elif event.keysym == 'Left':
            set_velocity_body(vehicle, 0, -gnd_speed, 0)
        elif event.keysym == 'Right':
            set_velocity_body(vehicle, 0, gnd_speed, 0)
        elif event.keysym == 'r':
            set_velocity_body(vehicle, 0, 0, -gnd_speed)
        elif event.keysym == 'f':
            set_velocity_body(vehicle, 0, 0, gnd_speed)
        elif event.keysym == 'r':
            set_velocity_body(vehicle, 0, 0, -gnd_speed)
        elif event.keysym == 'g':
            land(True)
        elif event.keysym == 't':
            arm_and_takeoff(.5)
        elif event.keysym == 'y':
            close_hook()
        elif event.keysym == 'h':
            open_hook()
        elif event.keysym == 'q':
            yaw(-5)
        elif event.keysym == 'e':
            yaw(5)

# ---- MAIN FUNCTION
# - Takeoff
arm_and_takeoff(2)

# - Read the keyboard with tkinter
root = tk.Tk()
print(">> Control the drone with the arrow keys. Press r for RTL mode")
root.bind_all('<Key>', key)
root.mainloop()
