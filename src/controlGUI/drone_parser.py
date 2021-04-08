#!/usr/bin/env python3

import rospy
from pymavlink import mavutil
from dronekit import connect, VehicleMode, LocationGlobal
import time
import os


def change_mode(mode: str):
    # print(f'changed mode from {vehicle.mode}')
    vehicle.mode = VehicleMode(mode)
    while vehicle.mode != VehicleMode(mode):
        # print("mode hasn't changed yet")
        time.sleep(.1)
    # print(f'mode changed to {vehicle.mode}')


def initialize_drone(connection_string: str):
    """
    a method that initializes a drone vehicle and returns a vehicle
    that is ready to arm
    :param connection_string: str
    :return: dronekit vehicle
    """
    try:
        print(f"Connecting to vehicle on: {connection_string}")
        vehicle = connect(connection_string, wait_ready=True)
    except:
        print("running the shell script")
        os.system('sudo ./usbpermission.sh')
        time.sleep(3)
        vehicle = connect(connection_string, wait_ready=True)

    print('reading GPS signal')
    while vehicle.gps_0.satellites_visible < 5:
        print(f'not enough GPS sats: {vehicle.gps_0.satellites_visible}')
        time.sleep(1)

    print('waiting for the drone to become armable')
    while not vehicle.is_armable:
        time.sleep(1)
        print('drone not yet armable')

    return vehicle


conn_str = '/dev/ttyACM0'
vehicle = initialize_drone(conn_str)

print('change the drone mode to loiter')
change_mode("LOITER")

print('arming the drone')
vehicle.armed = True

while not vehicle.armed:
    time.sleep(1)
    print('drone not armed yet')

print('drone armed')
print(f'state: {vehicle.system_status.state}')

time.sleep(5)

print('disarming the drone')
vehicle.armed = False

while vehicle.armed:
    time.sleep(.5)
    print('drone not disarmed yet')

# print(vehicle.system_status.state) # good thing to get on the gui
print(vehicle.system_status.state)

vehicle.close()
