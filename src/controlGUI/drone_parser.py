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


connection_string = '/dev/ttyACM0'

try:
    print("Connecting to vehicle on: %s" % (connection_string,))
    vehicle = connect(connection_string, wait_ready=True)
except:
    print("need to run shell script first")
    os.system('sudo ./usbpermission.sh')
    time.sleep(3)
    vehicle = connect(connection_string, wait_ready=True)


change_mode("GUIDED")

vehicle.close()
