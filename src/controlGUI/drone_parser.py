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


def empty_print(text: str):
    pass


def arm(vhcl, print_method=empty_print):
    print_method('arming the drone')
    vhcl.armed = True
    while not vhcl.armed:
        time.sleep(.1)
        print_method('drone not armed yet')
    print_method('drone armed')


def disarm(vhcl, print_method=empty_print):
    print_method('disarming the drone')
    vhcl.armed = False
    while vhcl.armed:
        time.sleep(.1)
        print_method('drone not disarmed yet')
    print_method("disarmed")


# Function to arm and then takeoff to a user specified altitude
def arm_and_takeoff(vhcl, aTargetAltitude):
    print("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    while not vhcl.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vhcl.mode = VehicleMode("GUIDED")
    vhcl.armed = True

    while not vhcl.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vhcl.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    # Check that vehicle has reached takeoff altitude
    while True:
        print(" Altitude: ", vhcl.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
        if vhcl.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)


def land(vhcl):
    pass



conn_str = '/dev/ttyACM0'
vehicle = initialize_drone(conn_str)

print('change the drone mode to loiter')
change_mode("LOITER")

arm(vehicle)

print(f'state: {vehicle.system_status.state}')
time.sleep(5)

arm_and_takeoff(vehicle, 2)

print('reached altitude')

time.sleep(10)

print('now going to land')

vehicle.mode = VehicleMode("LAND")

print('now we wait till we reach the ground')
time.sleep(20)

print('disarming the drone')
disarm(vehicle)

# print(vehicle.system_status.state) # good thing to get on the gui
print(vehicle.system_status.state)

vehicle.close()
