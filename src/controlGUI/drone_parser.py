#!/usr/bin/env python3

import rospy
from pymavlink import mavutil
from dronekit import connect, VehicleMode, LocationGlobal
import time
import os
import math


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


def get_location_metres(original_location, dNorth, dEast):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the
    specified `original_location`. The returned Location has the same `alt` value
    as `original_location`.

    The function is useful when you want to move the vehicle around specifying locations relative to
    the current vehicle position.
    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius=6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    return LocationGlobal(newlat, newlon,original_location.alt)


def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.

    This method is an approximation, and will not be accurate over large distances and close to the
    earth's poles. It comes from the ArduPilot test code:
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5


######3
# def goto(dNorth, dEast, vhcl, gotoFunction=land):
#     currentLocation = vhcl.location.global_relative_frame
#     targetLocation = get_location_metres(currentLocation, dNorth, dEast)
#     targetDistance = get_distance_metres(currentLocation, targetLocation)
#     gotoFunction(targetLocation)
#
#     while (vehicle.mode.name == "GUIDED") and (
#             get_distance_metres(vehicle.home_location, vehicle.location.global_frame) < radius) and (
#             vehicle.location.global_relative_frame.alt < alt_limit):
#         # Stop action if we are no longer in guided mode or outside radius.
#         remainingDistance = get_distance_metres(vehicle.location.global_frame, targetLocation)
#
#         print("Distance to target: ", remainingDistance)
#         if remainingDistance <= targetDistance * 0.1:  # Just below target, in case of undershoot.
#             print("Reached target")
#             break


time.sleep(2)

######


conn_str = '/dev/ttyACM0'
vehicle = initialize_drone(conn_str)

print('change the drone mode to loiter')
change_mode("GUIDED")

arm(vehicle)

print(f'state: {vehicle.system_status.state}')
time.sleep(5)

vehicle.airspeed = .5  # setting the airspeed to be slow

arm_and_takeoff(vehicle, 2)

print('reached altitude')

time.sleep(5)

print('going to 2, 2')
cur_loc = vehicle.location.global_relative_frame
target_location = get_location_metres(cur_loc, 2, 2)
vehicle.simple_goto(target_location)
time.sleep(5)
print('now going back home')
cur_loc = vehicle.location.global_relative_frame
target_location = get_location_metres(cur_loc, -2, -2)
vehicle.simple_goto(target_location)
time.sleep(5)


print('going back')

print('now going to land')

vehicle.mode = VehicleMode("LAND")

print('now we wait till we reach the ground')
time.sleep(20)

print('disarming the drone')
disarm(vehicle)

# print(vehicle.system_status.state) # good thing to get on the gui
print(vehicle.system_status.state)

vehicle.close()
