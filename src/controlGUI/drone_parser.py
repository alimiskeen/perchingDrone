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

    # print('reading GPS signal')
    # while vehicle.gps_0.satellites_visible < 5:
    #     print(f'not enough GPS sats: {vehicle.gps_0.satellites_visible}')
    #     time.sleep(1)
    #
    # print('waiting for the drone to become armable')
    # while not vehicle.is_armable:
    #     time.sleep(1)
    #     print('drone not yet armable')

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


def print_data(vhcl, print_statement=print):
    # vehicle is an instance of the Vehicle class
    print_statement("Autopilot Firmware version: %s" % vhcl.version)
    print_statement("Autopilot capabilities (supports ftp): %s" % vhcl.capabilities.ftp)
    print_statement("Global Location: %s" % vhcl.location.global_frame)
    print_statement("Global Location (relative altitude): %s" % vhcl.location.global_relative_frame)
    print_statement("Local Location: %s" % vhcl.location.local_frame)  # NED
    print_statement("Attitude: %s" % vhcl.attitude)
    print_statement("Velocity: %s" % vhcl.velocity)
    print_statement("GPS: %s" % vhcl.gps_0)
    print_statement("Groundspeed: %s" % vhcl.groundspeed)
    print_statement("Airspeed: %s" % vhcl.airspeed)
    print_statement("Gimbal status: %s" % vhcl.gimbal)
    print_statement("Battery: %s" % vhcl.battery)
    print_statement("EKF OK?: %s" % vhcl.ekf_ok)
    print_statement("Last Heartbeat: %s" % vhcl.last_heartbeat)
    print_statement("Rangefinder: %s" % vhcl.rangefinder)
    print_statement("Rangefinder distance: %s" % vhcl.rangefinder.distance)
    print_statement("Rangefinder voltage: %s" % vhcl.rangefinder.voltage)
    print_statement("Heading: %s" % vhcl.heading)
    print_statement("Is Armable?: %s" % vhcl.is_armable)
    print_statement("System status: %s" % vhcl.system_status.state)
    print_statement("Mode: %s" % vhcl.mode.name)  # settable
    print_statement("Armed: %s" % vhcl.armed)  # settable


def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)


    # send command to vehicle on 1 Hz cycle
    for x in range(0,duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)

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


conn_str = '/dev/ttyACM0'
vehicle = initialize_drone(conn_str)

print('change the drone mode to loiter')
change_mode("GUIDED")

arm(vehicle)

print(f'state: {vehicle.system_status.state}')
time.sleep(5)

vehicle.airspeed = 1  # setting the airspeed to be slow

# print_data(vehicle)


arm_and_takeoff(vehicle, 3)

print('reached altitude')

time.sleep(5)


print('moving the drone')
send_ned_velocity(.5, .5, -.5, 3)

time.sleep(5)

print('moving the drone back')
send_ned_velocity(-.5, -.5, .5, 3)

time.sleep(5)

# print('going back')

print('now going to land')

vehicle.mode = VehicleMode("LAND")

print('now we wait till we reach the ground')
time.sleep(20)

# print('disarming the drone')
disarm(vehicle)

# print(vehicle.system_status.state) # good thing to get on the gui
# print(vehicle.system_status.state)

vehicle.close()
