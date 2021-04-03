#!/usr/bin/env python3

import os
import time
from pymavlink import mavutil
from dronekit import connect, VehicleMode, LocationGlobal
from perchingDrone.msg import drone_status, drone_commands
import rospy
import threading

connection_string = '/dev/ttyACM0'
# Connect to the Vehicle.
try:
    print("Connecting to vehicle on: %s" % (connection_string,))
    vehicle = connect(connection_string, wait_ready=True)
except:
    print("need to run shell script first")
    os.system('sudo ./usbpermission.sh')
    time.sleep(3)
    vehicle = connect(connection_string, wait_ready=True)


def get_drone_status(msg):
    msg.bat_voltage = vehicle.battery.voltage
    msg.armable = vehicle.is_armable
    msg.armed = vehicle.armed
    msg.system_status = vehicle.system_status.state
    msg.mode = vehicle.mode.name
    msg.fixed_gps = vehicle.gps_0.fix_type
    msg.available_gps = vehicle.gps_0.satellites_visible
    msg.gnd_speed = vehicle.groundspeed
    msg.air_speed = vehicle.airspeed
    msg.local_north = vehicle.location.local_frame.north
    msg.local_east = vehicle.location.local_frame.east
    msg.local_down = vehicle.location.local_frame.down
    msg.heading = vehicle.heading
    msg.x = vehicle.version[0]
    msg.y = vehicle.version[1]
    msg.z = vehicle.version[2]


def status_sender():
    # ros topic
    pub = rospy.Publisher('drone_status_topic', drone_status, queue_size=10)
    rate = rospy.Rate(2)
    # getting msg
    while not rospy.is_shutdown():
        msg = drone_status()
        get_drone_status(msg)
        pub.publish(msg)
        rate.sleep()


def change_mode(mode):
    vehicle.mode = VehicleMode(mode)


def arm_drone(arm):
    vehicle.armed = arm


def set_airspeed(speed):
    vehicle.airspeed = speed


def take_off(do):
    if do:
        vehicle.simple_takeoff(2)


def land(do):
    if do:  # land if do is true
        vehicle.mode = VehicleMode('Land')


def move(fwd, left):  # in m/s
    # TODO: change distanse to lon and lat
    target_location = LocationGlobal(-40.0000, 111.225146, vehicle.location.global_frame.alt)
    vehicle.simple_goto(target_location)


def rise(dist):
    target_location = LocationGlobal(vehicle.location.global_frame.lat, vehicle.location.global_frame.lon,
                                     vehicle.location.global_frame.alt + dist)
    vehicle.simple_goto(target_location)


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


def read_commands(data: drone_commands):
    change_mode(data.mode)
    arm_drone(data.arm)
    set_airspeed(data.airspeed)
    take_off(data.takeoff)
    land(data.land)
    move(data.forward, data.leftward)
    rise(data.rise)
    yaw(data.angle)


def get_drone_commands():
    rospy.Subscriber("drone_command_topic", drone_commands, read_commands)
    rospy.spin()


if __name__ == '__main__':
    rospy.init_node('drone_controller', anonymous=True)

    status_sending = threading.Thread(target=status_sender)
    status_sending.start()

    get_drone_commands()

    vehicle.close()
