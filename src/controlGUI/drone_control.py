#!/usr/bin/env python3
import rospy
from pymavlink import mavutil
from dronekit import connect, VehicleMode, LocationGlobal
import time
import os
import math


def empty_print(string):
    pass


def initialize_drone(connection_string: str, print_statement=empty_print):
    """
    a method that initializes a drone vehicle and returns a vehicle
    that is ready to arm
    :param print_statement: put print in this field if you want to print the info
    :param connection_string: str
    :return: dronekit vehicle
    """
    try:
        vehicle = connect(connection_string, wait_ready=True)
        print_statement(f"connected to vehicle on: {connection_string}")
    except:
        print_statement("running the shell to authorize USB connection")
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


def change_mode(mode: str, print_statement=empty_print):
    print_statement(f'changed mode from {vehicle.mode}')
    vehicle.mode = VehicleMode(mode)
    while vehicle.mode != VehicleMode(mode):
        print_statement("mode hasn't changed yet")
        time.sleep(.1)
    print_statement(f'mode changed to {vehicle.mode}')


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

# def status_sender():
#     # ros topic
#     print('starting to send drone info')
#     pub = rospy.Publisher('drone_status_topic', drone_status, queue_size=10)
#     rate = rospy.Rate(2)
#     # getting msg
#     while not rospy.is_shutdown():
#         msg = drone_status()
#         get_drone_status(msg)
#         pub.publish(msg)
#         rate.sleep()
#
#
# def change_mode(mode):
#     vehicle.mode = VehicleMode(mode)
#
#
# def arm_drone(arm):
#     vehicle.arm()  # Arming my be some other way
#
#
# def set_airspeed(speed):
#     vehicle.airspeed = speed
#
#
# def take_off(do):
#     if do:
#         vehicle.simple_takeoff(2)
#
#
# def land(do):
#     if do:  # land if do is true
#         vehicle.mode = VehicleMode('Land')
#
#
# def move(fwd, left):  # in m/s
#     start = geopy.Point(latitude=vehicle.location.global_frame.lat, longitude=vehicle.location.global_frame.lon)
#
#     d = geopy.distance.distance(kilometers=(abs(fwd) + abs(left)) / 1000)
#
#     if fwd > 0:
#         final = d.destination(point=start, bearing=vehicle.heading)
#     elif fwd < 0:
#         final = d.destination(point=start, bearing=(vehicle.heading + 180) % 360)
#     elif left > 0:
#         final = d.destination(point=start, bearing=(vehicle.heading + 90) % 360)
#     else:
#         final = d.destination(point=start, bearing=(vehicle.heading + 90) % 360)
#
#     target_location = LocationGlobal(final.latitude, final.longitude, vehicle.location.global_frame.alt)
#     vehicle.simple_goto(target_location)
#
#
# def rise(dist):
#     target_location = LocationGlobal(vehicle.location.global_frame.lat, vehicle.location.global_frame.lon,
#                                      vehicle.location.global_frame.alt + dist)
#     vehicle.simple_goto(target_location)
#
#
# def condition_yaw(heading, relative=False):
#     if relative:
#         is_relative = 1  # yaw relative to direction of travel
#     else:
#         is_relative = 0  # yaw is an absolute angle
#     # create the CONDITION_YAW command using command_long_encode()
#     msg = vehicle.message_factory.command_long_encode(
#         0, 0,  # target system, target component
#         mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # command
#         0,  # confirmation
#         heading,  # param 1, yaw in degrees
#         0,  # param 2, yaw speed deg/s
#         1,  # param 3, direction -1 ccw, 1 cw
#         is_relative,  # param 4, relative offset 1, absolute angle 0
#         0, 0, 0)  # param 5 ~ 7 not used
#     # send command to vehicle
#     vehicle.send_mavlink(msg)
#
#
# def yaw(angle):
#     if angle != 0:
#         condition_yaw(angle, relative=True)
#
#
# def read_commands(data: drone_commands):
#     change_mode(data.mode)
#     arm_drone(data.arm)
#     set_airspeed(data.airspeed)
#     take_off(data.takeoff)
#     land(data.land)
#     move(data.forward, data.leftward)
#     rise(data.rise)
#     yaw(data.angle)
#
#
# def get_drone_commands():
#     rospy.Subscriber("drone_command_topic", drone_commands, read_commands)
#     rospy.spin()
#
#
# if __name__ == '__main__':
#     rospy.init_node('drone_controller', anonymous=True)
#
#     status_sending = threading.Thread(target=status_sender)
#     status_sending.start()
#
#     # get_drone_commands()
#
#     rospy.spin()
#
#     vehicle.close()


''' main '''


conn_str = '/dev/ttyACM0'
vehicle = initialize_drone(conn_str)
