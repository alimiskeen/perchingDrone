#!/usr/bin/env python3
import rospy
from pymavlink import mavutil
from dronekit import connect, VehicleMode, LocationGlobal
import time
import os
from std_msgs.msg import String


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


def arm_drone(arm):
    vehicle.arm()  # Arming my be some other way


def take_off(do: bool, alt: float):
    if do:
        aTargetAltitude = alt
        print("Basic pre-arm checks")
        # Don't let the user try to arm until autopilot is ready
        while not vehicle.is_armable:
            print(" Waiting for vehicle to initialise...")
            time.sleep(1)

        print("Arming motors")
        # Copter should arm in GUIDED mode
        vehicle.mode = VehicleMode("LOITER")
        vehicle.armed = True

        while not vehicle.armed:
            print(" Waiting for arming...")
            time.sleep(1)

        print("Taking off!")
        vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

        # Check that vehicle has reached takeoff altitude
        while True:
            print(" Altitude: ", vehicle.location.global_relative_frame.alt)
            # Break and return from function just below target altitude.
            if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
                print("Reached target altitude")
                break
            time.sleep(1)


def land(do):
    if do:  # land if do is true
        vehicle.mode = VehicleMode('Land')
        time.sleep(10)


def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,  # time_boot_ms (not used)
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
        0b0000111111000111,  # type_mask (only speeds enabled)
        0, 0, 0,  # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z,  # x, y, z velocity in m/s
        0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

    # send command to vehicle on 5 Hz cycle
    for x in range(0, duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)


def move(fwd, left):  # in m/s
    send_ned_velocity(left, fwd, 0, 1)


def rise(vel):
    send_ned_velocity(0, 0, -vel, 1)


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


def read_commands(data: String):
    if data.data == 'arm':
        arm(vehicle)
    elif data.data == 'takeoff':
        take_off(True, 2)
    elif data.data == 'stakeoff':
        take_off(True, .5)
    elif data.data == 'land':
        land(True)
    elif data.data == 'yawleft':
        yaw(-10)
    elif data.data == 'yawright':
        yaw(10)
    elif data.data == 'forward':
        move(.3, 0)
    elif data.data == 'backward':
        move(-.3, 0)
    elif data.data == 'left':
        move(0, .3)
    elif data.data == 'right':
        move(0, -.3)
    elif data.data == 'rise':
        rise(.3)
    elif data.data == 'low':
        rise(-.3)
    elif data.data == 'disarm':
        disarm(vehicle)
    elif data.data == 'brake':
        change_mode('BRAKE')


def get_drone_commands():
    rospy.Subscriber("drone_command_topic", String, read_commands)
    rospy.spin()


''' main '''

if __name__ == '__main__':
    rospy.init_node('drone_controller', anonymous=True)

    conn_str = '/dev/ttyACM0'
    vehicle = initialize_drone(conn_str)

    vehicle.airspeed = 1

    get_drone_commands()
