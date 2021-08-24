from dronekit import connect, VehicleMode
from pymavlink import mavutil
import os
import time

class Drone:

    def __init__(self, address: str):
        self.address_string = address
        self.vehicle = connect(self.address_string, wait_ready=True)  # TODO: change this later to none

        self._connect_to_drone()

    def _connect_to_drone(self):
        try:
            self.vehicle = connect(self.address_string, wait_ready=True)
        except:
            os.system('sudo ./usbpermission.sh')
            time.sleep(3)
            try:
                self.vehicle = connect(self.address_string, wait_ready=True)
            except:
                print('failed to connect to the drone')  # inform user of failure then quit
                quit()
        print(f'successfully connected to the drone: {self.vehicle}')

    def change_mode(self, mode: str):
        self.vehicle.mode = VehicleMode(mode)
        while self.vehicle.mode != VehicleMode(mode):
            time.sleep(.1)

    def arm_drone(self):
        # pre arm checks
        while not self.vehicle.is_armable:
            time.sleep(.5)

        # arming
        self.change_mode("GUIDED")
        self.vehicle.armed = True
        while not self.vehicle.armed:
            time.sleep(.1)

    def disarm_drone(self):
        self.vehicle.armed = False
        while self.vehicle.armed:
            time.sleep(.1)

    def takeoff(self, alt: float):
        if not self.vehicle.armed:
            self.arm_drone()

        self.vehicle.simple_takeoff(alt)

        while True:
            if self.vehicle.location.global_relative_frame.alt >= alt * 0.95:
                break
            time.sleep(.1)

    def land(self):
        self.vehicle.mode = VehicleMode("Land")
        last_altitude = self.vehicle.location.global_relative_frame.alt
        current_altitude = self.vehicle.location.global_relative_frame.alt
        while True:
            if current_altitude < 0.1:
                break
            time.sleep(1)  # TODO: is this right?
            current_altitude = self.vehicle.location.global_relative_frame.alt
            if abs(current_altitude - last_altitude) < .05:
                break
            last_altitude = self.vehicle.location.global_relative_frame.alt

    def move(self, x_velocity: float, y_velocity: float, z_velocity: float):
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,  # time_boot_ms (not used)
            0, 0,  # target system, target component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
            0b0000111111000111,  # type_mask (only speeds enabled)
            0, 0, 0,  # x, y, z positions (not used)
            x_velocity, y_velocity, z_velocity,  # x, y, z velocity in m/s
            0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

        self.vehicle.send_mavlink(msg)

    def yaw(self, heading):
        is_relative = 1

        msg = self.vehicle.message_factory.command_long_encode(
            0, 0,  # target system, target component
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # command
            0,  # confirmation
            heading,  # param 1, yaw in degrees
            0,  # param 2, yaw speed deg/s
            1,  # param 3, direction -1 ccw, 1 cw
            is_relative,  # param 4, relative offset 1, absolute angle 0
            0, 0, 0)  # param 5 ~ 7 not used
        # send command to vehicle
        self.vehicle.send_mavlink(msg)
