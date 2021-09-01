import time

from pyPS4Controller.controller import Controller  # requires pip3/pip install pyPS4Controller
from drone import Drone


class DroneController(Controller):

    def __init__(self, vehicle: Drone, **kwargs):
        Controller.__init__(self, **kwargs)
        self.drone = vehicle  # initialized drone

        self.is_hook_open = True
        self.is_core_open = True
        self.forward = 0.0  # forward is positive, backwards is negative
        self.sideways = 0.0  # going right is positive, left is negative
        self.rise = 0.0  # positive is up, negative down
        self.tilt = 0.0  # tilting right positive, tilting left is negative

    def on_x_press(self):
        """ Let the drone take off """
        self.drone.takeoff(0.6)

    def on_square_press(self):
        """ Let the drone land """
        self.drone.land()

    def on_circle_press(self):
        """ arm or disarm drone"""
        if self.drone.vehicle.armed:
            self.drone.disarm_drone()
        else:
            self.drone.arm_drone()

    def on_triangle_press(self):
        self.drone.emergency_break()

    def on_R1_press(self):
        """ Open and Close the hook """
        # if self.is_hook_open:
        #     #close the hook
        #     self.is_hook_open = False
        # else:
        #     # open hook
        #     self.is_hook_open = True
        self.rise = 0.5
        self._write_joystick_data()

    def on_L1_press(self):
        """ Open and Close the coil """
        # if self.is_core_open:
        #     # close core
        #     self.is_core_open = False
        # else:
        #     # open core
        #     self.is_core_open = True
        self.rise = -0.5
        self._write_joystick_data()


    """ All the methods to control the right joystick """

    def on_left_arrow_press(self):
        self.sideways = -0.5
        self._write_joystick_data()

    def on_right_arrow_press(self):
        self.sideways = 0.5
        self._write_joystick_data()

    def on_up_arrow_press(self):
        self.forward = 0.5
        self._write_joystick_data()

    def on_down_arrow_press(self):
        self.forward = -0.5
        self._write_joystick_data()

    def on_left_right_arrow_release(self):
        self.sideways = 0.0
        self._write_joystick_data()

    def on_up_down_arrow_release(self):
        self.forward = 0.0
        self._write_joystick_data()

    """ All the methods to control the left joystick """

    def on_options_press(self):
        self.drone.yaw(5)

    def on_share_press(self):
        self.drone.yaw(-5)


    def _write_joystick_data(self):
        self.drone.move(self.sideways, self.forward, self.rise)