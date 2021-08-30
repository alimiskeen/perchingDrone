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
        if self.is_hook_open:
            #close the hook
            self.is_hook_open = False
        else:
            # open hook
            self.is_hook_open = True

    def on_L1_press(self):
        """ Open and Close the coil """
        if self.is_core_open:
            # close core
            self.is_core_open = False
        else:
            # open core
            self.is_core_open = True


    """ All the methods to control the right joystick """

    def on_R3_down(self, value):
        self.forward = value / 33000.0
        self._write_joystick_data()

    def on_R3_up(self, value):
        self.forward = value / 33000.0
        self._write_joystick_data()

    def on_R3_left(self, value):
        self.sideways = value / 33000.0
        self._write_joystick_data()

    def on_R3_right(self, value):
        self.sideways = value / 33000.0
        self._write_joystick_data()

    def on_R3_x_at_rest(self):
        self.sideways = 0.0
        self._write_joystick_data()

    def on_R3_y_at_rest(self):
        self.forward = 0.0
        self._write_joystick_data()

    """ All the methods to control the left joystick """

    def on_L3_down(self, value):
        self.rise = value / 33000.0
        self._write_joystick_data()

    def on_L3_up(self, value):
        self.rise = value / 33000.0
        self._write_joystick_data()

    def on_L3_left(self, value):
        self.drone.yaw(value / 3300.0)

    def on_L3_right(self, value):
        self.drone.yaw(value / 3300.0)

    def on_L3_x_at_rest(self):
        self.drone.yaw(0.0)

    def on_L3_y_at_rest(self):
        self.rise = 0.0
        self._write_joystick_data()

    def _write_joystick_data(self):
        self.drone.move(self.sideways, self.forward, self.rise)