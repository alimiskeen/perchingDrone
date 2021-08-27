from pyPS4Controller.controller import Controller  # requires pip3/pip install pyPS4Controller
from drone import Drone


class DroneController(Controller):

    def __init__(self, vehicle: Drone, **kwargs):
        Controller.__init__(self, **kwargs)
        self.drone = vehicle  # initialized drone
        self.is_hook_open = True
        self.is_core_open = True

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
        pass

    def on_R3_up(self, value):
        pass

    def on_R3_left(self, value):
        pass

    def on_R3_right(self, value):
        pass

    def on_R3_x_at_rest(self):
        pass

    def on_R3_y_at_rest(self):
        pass


    """ All the methods to control the left joystick """

    def on_L3_down(self, value):
        pass

    def on_L3_up(self, value):
        pass

    def on_L3_left(self, value):
        pass

    def on_L3_right(self, value):
        pass

    def on_L3_x_at_rest(self):
        pass

    def on_L3_y_at_rest(self):
        pass
