from pyPS4Controller.controller import Controller  # requires pip3/pip install pyPS4Controller
from drone import Drone


class DroneController(Controller):

    def __init__(self, vehicle: Drone, **kwargs):
        Controller.__init__(self, **kwargs)
        self.vehicle = vehicle
        # init connection to the drone

    def on_x_press(self):
        """ Let the drone take off """
        pass

    def on_square_press(self):
        """ Let the drone land """
        pass

    def on_circle_press(self):
        """ arm or disarm drone"""
        pass

    def on_R1_press(self):
        """ Open the hook """
        pass

    def on_R2_press(self, value):
        """ Close the hook """
        pass

    def on_L1_press(self):
        """ Open the coil """
        pass

    def on_L2_press(self, value):
        """ Close the coil """
        pass

    """ All the methods to control the right joystick """

    def on_R3_down(self, value):
        pass

    def on_R3_up(self, value):
        pass

    def on_R3_left(self, value):
        pass

    def on_R3_right(self, value):
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


if __name__ == '__main__':
    ctrl = DroneController(interface="/dev/input/js0", connecting_using_ds4drv=False)
    ctrl.listen(timeout=60)
