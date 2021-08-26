import time

from drone import Drone
from ps4controller import DroneController
from pyPS4Controller.controller import Controller


def controller_connected():
    print("The controller has been connected")


def controller_disconnected():
    # TODO: stop the drone and land it
    print("control disconected")


if __name__ == '__main__':
    print('connecting to the drone')
    drn = Drone("/dev/ttyACM0")
    print('connected to the drone')

    print('arming drone')
    drn.arm_drone()

    time.sleep(5)

    print('disarming drone')
    drn.disarm_drone()

    # print('playing tune')
    # drn.vehicle.add_attribute_listener()

    time.sleep(2)

    print('disconnecting')
    drn.vehicle.close()

    # controller = DroneController(drn, interface="/dev/input/js0", connecting_using_ds4drv=False)
    # controller.listen(on_connect=controller_connected, on_disconnect=controller_disconnected)

    # controller = Controller(interface="/dev/input/js0", connecting_using_ds4drv=False)
    # controller.listen()
