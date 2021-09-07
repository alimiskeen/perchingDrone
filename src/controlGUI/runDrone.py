import time

from drone import Drone
from ps4controller import DroneController
from TerminalControl import TerminalControl


def controller_connected():
    print("The controller has been connected")


def controller_disconnected():
    # TODO: stop the drone and land it
    print("control disconected")


if __name__ == '__main__':
    print('connecting to the drone')
    drn = Drone("/dev/ttyACM0")
    print('connected to the drone')

    # drn.arm_drone()

    # drn.takeoff(2)

    # drn.yaw(180)

    # time.sleep(5)

    # drn.yaw(-180)

    # time.sleep(5)

    # drn.land()

    # time.sleep(5)

    # drn.disarm_drone()

    ctrl = TerminalControl(drn)
    ctrl.listen() # there is a problem with the negative yaw
