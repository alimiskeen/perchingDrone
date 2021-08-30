import time

from drone import Drone
from ps4controller import DroneController


def controller_connected():
    print("The controller has been connected")


def controller_disconnected():
    # TODO: stop the drone and land it
    print("control disconected")


if __name__ == '__main__':
    print('connecting to the drone')
    drn = Drone("/dev/ttyACM0")
    print('connected to the drone')

    # print('connecting to controller')
    # controller = DroneController(drn, interface="/dev/input/js0", connecting_using_ds4drv=False)
    # print('connected, now listening')
    # controller.listen(on_connect=controller_connected, on_disconnect=controller_disconnected)

    print('arming')
    drn.arm_drone()

    print('taking off')
    drn.takeoff(1.0)

    time.sleep(5)

    print('moving up')
    drn.move(0.0, 0.0, 0.2)

    time.sleep(5)

    print('stopping and landing')
    drn.move(0.0, 0.0, 0.0)

    drn.land()

    time.sleep(10)

    print('disarming')
    drn.disarm_drone()



    # controller = Controller(interface="/dev/input/js0", connecting_using_ds4drv=False)
    # controller.listen()
