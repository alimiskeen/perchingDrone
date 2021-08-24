from drone import Drone
from ps4controller import DroneController


def controller_connected():
    print("The controller has been connected")


if __name__ == '__main__':
    drone = Drone("localhost")  # change this string to actual address
    controller = DroneController(drone, interface="/dev/input/js0", connecting_using_ds4drv=False)
    controller.listen(on_connect=controller_connected)
