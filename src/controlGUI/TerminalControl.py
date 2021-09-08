import time
from drone import Drone
import serial


def _command_string():
    return 't: take off, l: land, o: arm/disarm, u: emergency,\n' \
           '            q: yaw left,             e: yaw right,\n' \
           'r: rise,                w: forward\n' \
           'f: low,    a: left,     s: backward,     d: right,\n' \
           '      x: hook,                      v: core'


def send_to_arduino(order: str, connection: serial.Serial):
    message = bytes(order, 'utf-8')
    connection.write(message)


class TerminalControl:

    def __init__(self, vehicle: Drone, connection: serial.Serial):
        self.drone = vehicle
        self.options = {'t': self.takeoff,
                        'l': self.land,
                        'o': self.arm,
                        'u': self.emergency,
                        'q': self.yaw_left,
                        'e': self.yaw_right,
                        'w': self.fwd,
                        'a': self.left,
                        's': self.bwd,
                        'd': self.right,
                        'r': self.rise_up,
                        'f': self.rise_down,
                        'x': self.toggle_hook,
                        'v': None,  # not connected because the core mechanism doesn't currently work
                        }
        self.arduino = connection
        self.hook_open = True

    def listen(self):
        while True:
            print(_command_string())
            command = input("Command: ")
            try:
                self.options[command]()
            except:
                print("input not recognized!")

    def takeoff(self):
        self.drone.takeoff(0.5)
        print('took off')

    def land(self):
        self.drone.land()
        print('landing')

    def arm(self):
        if not self.drone.vehicle.armed:
            self.drone.arm_drone()
            print('armed')
        else:
            self.drone.disarm_drone()
            print('disarmed')

    def emergency(self):
        self.drone.emergency_break()
        print('emergency break')

    def yaw_left(self):
        self.drone.yaw(340)
        print('yaw left')

    def yaw_right(self):
        self.drone.yaw(20)
        print('yaw right')

    def fwd(self):
        self.drone.move(0.5, 0.0, 0.0)
        time.sleep(1)
        self.drone.move(0.0, 0.0, 0.0)
        print('moved forward')

    def bwd(self):
        self.drone.move(-0.5, 0.0, 0.0)
        time.sleep(1)
        self.drone.move(0.0, 0.0, 0.0)
        print('moved backward')

    def left(self):
        self.drone.move(0.0, -0.5, 0.0)
        time.sleep(1)
        self.drone.move(0.0, 0.0, 0.0)
        print('moved leftward')

    def right(self):
        self.drone.move(0.0, 0.5, 0.0)
        time.sleep(1)
        self.drone.move(0.0, 0.0, 0.0)
        print('moved rightward')

    def rise_up(self):
        self.drone.move(0.0, 0.0, -0.5)
        time.sleep(1)
        self.drone.move(0.0, 0.0, 0.0)
        print('moved upward')

    def rise_down(self):
        self.drone.move(0.0, 0.0, 0.5)
        time.sleep(1)
        self.drone.move(0.0, 0.0, 0.0)
        print('moved downward')

    def toggle_hook(self):
        if self.hook_open:
            send_to_arduino('C104', self.arduino)
            self.hook_open = False
        else:
            send_to_arduino('C103', self.arduino)
            self.hook_open = True
