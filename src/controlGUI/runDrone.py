import time

from drone import Drone
from ps4controller import DroneController
from TerminalControl import TerminalControl

import serial
import serial.tools.list_ports_linux
import time
from signal import signal, SIGINT
from sys import exit


def setupserialconnection():
    # List the available ports
    ports = serial.tools.list_ports_linux.comports()

    # Figure out which usb device is an arduino
    arduinoFound = False
    for possdevice in ports:
        try:
            # Try to connect to the serial device
            connection = serial.Serial(possdevice.device, 9600, timeout=1)
            # Wait for arduino to reboot after connection and to send it's identifier message
            time.sleep(3.0)
            # Attemp to connect to read the arduinos message
            tempMess = connection.read_all()
            tempMess = tempMess.decode('utf-8')
            if tempMess == 'gripper':
                arduinoFound = True
                print('Connected to arduino successfully')
                return connection
        except:
            print("Unsuccessful connection device may not be the arduino")
    if arduinoFound == False:
        print("No arduino found in the com ports")
        return None


def send_to_arduino(order: str, connection: serial.Serial):
    message = bytes(order, 'utf-8')
    connection.write(message)


def on_close():
    ard.close()
    exit(0)


if __name__ == '__main__':
    signal(SIGINT, on_close)

    print('connecting to the drone')
    drn = Drone("/dev/ttyACM0")
    print('connected to the drone')

    ard = setupserialconnection()

    # if ard is None:
    #     exit(-1)
    #
    # send_to_arduino('C104', ard)
    #
    # time.sleep(5)
    #
    # send_to_arduino('C103', ard)
    #
    # ard.close()

    ctrl = TerminalControl(drn, ard)
    print('listening to the terminal commands')
    ctrl.listen()
