#! /usr/bin/env python3
import rospy
from std_msgs.msg import String
import serial
import serial.tools.list_ports_linux
import time

# Ali you can just change this make this node listen to a different one
# that way you don't need to worry about finding it 
nodeThisOneNeedsToListenTo = "arduinocommands"


def callback(data):
   # put this in the log 
   rospy.loginfo("sendardmess heard " + data.data)

   # Encode the data to a byte format 
   ardmessage = bytes(data.data, 'utf-8')

   # Write the data to the arduino 
   sercomm.write(ardmessage)

   # Get the response from the arduino 
   messRecvd = sercomm.read_until('\n')

   # Put this in the log 
   rospy.loginfo("The arduino heard " + messRecvd.decode('utf_8'))


def listener(sercomm):
   # Initialize the node 
   rospy.init_node('listenforarduinocommands', anonymous=True)

   # Set who this node subscribes to 
   rospy.Subscriber(nodeThisOneNeedsToListenTo, String, callback)
   
   # spin() simply keeps python from exiting until this node is stopped
   rospy.spin()

def setupserialconnection(): 
   # List the available ports
   ports = serial.tools.list_ports_linux.comports()

   #TODO figure out how to handle multiple arduinos 
   # Figure out which usb device is an arduino
   for possdevice in ports:
      if "arduino" in possdevice.manufacturer :
         portName = possdevice.device

   # Create the serial connection 
   ardConn = serial.Serial(portName, 9600, timeout=1)

   # This gives the arduino time to restart after the serial connection is created
   time.sleep(1)
   return ardConn


if __name__ == '__main__':
   sercomm = setupserialconnection()
   listener(sercomm)