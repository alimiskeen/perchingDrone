#! /usr/bin/env python3

import rospy
from std_msgs.msg import String
import time


def talker():
   print("Enter Commands for the arduino:")
   while True: 
      sayThis = str(input())
      # Have to add this in or else the program never exits
      if sayThis == "c":
         break
      pub = rospy.Publisher('arduinocommands', String, queue_size=10)
      rospy.init_node('testnodeforarduino', anonymous=True)
      rate = rospy.Rate(10)  # 10hz

      message2say = sayThis
      rospy.loginfo(message2say)
      pub.publish(message2say)
      rate.sleep()
      print("Ready for another command")
   rospy.spin()
      


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
