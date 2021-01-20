#!/usr/bin/env python3

import rospy
import time

#needed for cmd/vel
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from math import radians

class SquareDriver(object):

    def __init__(self):
        rospy.init_node("drive_in_square")
        self.leader = rospy.Publisher("/cmd_vel", Twist, queue_size=10)


        forward = Twist()
        forward.linear.x = 0.2

        turn = Twist()
        turn.angular.z = radians(30)

        # Keep going in a square
        while True:
            start_time = time.time()
            # Move forward along one side of square
            while True:
                time_elapsed = time.time() - start_time
                if time_elapsed >= 5.05:
                    break
                else:
                    self.leader.publish(forward)
            # Turn robot 90 degrees
            while True:
                time_elapsed = time.time() - start_time-5
                if time_elapsed >= 3.11:
                    break
                else:
                    self.leader.publish(turn)
        #for x in range(0, 4):
       # for x in range(0, 20):
        #    self.leader.publish(forward)
        #for x in range(0, 10):
         #   self.leader.publish(turn)

    def run(self):
        rospy.spin()



if __name__ == '__main__':
    node = SquareDriver()
    node.run()
