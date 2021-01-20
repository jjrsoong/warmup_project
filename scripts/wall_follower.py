#!/usr/bin/env python3

import rospy

#msg needed for /scan
from sensor_msgs.msg import LaserScan

# msgs needed for /cmd cmd_vel
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from math import radians

# How close we will get to wall.
distance = 0.4

class WallFollower(object):

    def __init__(self):
        rospy.init_node("follow_wall")
        rospy.Subscriber("/scan", LaserScan, self.process_scan)
        self.navigator = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        lin = Vector3()
        ang = Vector3()
        self.instruction = Twist(linear=lin,angular=ang)

        #Boolean variable, set to True is in the middle of a turn
        self.turning = False

    def process_scan(self, data):
        if data.ranges[0] >= distance and not self.turning:
            self.instruction.linear.x = 0.5
            self.instruction.angular.z = radians(0)
        #elif data.ranges[0] >= distance and data.ranges[45] >= distance:
        #    self.instruction.linear.x = 0
        #    self.instruction.angular.z = radians(-30)
        #elif data.ranges[0] >= distance and data.ranges[315] >= distance:
        #    self.instruction.linear.x = 0
        #    self.instruction.angular.z = radians(330)

        # Near a wall, need to turn somehow
        else:
            # Has turned enough, can resume forward motion
            if data.ranges[0] >= 1:
                self.turning = False
            else:
                self.turning = True

            self.instruction.linear.x = 0
            self.instruction.angular.z = radians(30)

        self.navigator.publish(self.instruction)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = WallFollower()
    node.run()
