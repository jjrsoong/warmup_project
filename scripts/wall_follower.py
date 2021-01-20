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

    def process_scan(self, data):
        lin = Vector3()
        ang = Vector3()
        instruction = Twist(linear=lin,angular=ang)
        if data.ranges[0] >= distance:
            instruction.linear.x = 0.2
            #instruction.angular.z = radians(0)
        # Near a wall, need to turn somehow
        else:
            #instruction.linear.x = 0
            instruction.angular.z = radians(30)

        self.navigator.publish(instruction)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = WallFollower()
    node.run()
