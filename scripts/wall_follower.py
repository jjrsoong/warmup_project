#!/usr/bin/env python3

import rospy

#msg needed for /scan
from sensor_msgs.msg import LaserScan

# msgs needed for /cmd cmd_vel
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from math import radians

# How close we will get to wall.
distance = 0.5

class WallFollower(object):

    def __init__(self):
        rospy.init_node("follow_wall")
        rospy.Subscriber("/scan", LaserScan, self.process_scan)
        self.navigator = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        #Boolean variable, set to True is in the middle of a turn
        self.turning = False

        #Boolean variable, set to True is bot has never turned before
        self.first = True

    def process_scan(self, data):
        # error = data.ranges[269] - distance
        # prop_control = 1

        lin = Vector3()
        ang = Vector3()
        instruction = Twist(linear=lin,angular=ang)

        front = min(min(data.ranges[344:359]), min(data.ranges[0:14]))
        left = min(data.ranges[269:314])
        frontLeft = min(data.ranges[315:345])

        if front >= distance and frontLeft >= distance and left >= distance:# and self.first:
            instruction.linear.x = 0.3
            # Just starting, got straight
            if self.first:
                print('just starting')
                instruction.angular.z = radians(0)
            # Right direction, but deviating from wall -- make adjustment
            else:
                print('forward + adjust')
                instruction.angular.z = radians(-50)
        elif front < distance:
            self.first = False
            print('obs in front')
            instruction.linear.x = 0
            instruction.angular.z = radians(20)
        # elif front < distance and frontLeft < distance and left >= distance:
        #     instruction.linear.x = 0.1
        #     instruction.angular.z = radians(15)
        # No obstacle in front but too close to left hand side -- slow adjustment
        elif front >= distance and frontLeft >= distance and left < distance:
            self.first = False
            print('slow turn 1 ')
            error_rate = (distance - frontLeft) * 125
            prop_control = 1
            instruction.angular.z = radians(error_rate * prop_control)

            # If the distance is roughly on target (implied by low angular z),
            # then can begin to move faster. Otherwise, move slowly.
            if (error_rate * prop_control) < 5:
                instruction.linear.x = 0.3
            else:
                instruction.linear.x = 0.1

        # Obstacle in front left -- slow adjustment
        elif front >= distance and frontLeft < distance and left < distance:
            self.first = False
            print('slow turn 2')
            instruction.linear.x = 0.2
            instruction.angular.z = radians(10)
        # Major obstacle -- stop forward motion, only turn
        # elif front < distance and frontLeft < distance and left < distance:
        #     self.first = False
        #     print('fast turn')
        #     instruction.linear.x = 0
        #     instruction.angular.z = radians(30)


            # if frontLeft < distance:
            #     if left < distance:
            #         print('fast turn')
            #         instruction.linear.x = 0
            #         instruction.angular.z = radians(30)

        # elif left

        # elif data.ranges[0] >= distance and not self.turning:
        #     print('out of first turn')
        #     instruction.linear.x = 0.3
        #     #self.instruction.angular.z = radians(0)
        #
        #     # moving forward but still getting closer to the wall
        #     #if data.ranges[269] < distance:
        #     instruction.angular.z = error * prop_control
        #     print(error)
        # #elif data.ranges[0] >= distance and data.ranges[45] >= distance:
        # #    self.instruction.linear.x = 0
        # #    self.instruction.angular.z = radians(-30)
        # #elif data.ranges[0] >= distance and data.ranges[315] >= distance:
        # #    self.instruction.linear.x = 0
        # #    self.instruction.angular.z = radians(330)
        #
        # # Near a wall, need to turn somehow
        # else:
        #     self.first = False
        #     self.turning = True
        #     instruction.linear.x = 0
        #     instruction.angular.z = error * prop_control
        #     print(error)
        #
        #     if data.ranges[269] < distance and data.ranges[0] > distance:
        #         self.turning = False
        #         print('stop turning')


        print(front, frontLeft, left)
        self.navigator.publish(instruction)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = WallFollower()
    node.run()
