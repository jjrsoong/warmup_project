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

        #Boolean variable, set to True if bot has never turned before
        # (ie. it just spawned into the world and has not encountered a wall)
        self.first = True

    def process_scan(self, data):
        lin = Vector3()
        ang = Vector3()
        instruction = Twist(linear=lin,angular=ang)

        #LIDAR ranges. Bot is designed to follow a wall on its right side
        front = min(min(data.ranges[344:359]), min(data.ranges[0:14]))
        right = min(data.ranges[269:314])
        frontRight = min(data.ranges[315:345])
        left = data.ranges[89]

        if front >= distance and frontRight >= distance and right >= distance:
            instruction.linear.x = 0.3
            # Just entered the world, go straight ahead
            if self.first:
                instruction.angular.z = radians(0)
            # Right direction, but deviating from wall -- make an adjustment
            else:
                instruction.angular.z = radians(-50)
        # Obstacle in front, need to stop moving forward and turn
        elif front < distance:
            self.first = False
            instruction.linear.x = 0
            instruction.angular.z = radians(20)
        # No obstacle in front but too close to right hand side -- slow adjustment
        # using proportional control
        elif front >= distance and frontRight >= distance and right < distance:
            self.first = False
            error_rate = (distance - frontRight) * 125
            prop_control = 1
            instruction.angular.z = radians(error_rate * prop_control)

            # If the distance is roughly on target (implied by low angular z),
            # then can begin to move faster. Otherwise, move slowly.
            if (error_rate * prop_control) < 5:
                instruction.linear.x = 0.3
            else:
                instruction.linear.x = 0.1

        # Obstacle in front right -- slow adjustment
        elif front >= distance and frontRight < distance and right < distance:
            self.first = False
            instruction.linear.x = 0.2
            instruction.angular.z = radians(10)

        # This means that there is a wall
        # at the front right **only**. As a failsafe, start turning so the wall
        # is directly right of the bot
        elif front >= distance and frontRight < distance and right >= distance:
            self.first = False
            print('Houston, we have a problem')
            instruction.linear.x = 0
            instruction.angular.z = radians(10)

        # There is a wall on the left hand side
        if left < distance:
            print('Houston, there is an asteroid to the left')
            # There is no wall on the right; since the bot is optimized to
            # follow walls on its right, spin the bot around
            if right >= 500:
                instruction.angular.z = radians(-90)
            # Right hand side wall is closer than right hand side wall
            elif right >= distance:
                instruction.angular.z = radians(-20)
            # Walls on both sides are close by (ie in a hall)
            else:
                instruction.linear.x = 0.05
                # Right wall marginally closer than right
                if left < right:
                    instruction.angular.z = radians(-1)
                else:
                    instruction.angular.z = radians(1)

        self.navigator.publish(instruction)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = WallFollower()
    node.run()
