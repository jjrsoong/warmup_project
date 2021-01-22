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

        #LIDAR ranges. Bot is designed to follow a wall on its left side
        front = min(min(data.ranges[344:359]), min(data.ranges[0:14]))
        left = min(data.ranges[269:314])
        frontLeft = min(data.ranges[315:345])
        right = data.ranges[89]

        if front >= distance and frontLeft >= distance and left >= distance:
            instruction.linear.x = 0.3
            # Just entered the world, go straight ahead
            if self.first:
                print('just starting')
                instruction.angular.z = radians(0)
            # Right direction, but deviating from wall -- make an adjustment
            else:
                print('forward + adjust')
                instruction.angular.z = radians(-50)
        # Obstacle in front, need to stop moving forward and turn
        elif front < distance:
            self.first = False
            print('obs in front')
            instruction.linear.x = 0
            instruction.angular.z = radians(20)
        # No obstacle in front but too close to left hand side -- slow adjustment
        # using proportional control
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

    # -- BELOW HAS NEVER BEEN TESTED --
        # Theoretically should never occur. This means that there is a wall
        # at the front left **only**. As a failsafe, start turning so the wall
        # is directly left of the bot
        elif front >= distance and frontLeft < distance and left >= distance:
            self.first = False
            print('Houston, we have a problem')
            instruction.linear.x = 0
            instruction.angular.z = radians(10)

        # There is a wall on the right hand side
        if right < distance:
            print('Houston, there is an asteroid to the right')
            # There is no wall on the left; since the bot is optimized to
            # follow walls on its left, spin the bot around
            if left >= 500:
                instruction.angular.z = radians(-90)
            # Right hand side wall is closer than left hand side wall
            elif left >= distance:
                instruction.angular.z = radians(-20)
            # Walls on both sides are close by (ie in a hall)
            else:
                instruction.linear.x = 0.05
                # Right wall marginally closer than left
                if right < left:
                    instruction.angular.z = radians(-1)
                else:
                    instruction.angular.z = radians(1)

        print(front, frontLeft, left)
        self.navigator.publish(instruction)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = WallFollower()
    node.run()
