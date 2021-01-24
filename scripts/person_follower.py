#!/usr/bin/env python3

import rospy

#msg needed for /scan
from sensor_msgs.msg import LaserScan

# msgs needed for /cmd cmd_vel
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from math import radians

# Begin slowing down if bot is within this distance
decel_point = 1
# Follow person at this distance
stop_point = 0.5

class PersonFollower(object):

    def __init__(self):
        rospy.init_node("follow_wall")
        rospy.Subscriber("/scan", LaserScan, self.process_scan)
        self.navigator = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

    def process_scan(self, data):
        inf = float('inf')

        lin = Vector3()
        ang = Vector3()
        instruction = Twist(linear=lin,angular=ang)

        # Split 360 degree LIDAR scan into 9 distinct areas
        front = min(min(data.ranges[344:359]), min(data.ranges[0:14]))
        frontLeft = min(data.ranges[15:44])
        frontCenterLeft = min(data.ranges[45:89])
        backCenterLeft = min(data.ranges[90:134])
        backLeft = min(data.ranges[135:179])
        backRight = min(data.ranges[180:224])
        backCenterRight = min(data.ranges[225:269])
        frontCenterRight = min(data.ranges[270:314])
        frontRight = min(data.ranges[315:345])

        # The below if statements dictate how the bot should turn (both direction
        # and magnitude) when the person is in each LIDAR zone. Using all if statements
        # so that the zones toward the front -- which correspond to later if statements
        # -- will override turning directions from zones behind the bot
        # All things being equal, the bot will prefer to turn to its right
        # to reach the person. (ie. if person is directly behind, will turn right)
        if backLeft != inf:
            instruction.angular.z = radians(60)
        if backRight != inf:
            instruction.angular.z = radians(-60)

        if backCenterLeft != inf:
            instruction.angular.z = radians(50)
        if backCenterRight != inf:
            instruction.angular.z = radians(-50)

        if frontCenterLeft != inf:
            instruction.angular.z = radians(30)
        if frontCenterRight != inf:
            instruction.angular.z = radians(-30)

        if frontLeft != inf:
            instruction.angular.z = radians(20)
        if frontRight != inf:
            instruction.angular.z = radians(-20)

        # If person is somewhere in front of robot, can move forward
        if front != inf:
            error_rate = front - stop_point
            prop_control = 1

            # Goal condition: if here, bot should stop moving forward or turning
            if front <= stop_point * 1.05 and front >= stop_point * 0.95:
                instruction.linear.x = 0
                # Person is directly in front and the right distance from the bot
                # --> stop moving linearly or angularly
                if min(frontCenterLeft, backCenterLeft, backLeft,
                frontCenterRight, backCenterRight, backRight) == inf:
                    instruction.angular.z = 0
            # Right the behind person, just back up without turning
            elif front <= stop_point:
                instruction.linear.x = error_rate * prop_control
            # Still quite far from person, can move quickly
            elif front > decel_point:
                instruction.linear.x = 1
            # Pretty close to person, move slowly
            else:
                instruction.linear.x = 0.2

        self.navigator.publish(instruction)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = PersonFollower()
    node.run()
