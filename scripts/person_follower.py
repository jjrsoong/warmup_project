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

        front = min(min(data.ranges[344:359]), min(data.ranges[0:14]))
        frontLeft = min(data.ranges[15:44])
        frontCenterLeft = min(data.ranges[45:89])
        backCenterLeft = min(data.ranges[90:134])
        backLeft = min(data.ranges[135:179])
        backRight = min(data.ranges[180:224])
        backCenterRight = min(data.ranges[225:269])
        frontCenterRight = min(data.ranges[270:314])
        frontRight = min(data.ranges[315:345])

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

        # if front < frontLeft:
        #     instruction.angular.z = radians(5)
        #     print("front<others 1")
        # if front < frontRight:
        #     instruction.angular.z = radians(-5)
        #     print("front<others 2")



        # If person is somewhere in front of robot, can move forward
        if front != inf:
            error_rate = front - stop_point
            prop_control = 1

            # if front > decel_point:
            #     instruction.linear.x = error_rate * prop_control

            # elif front < stop_point:
            #     instruction.linear.x = -0.2
            # Goal condition: should stop moving forward or turning
            if front <= stop_point * 1.05 and front >= stop_point * 0.95:
                instruction.linear.x = 0
                if min(frontCenterLeft, backCenterLeft, backLeft,
                frontCenterRight, backCenterRight, backRight) == inf:
                    print("finished")
                    instruction.angular.z = 0
            # Right behind person, just back up without turning
            elif front <= stop_point: # and (min(frontCenterLeft, backCenterLeft, backLeft,
            #backRight, backCenterRight, frontCenterRight) == inf):
                print("only backup")
                #instruction.angular.z = 0
                instruction.linear.x = error_rate * prop_control
            elif front > decel_point:
                instruction.linear.x = 1
            else:
                instruction.linear.x = 0.4

        if instruction.angular.z > 0:
            print("turning left" + str(instruction.angular.z))
        elif instruction.angular.z < 0:
            print("turning right" + str(instruction.angular.z))
        if instruction.linear.x > 0:
            print("forward" + str(instruction.linear.x))
        elif instruction.linear.x < 0:
            print("back: " + str(instruction.linear.x))

        print(frontLeft, front, frontRight)
        self.navigator.publish(instruction)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = PersonFollower()
    node.run()
