#!/usr/bin/env python3

# creates a ros node that controls two servo motors
# on a arduino running ros serial
# this node accepts JointState which is
# ---------------
# would the datastructure look like
# jointStateObj.name = ["neck_yaw", "neck_tilt"]
# jointStateObj.position = [180,0]
# jointStateObj.velocity# unused
# jointStateObj.effort#unused
# ---------------
# by oran collins
# github.com/wisehackermonkey
# oranbusiness@gmail.com
# 20210307


import rospy
import math
from std_msgs.msg import UInt16
# from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

global yaw_servo
global tilt_servo
global sub
global yaw_servo_position
global tilt_servo_position

# standard delay between moving the joints
DELAY = 1.0
# setting up interger variables
#     the arduino only accepts integers
yaw_servo_position = UInt16()
tilt_servo_position = UInt16()

yaw_min = 0  # in degrees from x to y angles are accepted positions
yaw_max = 360
tilt_min = 75
tilt_max = 115


# helper function
# keeps the input number between a high and alow
def constrain(input: float, low: float, high: float) -> float:
    """
    input: radian float  an number to be constrained to a range low<-> high
    low: radian float minimum value the input can be
    high: radian float maximum value the input can be
    """
    return max(min(input, high), low)


def move_servos(msg: JointTrajectoryPoint) -> None:
    print(msg)
    print(msg.positions[0])
    print(msg.positions[1])

    yaw_degrees = constrain(math.degrees(msg.positions[0]), yaw_min, yaw_max)
    tilt_degrees = constrain(math.degrees(msg.positions[1]), tilt_min, tilt_max)

    print("Yaw: ", yaw_degrees, "tilt: ", tilt_degrees)
    # convert float angle radians -pi/2 to pi/2 to integer degrees 0-180
    yaw_servo_position.data = int(yaw_degrees)
    tilt_servo_position.data = int(tilt_degrees)

    # send an int angle to move the servo position to 0-180
    yaw_servo.publish(yaw_servo_position)
    tilt_servo.publish(tilt_servo_position)


# im only dulpicating this one betcause i cant think in radians and its a identical function to the one above
# exept i dont convert to degrees from radians
def move_servos_degrees_debug(msg: JointTrajectoryPoint) -> None:
    print(msg)
    print(msg.positions[0])
    print(msg.positions[1])

    yaw_degrees = constrain((msg.positions[0]), yaw_min, yaw_max)
    tilt_degrees = constrain((msg.positions[1]), tilt_min, tilt_max)

    print("Yaw: ", yaw_degrees, "tilt: ", tilt_degrees)
    # convert float angle radians -pi/2 to pi/2 to integer degrees 0-180
    yaw_servo_position.data = int(yaw_degrees)
    tilt_servo_position.data = int(tilt_degrees)

    # send an int angle to move the servo position to 0-180
    yaw_servo.publish(yaw_servo_position)
    tilt_servo.publish(tilt_servo_position)


# runs though all the JointTrajectoryPoint's inside a JointTrajectory
# which basically runs though an array of angles for each of the joints in this case
# a joint is a servo motor (and a diamixel motor for the antenna)
# and waits `DELAY` time before setting the motors to go to the next position
def process_positions(msg: JointTrajectory) -> None:
    print(msg)
    for joint_point in msg.points:
        print("Here")
        move_servos(joint_point)
        rospy.sleep(DELAY)


def process_positions_debug_degrees(msg: JointTrajectory) -> None:
    print(msg)
    for joint_point in msg.points:
        print("Here")
        move_servos_degrees_debug(joint_point)
        rospy.sleep(DELAY)


if __name__ == "__main__":
    rospy.init_node("position_animator_node")

    # setup topics to control into arduino servo angles
    # publishing a integer between angle 0-180 /servo1 or /servo2
    yaw_servo = rospy.Publisher("/head/neck_pan_goal", UInt16, queue_size=1)
    tilt_servo = rospy.Publisher("/head/neck_tilt_goal", UInt16, queue_size=1)

    # waiting for a JointState data on the topic "/move_head"

    # example data
    #  DOCS for joint state
    # http://wiki.ros.org/joint_state_publisher

    # Data format
    # http://docs.ros.org/en/api/sensor_msgs/html/msg/JointState.html

    # Header headerx
    # string[] name  <- optional
    # float64[] position
    # float64[] velocity
    # float64[] effort

    # rostopic pub /move_head "/{header:{}, name: ['servo1', 'servo2'], position: [0.5, 0.5], velocity:[], effort:[]}""
    sub = rospy.Subscriber("/head/position_animator", JointTrajectory, process_positions)
    sub = rospy.Subscriber("/head/position_animator_debug_degrees", JointTrajectory, process_positions_debug_degrees)

    sub = rospy.Subscriber("/head/position_animator/debug_point", JointTrajectoryPoint, move_servos)
    sub = rospy.Subscriber(
        "/head/position_animator/debug_point_degrees", JointTrajectoryPoint, move_servos_degrees_debug
    )

    rate = rospy.Rate(10)

    print("servo_mover: Running")
    while not rospy.is_shutdown():
        rate.sleep()
