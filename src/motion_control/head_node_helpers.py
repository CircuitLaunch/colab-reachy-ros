# head node helpers 
# makes some of the tedius stuff with creating messages for the head node animation easer
# by oran collins
# github.com/wisehackermonkey
# oranbusiness@gmail.com
# 20210330

from typing import List
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
# for the array of poses 
# [trajectory_msgs/JointTrajectory Documentation](http://docs.ros.org/en/api/trajectory_msgs/html/msg/JointTrajectory.html)
# for the individual poses
# [trajectory_msgs/JointTrajectoryPoint Documentation](http://docs.ros.org/en/melodic/api/trajectory_msgs/html/msg/JointTrajectoryPoint.html)

def create_head_animation(animation_list: List)-> JointTrajectoryPoint:
    """
    giving a list of angles the head needs to move to in a list of touples
    ex: look to the left 
        [[94,90],[115,90],[115,90],[94,90]] returns a JointTrajectoryPoint 
    which looks something like this
        {points:[{positions:[94,90]},{positions:[115,90]},{positions:[115,90]},{positions:[94,90]}]}
    """
    full_animation = JointTrajectory()

    # create a JointTrajectoryPoint for every [115,90] 
    for animation_pose in animation_list:
        temp_animation_pose = JointTrajectoryPoint()
        # grab the degree angles for both the yaw, and the tilt
        # example [115,90]
        yaw, tilt = animation_pose
        # set the positions array to contain the yaw and title
        temp_animation_pose.positions = [yaw,tilt]

        # result is a JointTrajectory with the points array lookin like 
        # something like this
        #  {points:[{positions:[94,90]},{positions:[115,90]},{positions:[115,90]},{positions:[94,90]}]}

        full_animation.points.append(temp_animation_pose)
    
    return full_animation