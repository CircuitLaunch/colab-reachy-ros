import rospy
import sys
import moveit_commander
from motion_control.moveit_helpers import load_joint_configurations_from_file


if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("moveit_commander_node")

    group_name = rospy.get_param("~move_group")
    pose_name = rospy.get_param("~pose_name")

    move_group = moveit_commander.MoveGroupCommander(group_name)
    load_joint_configurations_from_file(move_group)

    move_group.set_named_target(pose_name)
    print("Moving...")
    move_group.go(wait=True)
    move_group.stop()

    print("Movement done")
