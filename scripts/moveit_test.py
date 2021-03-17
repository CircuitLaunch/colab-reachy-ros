import rospy
import sys
import moveit_commander
from motion_control.moveit_helpers import load_joint_configurations_from_file

TARGET_POSE = "pose_1"

if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("moveit_commander_node")
    move_group = moveit_commander.MoveGroupCommander("left_arm")
    load_joint_configurations_from_file(move_group)

    print(f"Available targets are {move_group.get_remembered_joint_values()}")
    print(f"Goal target is {move_group.get_named_target_values(TARGET_POSE)}")

    move_group.set_named_target(TARGET_POSE)
    print("Moving...")
    move_group.go(wait=True)
    move_group.stop()

    print("Movement done")