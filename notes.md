### All topics started by `roslaunch reachy_description display.launch`
/clicked_point
/initialpose
/joint_states
/move_base_simple/goal
/rosout
/rosout_agg
/tf
/tf_static

### All topics started by `roslaunch reachy_moveit_config demo.launch`

/attached_collision_object
/collision_object
/execute_trajectory/cancel
/execute_trajectory/feedback
/execute_trajectory/goal
/execute_trajectory/result
/execute_trajectory/status
/joint_states
/move_group/cancel
/move_group/display_contacts
/move_group/display_cost_sources
/move_group/display_grasp_markers
/move_group/display_planned_path
/move_group/fake_controller_joint_states
/move_group/feedback
/move_group/goal
/move_group/monitored_planning_scene
/move_group/motion_plan_request
/move_group/ompl/parameter_descriptions
/move_group/ompl/parameter_updates
/move_group/plan_execution/parameter_descriptions
/move_group/plan_execution/parameter_updates
/move_group/planning_scene_monitor/parameter_descriptions
/move_group/planning_scene_monitor/parameter_updates
/move_group/result
/move_group/sense_for_plan/parameter_descriptions
/move_group/sense_for_plan/parameter_updates
/move_group/status
/move_group/trajectory_execution/parameter_descriptions
/move_group/trajectory_execution/parameter_updates
/pickup/cancel
/pickup/feedback
/pickup/goal
/pickup/result
/pickup/status
/place/cancel
/place/feedback
/place/goal
/place/result
/place/status
/planning_scene
/planning_scene_world
/recognized_object_array
/rosout
/rosout_agg
/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/feedback
/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/update
/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/update_full
/rviz_parallels_Parallels_Virtual_Platform_46663_5858044746647360037/motionplanning_planning_scene_monitor/parameter_descriptions
/rviz_parallels_Parallels_Virtual_Platform_46663_5858044746647360037/motionplanning_planning_scene_monitor/parameter_updates
/tf
/tf_static
/trajectory_execution_event

`[move_group/fake_controller_joint_states]` is given as the parameter `source_list` ot the `joint_state_publisher`. The topic `/move_group/fake_controller_joint_states` is of type `sensor_msgs/JointState`, published by `/move_group` and subscribed to by `/joint_state_publisher`.

A captured message looked like this:

```
header: 
  seq: 0
  stamp: 
    secs: 1613003801
    nsecs: 678501497
  frame_id: ''
name: 
  - l_arm_yaw
  - l_elbow_pitch
  - l_forearm_yaw
  - l_shoulder_pitch
  - l_shoulder_roll
  - l_wrist_pitch
  - l_wrist_roll
  - orbita_pitch
  - orbita_roll
  - orbita_yaw
  - r_arm_yaw
  - r_elbow_pitch
  - r_forearm_yaw
  - r_shoulder_pitch
  - r_shoulder_roll
  - r_wrist_pitch
  - r_wrist_roll
position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
velocity: []
effort: []
---
```