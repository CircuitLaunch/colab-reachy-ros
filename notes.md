### Requirements of MoveIt
[MoveItSimpleControllerManager](https://ros-planning.github.io/moveit_tutorials/doc/controller_configuration/controller_configuration_tutorial.html) expects the robot controller (provided by us) to provide [action servers](http://wiki.ros.org/actionlib) for [FollowJointTrajectory](http://docs.ros.org/en/api/control_msgs/html/action/FollowJointTrajectory.html) and optionally [GripperCommand](http://docs.ros.org/en/api/control_msgs/html/action/GripperCommand.html).

Additionally, present joint positions should be published on the `/joint_states` topic (I think?)

### All topics started by `roslaunch reachy_description display.launch`
/clicked_point
/initialpose
/joint_states
/move_base_simple/goal
/rosout
/rosout_agg
/tf
/tf_static


### All nodes started by `roslaunch reachy_moveit_config demo.launch`
/joint_state_publisher
/move_group
/robot_state_publisher
/rosout
/rviz


```
--------------------------------------------------------------------------------
Node [/joint_state_publisher]
Publications: 
 * /joint_states [sensor_msgs/JointState]
 * /rosout [rosgraph_msgs/Log]

Subscriptions: 
 * /move_group/fake_controller_joint_states [sensor_msgs/JointState]

Services: 
 * /joint_state_publisher/get_loggers
 * /joint_state_publisher/set_logger_level
```

```
--------------------------------------------------------------------------------
Node [/robot_state_publisher]
Publications: 
 * /rosout [rosgraph_msgs/Log]
 * /tf [tf2_msgs/TFMessage]
 * /tf_static [tf2_msgs/TFMessage]

Subscriptions: 
 * /joint_states [sensor_msgs/JointState]

Services: 
 * /robot_state_publisher/get_loggers
 * /robot_state_publisher/set_logger_level
```

```
--------------------------------------------------------------------------------
Node [/move_group]
Publications: 
 * /execute_trajectory/feedback [moveit_msgs/ExecuteTrajectoryActionFeedback]
 * /execute_trajectory/result [moveit_msgs/ExecuteTrajectoryActionResult]
 * /execute_trajectory/status [actionlib_msgs/GoalStatusArray]
 * /move_group/display_contacts [visualization_msgs/MarkerArray]
 * /move_group/display_cost_sources [visualization_msgs/MarkerArray]
 * /move_group/display_grasp_markers [visualization_msgs/MarkerArray]
 * /move_group/display_planned_path [moveit_msgs/DisplayTrajectory]
 * /move_group/fake_controller_joint_states [sensor_msgs/JointState]
 * /move_group/feedback [moveit_msgs/MoveGroupActionFeedback]
 * /move_group/monitored_planning_scene [moveit_msgs/PlanningScene]
 * /move_group/motion_plan_request [moveit_msgs/MotionPlanRequest]
 * /move_group/ompl/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
 * /move_group/ompl/parameter_updates [dynamic_reconfigure/Config]
 * /move_group/plan_execution/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
 * /move_group/plan_execution/parameter_updates [dynamic_reconfigure/Config]
 * /move_group/planning_scene_monitor/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
 * /move_group/planning_scene_monitor/parameter_updates [dynamic_reconfigure/Config]
 * /move_group/result [moveit_msgs/MoveGroupActionResult]
 * /move_group/sense_for_plan/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
 * /move_group/sense_for_plan/parameter_updates [dynamic_reconfigure/Config]
 * /move_group/status [actionlib_msgs/GoalStatusArray]
 * /move_group/trajectory_execution/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
 * /move_group/trajectory_execution/parameter_updates [dynamic_reconfigure/Config]
 * /pickup/feedback [moveit_msgs/PickupActionFeedback]
 * /pickup/result [moveit_msgs/PickupActionResult]
 * /pickup/status [actionlib_msgs/GoalStatusArray]
 * /place/feedback [moveit_msgs/PlaceActionFeedback]
 * /place/result [moveit_msgs/PlaceActionResult]
 * /place/status [actionlib_msgs/GoalStatusArray]
 * /rosout [rosgraph_msgs/Log]

Subscriptions: 
 * /attached_collision_object [moveit_msgs/AttachedCollisionObject]
 * /collision_object [unknown type]
 * /execute_trajectory/cancel [actionlib_msgs/GoalID]
 * /execute_trajectory/goal [moveit_msgs/ExecuteTrajectoryActionGoal]
 * /joint_states [sensor_msgs/JointState]
 * /move_group/cancel [actionlib_msgs/GoalID]
 * /move_group/goal [moveit_msgs/MoveGroupActionGoal]
 * /pickup/cancel [actionlib_msgs/GoalID]
 * /pickup/goal [moveit_msgs/PickupActionGoal]
 * /place/cancel [actionlib_msgs/GoalID]
 * /place/goal [moveit_msgs/PlaceActionGoal]
 * /planning_scene [moveit_msgs/PlanningScene]
 * /planning_scene_world [moveit_msgs/PlanningSceneWorld]
 * /tf [tf2_msgs/TFMessage]
 * /tf_static [tf2_msgs/TFMessage]
 * /trajectory_execution_event [std_msgs/String]

Services: 
 * /apply_planning_scene
 * /check_state_validity
 * /clear_octomap
 * /compute_cartesian_path
 * /compute_fk
 * /compute_ik
 * /get_planner_params
 * /get_planning_scene
 * /move_group/get_loggers
 * /move_group/load_map
 * /move_group/ompl/set_parameters
 * /move_group/plan_execution/set_parameters
 * /move_group/planning_scene_monitor/set_parameters
 * /move_group/save_map
 * /move_group/sense_for_plan/set_parameters
 * /move_group/set_logger_level
 * /move_group/trajectory_execution/set_parameters
 * /plan_kinematic_path
 * /query_planner_interface
 * /set_planner_params
```

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
/rviz/motionplanning_planning_scene_monitor/parameter_descriptions
/rviz/motionplanning_planning_scene_monitor/parameter_updates
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