
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>

#include <moveit_msgs/RobotTrajectory.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/GetMotionPlan.h>
#include <moveit_msgs/PlanningScene.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_state/conversions.h>

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>


#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

//#include <tf2_ros/transform_listener.h>
//#include "tf2/transform_datatypes.h"
//#include <tf2/impl/convert.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <trajectory_msgs/JointTrajectory.h>

/*#include <apc16delft_msgs/Error.h>
#include <apc16delft_msgs/ExecuteCoarseMotion.h>
#include <apc16delft_msgs/GetCoarseMotion.h>
#include <apc16delft_msgs/ExecuteFineMotion.h>
#include <apc16delft_msgs/ExecuteStitchedMotion.h>
#include <apc16delft_msgs/ExecuteCalibrationMotion.h>*/
#include <lenny_msgs/MoveToPredefinedPose.h>
#include <lenny_msgs/CreatePickMovements.h>
#include <lenny_msgs/ExecuteCoarseMotion.h>
#include <lenny_msgs/PlanCoarseMotion.h>
#include <lenny_msgs/PlanExecuteFineMotion.h>
#include <lenny_msgs/ExtendTCP.h>
#include <lenny_msgs/RestoreTCP.h>


/*
#include <apc16delft_msgs/MoveToCalibrateShelf.h>
#include <apc16delft_msgs/MoveToCalibrateTote.h>
#include <std_msgs/Int32.h>*/

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <Eigen/Geometry>

#include <iostream>
#include <string>

#include "motion_utilities.h"


using namespace tf;



class MotionExecutor {
public:
	/// Construct a motion executor node.
	MotionExecutor();
	

protected:
	ros::AsyncSpinner spinner;

private:

	ros::NodeHandle node_handle_;
  
  
	/// SERVICES advertise by the class
	ros::ServiceServer move_to_predefined_pose_;
	ros::ServiceServer create_pick_movements_;
	ros::ServiceServer execute_coarse_motion_;
	ros::ServiceServer plan_coarse_motion_;
  ros::ServiceServer execute_fine_motion_;
	ros::ServiceServer plan_execute_fine_motion_;
  ros::ServiceServer extendTCP_; 
  ros::ServiceServer restoreTCP_; 
  
  
	/// MOVE GROUP VARIABLES
  moveit::planning_interface::MoveGroupInterface arms_group_;
	moveit::planning_interface::MoveGroupInterface arm_right_group_;
	moveit::planning_interface::MoveGroupInterface arm_left_group_;
	moveit::planning_interface::MoveGroupInterface torso_group_;
	moveit::planning_interface::MoveGroupInterface sda10f_group_;
	
	moveit::planning_interface::MoveGroupInterface * current_moveit_group_;
	moveit::planning_interface::MoveGroupInterface::Plan plan_;
	
  /// MOTION PARAMETERS
  
  double approach_distance_; ///DeltaZ position "Pre-grasp" to move to before reaching the object
  double retreat_distance_;  ///DeltaZ position "Post-grasp" to move to after reaching the object
  double trajectory_velocity_scale_; ///Velocity scale when working with the real robot
  std::string planner_id_; ///Name of the planner to be used. Name according to the OMLP library
	double planning_time_;
  double planning_attemps_; ///This is used if the variable replaning = True
  double position_tolerance_; 
  double orientation_tolerance_; 
  double goal_tolerance_; 
  std::string world_frame_id_;
  
  ///END EFFECTOR LINKS
  std::string tcp_link_name_arm_left_;
  std::string tcp_link_name_arm_right_;
  std::string tcp_link_name_;
  
  std::string wrist_link_name_arm_right_;
  std::string wrist_link_name_arm_left_;
  std::string wrist_link_name_;
  
  ///END EFFECTOR NAMES
  ///With these names we can know which gripper is in which arm
  std::string ee_arm_left_;
  std::string ee_arm_right_;
  
  
  ///Transformation variables between TCP and Wrist
  ///If another ee is used e.g. the tool, this transformation will change, 
  ///otherwise they are constant.
  tf::StampedTransform tcp_to_wrist_tf_right_;
  tf::StampedTransform tcp_to_wrist_tf_;  
  tf::StampedTransform tcp_to_wrist_tf_left_; 
  
  
  
  ///TODO: Check if these variables are really used
	robot_state::RobotStatePtr rs_;
  MotionUtilities motion_utilities_;
	geometry_msgs::Pose object_pose_;
 
 
  const moveit::core::JointModelGroup* joint_model_group_; ///Used to request current position
  
  
  moveit::core::RobotStatePtr kinematic_state_; ///Used to request current position ???
  
	 
  ros::ServiceClient motion_plan_client; ///To request a plan
  
  
  ///Function to check if the robot can reach the requested pose.
  ///TODO: check if it really works, becasue sometimes it says false but the robot
  ///can reach that position...
  bool checkWayPointReachability(const geometry_msgs::Pose& waypoint);
  
  
  

protected:
	
   
	/// Functions of the SERVICES
  ///This service is used to move to poses predefined in the srdf, like HOME, PICK_HOME etc.
  bool moveToPredefinedPose(lenny_msgs::MoveToPredefinedPose::Request & req, lenny_msgs::MoveToPredefinedPose::Response & res);
	
  ///This service is used to create 3 poses, the approach, pick, and retreat poses, relative to the pose of the object to pick.
  ///retreat and approach poses are the same as object's pose but with a +/-deltaZ, this increments are defined in the yaml file
  ///Input: pose of the object (wrt world), move_group
  ///Output: a vector of poses with 3 poses Approach,Pick,Retreat. These poses are wrt the wrist link (in case of Lenny, link7) 
  bool createPickMovements(lenny_msgs::CreatePickMovements::Request & req, lenny_msgs::CreatePickMovements::Response & res);
	
  ///This service is used to momve to poses predefined in the srdf, like HOME, PICK_HOME etc.
  bool planCoarseMotion(lenny_msgs::PlanCoarseMotion::Request & req, lenny_msgs::PlanCoarseMotion::Response & res);
	bool executeCoarseMotion(lenny_msgs::ExecuteCoarseMotion::Request & req, lenny_msgs::ExecuteCoarseMotion::Response & res);	
	bool planExecuteFineMotion(lenny_msgs::PlanExecuteFineMotion::Request & req, lenny_msgs::PlanExecuteFineMotion::Response & res);
	bool extendTCP(lenny_msgs::ExtendTCP::Request & req, lenny_msgs::ExtendTCP::Response & res); 
  bool restoreTCP(lenny_msgs::RestoreTCP::Request & req, lenny_msgs::RestoreTCP::Response & res); 
  
  ///This function sets motion parameters of a Moveit group
  ///Input: move group
  ///Output: true
  bool setMoveitGroupParams(moveit::planning_interface::MoveGroupInterface * moveit_group);
  
  ///This function loads config parametrs (yaml file) that define the task. The parameters are related to 
  ///the links name of the robot, grippers, and motion parameters such as the planner, number of attepts, etc.
  ///Input: move group
  ///Output: void
  void load_config_params();
  
  
};


