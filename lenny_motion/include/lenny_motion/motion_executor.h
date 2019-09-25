
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

#include "motion_utilities.h"

///TODO: add reading the yaml with information about the move group

using namespace tf;

namespace pirLenny {

class MotionExecutor {
public:
	/// Construct a motion executor node.
	MotionExecutor();
	

protected:
	ros::AsyncSpinner spinner;

private:
	ros::NodeHandle node_handle_;
	
	std::string yaml_path_;
	int calibration_trajectory_length_;
	double trajectory_velocity_scaling_;
	double calibration_trajectory_tolerance_;

	/*ros::ServiceServer execute_coarse_motion_;
	ros::ServiceServer get_coarse_motion_;
	ros::ServiceServer execute_fine_motion_;
	ros::ServiceServer execute_stitched_motion_;
	ros::ServiceServer execute_calibration_motion_;*/
	ros::ServiceServer move_to_predefined_pose_;
	ros::ServiceServer create_pick_movements_;
	ros::ServiceServer execute_coarse_motion_;
	ros::ServiceServer plan_coarse_motion_;
  ros::ServiceServer execute_fine_motion_;
	ros::ServiceServer plan_execute_fine_motion_;
  ros::ServiceServer extendTCP_; 
  ros::ServiceServer restoreTCP_; 
  
  
	/*
	ros::ServiceServer move_to_calibrate_shelf_;
	ros::ServiceServer move_to_calibrate_tote_;
	ros::Publisher grasp_pose_visualizer_;
*/

    moveit::planning_interface::MoveGroupInterface arms_group_;
	moveit::planning_interface::MoveGroupInterface arm_right_group_;
	moveit::planning_interface::MoveGroupInterface arm_left_group_;
	moveit::planning_interface::MoveGroupInterface torso_group_;
	moveit::planning_interface::MoveGroupInterface sda10f_group_;
	
	moveit::planning_interface::MoveGroupInterface * current_moveit_group_;
	moveit::planning_interface::MoveGroupInterface::Plan plan_;
	
	robot_state::RobotStatePtr rs_;
  const moveit::core::JointModelGroup* joint_model_group_;
  
  
  moveit::core::RobotStatePtr kinematic_state_;
  
	MotionUtilities motion_utilities_;
  
	geometry_msgs::Pose object_pose_;
 // std::vector<geometry_msgs::Pose> pick_move_poses_;
  
  ros::ServiceClient motion_plan_client;
  
  
  bool checkWayPointReachability(const geometry_msgs::Pose& waypoint);
  tf::StampedTransform tcp_to_wrist_tf_right_;
   tf::StampedTransform tcp_to_wrist_tf_;  
  
  tf::StampedTransform tcp_to_wrist_tf_left_; 

protected:
	/*bool executeCoarseMotion(apc16delft_msgs::ExecuteCoarseMotion::Request & req, apc16delft_msgs::ExecuteCoarseMotion::Response & res);
	bool getCoarseMotion(apc16delft_msgs::GetCoarseMotion::Request & req, apc16delft_msgs::GetCoarseMotion::Response & res);
	bool executeStitchedMotion(apc16delft_msgs::ExecuteStitchedMotion::Request & req, apc16delft_msgs::ExecuteStitchedMotion::Response & res);
	bool executeFineMotion(apc16delft_msgs::ExecuteFineMotion::Request & req, apc16delft_msgs::ExecuteFineMotion::Response & res);
	bool executeCalibrationMotion(apc16delft_msgs::ExecuteCalibrationMotion::Request & req, apc16delft_msgs::ExecuteCalibrationMotion::Response & res);*/
	bool moveToPredefinedPose(lenny_msgs::MoveToPredefinedPose::Request & req, lenny_msgs::MoveToPredefinedPose::Response & res);
	
	bool createPickMovements(lenny_msgs::CreatePickMovements::Request & req, lenny_msgs::CreatePickMovements::Response & res);

	bool planCoarseMotion(lenny_msgs::PlanCoarseMotion::Request & req, lenny_msgs::PlanCoarseMotion::Response & res);
	
	bool executeCoarseMotion(lenny_msgs::ExecuteCoarseMotion::Request & req, lenny_msgs::ExecuteCoarseMotion::Response & res);
	
  //bool planFineMotion(lenny_msgs::PlanFineMotion::Request & req, lenny_msgs::PlanFineMotion::Response & res);
	
	bool planExecuteFineMotion(lenny_msgs::PlanExecuteFineMotion::Request & req, lenny_msgs::PlanExecuteFineMotion::Response & res);

	bool extendTCP(lenny_msgs::ExtendTCP::Request & req, lenny_msgs::ExtendTCP::Response & res); 
  bool restoreTCP(lenny_msgs::RestoreTCP::Request & req, lenny_msgs::RestoreTCP::Response & res); 

	/*
	bool moveToCalibrateShelf(apc16delft_msgs::MoveToCalibrateShelf::Request &, apc16delft_msgs::MoveToCalibrateShelf::Response & res);
	bool moveToCalibrateTote(apc16delft_msgs::MoveToCalibrateTote::Request &, apc16delft_msgs::MoveToCalibrateTote::Response & res);
	bool checkCalibrationTrajectorySanity(trajectory_msgs::JointTrajectory & motion_trajectory, moveit::planning_interface::MoveGroup* current_group);*/
};

}
