#include "motion_executor.h"
#include <trajectory_msgs/JointTrajectory.h>

#include <string>
#include <cmath>
#include <limits>
#include <vector>

namespace pirLenny {


MotionExecutor::MotionExecutor() :
	spinner(1),
	node_handle_("~"),
	arms_group_("arms"),
	arm_right_group_("arm_right"),
	arm_left_group_("arm_left"),
	torso_group_("torso"),
	sda10f_group_("sda10f"),
	current_group_(&arm_right_group_),
	trajectory_velocity_scaling_(0.1)
{
	spinner.start();
	//std::string default_path = "path_to_yaml_directory";
	//node_handle_.param<std::string>("yaml_path", yaml_path_, default_path);
	//calibration_trajectory_length_   = dr::getParam<int>(node_handle_, "calibration_trajectory_length", 50);
	//trajectory_velocity_scaling_     = dr::getParam<double>(node_handle_, "trajectory_velocity_scaling", 0.05);
	//calibration_trajectory_tolerance_= dr::getParam<double>(node_handle_, "calibration_trajectory_tolerance", 0.5);

	/*execute_coarse_motion_      = node_handle_.advertiseService("execute_coarse_motion", &MotionExecutor::executeCoarseMotion, this);
	get_coarse_motion_          = node_handle_.advertiseService("get_coarse_motion", &MotionExecutor::getCoarseMotion, this);
	execute_fine_motion_        = node_handle_.advertiseService("execute_fine_motion", &MotionExecutor::executeFineMotion, this);
	execute_stitched_motion_    = node_handle_.advertiseService("execute_stitched_motion", &MotionExecutor::executeStitchedMotion, this);
	execute_calibration_motion_ = node_handle_.advertiseService("execute_calibration_motion", &MotionExecutor::executeCalibrationMotion, this);*/
	
	move_to_home_               = node_handle_.advertiseService("move_to_home", &MotionExecutor::moveToHome, this);

  create_pick_movements_          = node_handle_.advertiseService("create_pick_movements", &MotionExecutor::createPickMovements, this);
  
  execute_coarse_motion_          = node_handle_.advertiseService("execute_coarse_motion", &MotionExecutor::executeCoarseMotion, this);

  execute_coarse_move_          = node_handle_.advertiseService("execute_coarse_move", &MotionExecutor::executeCoarseMove, this);


  
	/*move_to_calibrate_shelf_    = node_handle_.advertiseService("move_to_calibrate_shelf", &MotionExecutor::moveToCalibrateShelf, this);
	move_to_calibrate_tote_     = node_handle_.advertiseService("move_to_calibrate_tote", &MotionExecutor::moveToCalibrateTote, this);
	grasp_pose_visualizer_      = node_handle_.advertise<geometry_msgs::PoseStamped>("the_chosen_one", 10, true);
	*/

	// Fill cache with all yaml files from directory.
	//current_group_->setMaxVelocityScalingFactor(trajectory_velocity_scaling_);
	rs_ = current_group_->getCurrentState();
}

/*
bool MotionExecutor::executeCoarseMotion(apc16delft_msgs::ExecuteCoarseMotion::Request & req, apc16delft_msgs::ExecuteCoarseMotion::Response & res) {
	ROS_DEBUG_STREAM("Received request to execute coarse motion from MasterPose " << req.start << " to MasterPose " << req.target << ".");

	trajectory_msgs::JointTrajectory cached_trajectory = trajectory_cache_.lookupTrajectory(req.start, req.target);

	if (cached_trajectory.points.size() == 0) {
		ROS_ERROR_STREAM("Cache returned empty trajectory! Aborting.");
		res.error.code    = 1; // TODO
		res.error.message = "Cache returned empty trajectory! Aborting.";
		return true;
	}

	current_group_ = req.target.group_name == apc16delft_msgs::MasterPoseDescriptor::GROUP_TOOL0 ? &tool0_group_ : &tool1_group_;
	current_group_->setGoalTolerance(0.001);


	if (motion_sanity_checker_.checkTrajectorySanity(cached_trajectory, current_group_)) {
		// Set the motion plan trajectory to be the found trajectory in cache.
		plan_.trajectory_.joint_trajectory = cached_trajectory;
		current_group_->execute(plan_);
		motion_sanity_checker_.waitForRobotToStop();
		return true;
	} else {
		res.error.code    = 1; // TODO
		res.error.message = "Will not execute coarse motion due to safety reasons!";
		return true;
	}
}


bool MotionExecutor::getCoarseMotion(apc16delft_msgs::GetCoarseMotion::Request & req, apc16delft_msgs::GetCoarseMotion::Response & res) {
	ROS_DEBUG_STREAM("Received request to retrieve coarse motion from MasterPose " << req.start << " to MasterPose " << req.target << ".");

	trajectory_msgs::JointTrajectory cached_trajectory = trajectory_cache_.lookupTrajectory(req.start, req.target);

	if (cached_trajectory.points.size() == 0) {
		ROS_ERROR_STREAM(" Cache returned empty trajectory! Aborting.");
		res.error.code    = 1; // TODO
		res.error.message = "Cache returned empty trajectory!.";
		return true;
	} else {
		res.trajectory = cached_trajectory;
		res.error.code = apc16delft_msgs::Error::SUCCESS;
		res.error.message = "Retrieved requested trajectory from cache.";
		return true;
	}
}



bool MotionExecutor::executeStitchedMotion(apc16delft_msgs::ExecuteStitchedMotion::Request & req, apc16delft_msgs::ExecuteStitchedMotion::Response & res) {
	ROS_DEBUG_STREAM("Received request to stitch and execute " << req.trajectories.size() << "trajectories");

	double stitched_velocity_scaling;
	ROS_INFO_STREAM("req vel scaling: " << req.velocity_scaling);
	if (req.velocity_scaling > 0.0){
		stitched_velocity_scaling = req.velocity_scaling;
		ROS_INFO_STREAM("setting velocity scaling from request: " << stitched_velocity_scaling);
	} else {
		stitched_velocity_scaling = trajectory_velocity_scaling_;
		ROS_INFO_STREAM("setting velocity scaling from parameter: " << stitched_velocity_scaling);
	}


	// adding first trajectory to the plan
	if (req.trajectories[0].points.size() < 2) {
		ROS_ERROR_STREAM("Trajectory " << 0 << " has fewer than 2 states!");
		res.error.code = 1;
		res.error.message = "Too few points to stitch!";
		return true;
	}
	plan_.trajectory_.joint_trajectory = req.trajectories[0];

	for (int i = 1; i < req.trajectories.size(); i++) {
		ROS_DEBUG_STREAM("stitching " << i << " to " << (i - 1));
		if (req.trajectories[i].points.size() < 2) {
			ROS_ERROR_STREAM("Trajectory " << i << " has fewer than 2 states!");
			res.error.code = 1;
			res.error.message = "Too few points to stitch!";
			return true;
		}

		bool sane_connection = motion_sanity_checker_.checkTrajectorySanity(req.trajectories[i-1], req.trajectories[i]);
		if (!sane_connection) {
			res.error.code = MotionExecutor::MOTION_SAFETY_VIOLATION;
			res.error.message = "Motion safety violation between traj " + std::to_string(i-1) + " and " + std::to_string(i);
			return true;
		}

		// Insert the points into the plan
		plan_.trajectory_.joint_trajectory.points.insert(
			plan_.trajectory_.joint_trajectory.points.end(),
			req.trajectories[i].points.begin()+1,
			req.trajectories[i].points.end()
		);
	}

	// Clear the velocities and accelerations
	for (size_t idx = 0; idx < plan_.trajectory_.joint_trajectory.points.size(); idx++) {
		plan_.trajectory_.joint_trajectory.points[idx].velocities.clear();
		plan_.trajectory_.joint_trajectory.points[idx].accelerations.clear();
	}

	rs_->setVariablePositions(plan_.trajectory_.joint_trajectory.joint_names, plan_.trajectory_.joint_trajectory.points.front().positions);
	current_group_->setStartState(*rs_);
	robot_trajectory::RobotTrajectory rt(rs_->getRobotModel(), current_group_->getName());

	// Fill robot trajectory with computed cartesian path
	rt.setRobotTrajectoryMsg(*rs_, plan_.trajectory_);

	// Add velocities to the trajectory using iterative parabolic time parameterization
	trajectory_processing::IterativeParabolicTimeParameterization iptp;
	iptp.computeTimeStamps(rt, stitched_velocity_scaling);

	std::vector<int> reset_indices;
	for (int i = 0; i < req.delay_indices.size(); i++){
		rs_->setVariablePositions(plan_.trajectory_.joint_trajectory.joint_names, plan_.trajectory_.joint_trajectory.points[req.delay_indices[i]+i].positions);
		rt.insertWayPoint(req.delay_indices[i]+i+1,*rs_,req.delay_times[i]);

		reset_indices.push_back(req.delay_indices[i]);
	}
	
	rt.getRobotTrajectoryMsg(plan_.trajectory_);

	for (int i = 0; i<reset_indices.size(); i++){
		plan_.trajectory_.joint_trajectory.points[reset_indices[i]].velocities.assign(
			plan_.trajectory_.joint_trajectory.points[reset_indices[i]].velocities.size(),
			0.0);
		plan_.trajectory_.joint_trajectory.points[reset_indices[i]].accelerations.assign(
			plan_.trajectory_.joint_trajectory.points[reset_indices[i]].accelerations.size(),
			0.0);
		
		plan_.trajectory_.joint_trajectory.points[reset_indices[i]+1].velocities.assign(
			plan_.trajectory_.joint_trajectory.points[reset_indices[i]+1].velocities.size(),
			0.0);
		plan_.trajectory_.joint_trajectory.points[reset_indices[i]+1].accelerations.assign(
			plan_.trajectory_.joint_trajectory.points[reset_indices[i]+1].accelerations.size(),
			0.0);	
	}

	ROS_INFO_STREAM(plan_.trajectory_.joint_trajectory);


	// perform motion sanity check for final trajectory and execute.
	if (!motion_sanity_checker_.checkTrajectorySanity(plan_.trajectory_.joint_trajectory, current_group_)) {
		res.error.code = MotionExecutor::MOTION_SAFETY_VIOLATION;
		res.error.message = "Motion safety violation!";
		return true;
	}

	trajectory_tracker_.startTracking(plan_.trajectory_.joint_trajectory);
	current_group_->setGoalTolerance(0.001);
	bool success = current_group_->execute(plan_);
	//trajectory_tracker_.stopTracking();

	if (!success) {
		res.error.code    = 1; // TODO
		res.error.message = "Failed to execute motion plan.";
		return true;
	}
	res.error.code = apc16delft_msgs::Error::SUCCESS;
	res.error.message = "Successfully executed stitched motion.";
	return true;
}

bool MotionExecutor::executeFineMotion(apc16delft_msgs::ExecuteFineMotion::Request & req, apc16delft_msgs::ExecuteFineMotion::Response & res) {
	if (req.trajectory.points.size() == 0) {
		res.error.code    = 1; // TODO
		res.error.message = "Can not execute empty trajectory.";
		return true;
	}

	if (motion_sanity_checker_.checkTrajectorySanity(req.trajectory, current_group_)) {
		plan_.trajectory_.joint_trajectory = req.trajectory;
		current_group_->setGoalTolerance(0.001);
		current_group_->execute(plan_);
		motion_sanity_checker_.waitForRobotToStop();
		res.error.code = apc16delft_msgs::Error::SUCCESS;
		res.error.message = "Suceesfully executed fine motion.";
		return true;
	} else {
		res.error.code    = MotionExecutor::MOTION_SAFETY_VIOLATION;
		res.error.message = "Motion safety violation!";
		return true;
	}
}
bool MotionExecutor::checkCalibrationTrajectorySanity(trajectory_msgs::JointTrajectory & motion_trajectory, moveit::planning_interface::MoveGroup* current_group) {
	std::vector<double> current_joint_values;
	double *joint_value_ptr;

	robot_state::RobotStatePtr kinematic_state(current_group->getCurrentState());
	//Get current joint values.
	double joint_diff;
	double distance = 0.0;
	joint_value_ptr = kinematic_state->getVariablePositions();
	const std::vector<std::string> joint_names= kinematic_state->getVariableNames();
	//Compute distance between current state and the trajectory start state
	std::vector<std::string>::iterator it;
	for (size_t traj_idx = 0; traj_idx < motion_trajectory.points.size(); traj_idx++) {
		for (size_t idx=0; idx < motion_trajectory.points[traj_idx].positions.size(); idx++) {
			it = std::find(motion_trajectory.joint_names.begin(), motion_trajectory.joint_names.end(),joint_names[idx]);
			int pos_idx = std::distance(motion_trajectory.joint_names.begin(), it);

			joint_diff = joint_value_ptr[idx] - motion_trajectory.points[traj_idx].positions[pos_idx];
			//Adjust starting point of cached trajectory to current position for practical reasons.
			motion_trajectory.points[0].positions[pos_idx] = joint_value_ptr[idx];
			ROS_DEBUG_STREAM(joint_names[idx] <<": " << joint_value_ptr[idx]);
			distance += (joint_diff*joint_diff);
		}
		distance = sqrt(distance);
		if (distance > calibration_trajectory_tolerance_) {
			ROS_ERROR_STREAM("Motion safety check violation, distance is " << distance << " which is above the threshold of " << calibration_trajectory_tolerance_ << ".");
			return false;
			break;
		}
	}
	return true;

}
bool MotionExecutor::executeCalibrationMotion(apc16delft_msgs::ExecuteCalibrationMotion::Request & req, apc16delft_msgs::ExecuteCalibrationMotion::Response & res) {
	ROS_INFO_STREAM("Received request to execute calibration motion to " << req.calibration_pose << ".");

	current_group_ = &tool0_group_;
	current_group_->clearPoseTargets();

	std::vector<double> joint_value_current;
	std::vector<double> joint_value_target;


	robot_state::RobotStatePtr current_state(current_group_->getCurrentState());
	const robot_state::JointModelGroup *jmg = current_state->getJointModelGroup(current_group_->getName());

	current_state->copyJointGroupPositions(jmg,joint_value_current);

	const robot_state::RobotState target_state = current_group_->getJointValueTarget();

	current_group_->setJointValueTarget(req.calibration_pose, "gripper_tool0");
	current_group_->setMaxVelocityScalingFactor(trajectory_velocity_scaling_);
	moveit::planning_interface::MoveGroup::Plan calibration_motion_plan;

	current_group_->setStartState(*current_state);
	current_group_->setPlannerId("RRTConnectkConfigDefault");

	current_group_->allowReplanning(true);
	current_group_->setNumPlanningAttempts(5);

	if (current_group_->plan(calibration_motion_plan) != 1) {
		ROS_ERROR_STREAM("No motion plan found. Pose rejected.");
		res.error.code    = 1; // TODO
		res.error.message = "No motion plan found..";
		return false;
	}

	if (!checkCalibrationTrajectorySanity(calibration_motion_plan.trajectory_.joint_trajectory, current_group_)) {
		ROS_INFO_STREAM("Calibration motion not allowed due to configuration change.");
		res.error.code    = 1; // TODO
		res.error.message = "Resulting plan has a configuration change. Rejecting.";
		return false;
	}

	if (!motion_sanity_checker_.checkTrajectorySanity(calibration_motion_plan.trajectory_.joint_trajectory, current_group_)) {
		ROS_ERROR_STREAM("Calibration motion trajectory sanity check failed!");
		res.error.code    = 1; // TODO
		res.error.message = "Calibration trajectory not safe to execute";
		return false;
	}

	ROS_INFO_STREAM("Executing calibration motion.");
	current_group_->execute(calibration_motion_plan);
	return true;
}

*/

bool MotionExecutor::moveToHome(lenny_msgs::MoveToHome::Request & req, lenny_msgs::MoveToHome::Response & res)
{
	moveit::planning_interface::MoveGroupInterface group(req.move_group);
	current_group_ = &group;
	current_group_->setGoalTolerance(0.001);
	current_group_->clearPoseTargets();
	current_group_->setPlannerId("RRTConnectkConfigDefault");

	current_group_->allowReplanning(true);
	current_group_->setNumPlanningAttempts(10);
	current_group_->setMaxVelocityScalingFactor(trajectory_velocity_scaling_);

    current_group_->setStartState(*current_group_->getCurrentState());

	//geometry_msgs::PoseStamped gripper_current_pose = current_group_->getCurrentPose("gripper_tool0");

	/*if(gripper_current_pose.pose.position.y > 0.44) {
		ROS_ERROR_STREAM("Preventing the robot from breaking the shelf again.");
		res.error.code = 1;
		res.error.message = "Current robot position too close to shelf.";
		return true;
	}*/

	current_group_->setNamedTarget(req.pose_name);

	moveit::planning_interface::MoveGroupInterface::Plan motion_plan;
  
	if (current_group_->plan(motion_plan) != 1)
	{
       ROS_ERROR_STREAM("No motion plan found.");
       res.success = false;
        return false;
    
	}


	bool success = (current_group_->execute(motion_plan)== moveit::planning_interface::MoveItErrorCode::SUCCESS);
//bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO_STREAM("Waiting for robot to STOP");
	success=motion_utilities_.waitForRobotToStop();
  //ros::Duration(1.5).sleep();
	if (!success)
	{
		ROS_ERROR_STREAM("Failed to execute motion.");
    res.success = false;
		return false;
	}
  else
  {
    	ROS_INFO("Finished execution.");
      res.success = true;
      return true;
    
  }

	
}


bool MotionExecutor::createPickMovements(lenny_msgs::CreatePickMovements::Request & req, lenny_msgs::CreatePickMovements::Response & res) 
{
	ROS_DEBUG_STREAM("Received request to create pick moves " );

  tf::Transform world_to_tcp_tf;
  tf::Transform world_to_object_tf;
 
	object_pose_=req.object_pose;
  
         
  tf::Vector3 object_position(object_pose_.position.x, object_pose_.position.y, object_pose_.position.z);

  world_to_tcp_tf.setOrigin(object_position);

 
  /* Setting tcp orientation
	   * Inverting the approach direction so that the tcp points towards the object instead of
	   * away from it.*/
  world_to_tcp_tf.setRotation(world_to_object_tf.getRotation());


  
  geometry_msgs::Pose start_pose, target_pose, end_pose;
  std::vector<geometry_msgs::Pose> poses;

  // creating start pose by applying a translation along +z by approach distance
  // creating start pose by applying a translation along +X by approach distance (nico)
  
  tf::poseTFToMsg(Transform(Quaternion::getIdentity(),Vector3(0,0,0.2))*world_to_tcp_tf,start_pose);


  // converting target pose
  tf::poseTFToMsg(world_to_tcp_tf,target_pose);

  // creating end pose by applying a translation along +z by retreat distance
  tf::poseTFToMsg(Transform(Quaternion::getIdentity(),Vector3(0,0,0.3))*world_to_tcp_tf,end_pose);

  poses.clear();
  poses.push_back(start_pose);
  poses.push_back(target_pose);
  poses.push_back(end_pose);
  
  
  
  res.success = true;
  res.robot_movements = poses;
  
  return true;
}


bool MotionExecutor::executeCoarseMotion(lenny_msgs::ExecuteCoarseMotion::Request & req, lenny_msgs::ExecuteCoarseMotion::Response & res) 
{
	ROS_DEBUG_STREAM("Received request to execute coarse motion " );
	geometry_msgs::Pose target_pose;
	target_pose=req.target_pose;

    moveit::planning_interface::MoveGroupInterface group(req.move_group);
	current_group_ = &group;
	current_group_->setGoalTolerance(0.001);
	current_group_->setPlannerId("RRTConnectk");
	current_group_->allowReplanning(true);
	current_group_->setNumPlanningAttempts(10);
	current_group_->setMaxVelocityScalingFactor(trajectory_velocity_scaling_);

    current_group_->setStartState(*current_group_->getCurrentState());

// constructing motion plan goal constraints
   std::vector<double> position_tolerances(3,0.01);
   std::vector<double> orientation_tolerances(3,0.01);
   geometry_msgs::PoseStamped p;
   p.header.frame_id = "torso_base_link";
   p.pose = target_pose;
   moveit_msgs::Constraints pose_goal = kinematic_constraints::constructGoalConstraints("arm_left_link_7_t",p,0.01,
				   0.01);



   // creating motion plan request
   moveit_msgs::GetMotionPlan motion_plan;
   moveit_msgs::MotionPlanRequest &plan_req = motion_plan.request.motion_plan_request;
   moveit_msgs::MotionPlanResponse &plan_res = motion_plan.response.motion_plan_response;
   
   plan_req.planner_id = "RRTConnectk";
   plan_req.group_name = req.move_group;
   plan_req.goal_constraints.push_back(pose_goal);
   plan_req.allowed_planning_time = 60;
   plan_req.num_planning_attempts = 10;
   plan_req.max_velocity_scaling_factor = 0.5;

    //Updating current robot state
    moveit_msgs::RobotState robot_state;
    robot_state::RobotStatePtr current_state(current_group_->getCurrentState());
    const robot_state::JointModelGroup *joint_model_group = current_state->getJointModelGroup(current_group_->getName());
    current_group_->setStartState(*current_state);

    robot_state::robotStateToRobotStateMsg(*current_state,plan_req.start_state);


moveit::planning_interface::MoveGroupInterface::Plan plan;
 // request motion plan
       bool success = false;
       //moveit::planning_interface::MoveItErrorCode success;
       if(motion_plan_client.call(motion_plan) && plan_res.error_code.val == plan_res.error_code.SUCCESS)
       {
               // saving motion plan results
               plan.start_state_ = plan_res.trajectory_start;
               plan.trajectory_ = plan_res.trajectory;
               success = true;
               
       }
       
       if (success)
       {
          current_group_->execute(plan);
          success = motion_utilities_.waitForRobotToStop();
          
          if (success)
          {
             current_group_->execute(plan);
              res.success = true;
              return true;
          }
          else 
          {
            res.success = false;
            return false;
          }
        }
        else
          return false;



	

}


bool MotionExecutor::executeCoarseMove(lenny_msgs::ExecuteCoarseMove::Request & req, lenny_msgs::ExecuteCoarseMove::Response & res) 
{
	ROS_DEBUG_STREAM("Received request to execute coarse motion " );
	geometry_msgs::Pose target_pose;
	target_pose=req.target_pose;

    moveit::planning_interface::MoveGroupInterface group(req.move_group);
	current_group_ = &group;
	current_group_->setGoalTolerance(0.001);
	current_group_->setPlannerId("RRTConnectk");
	current_group_->allowReplanning(true);
	current_group_->setNumPlanningAttempts(10);
	current_group_->setMaxVelocityScalingFactor(trajectory_velocity_scaling_);

    current_group_->setStartState(*current_group_->getCurrentState());

// constructing motion plan goal constraints
   std::vector<double> position_tolerances(3,0.01);
   std::vector<double> orientation_tolerances(3,0.01);
   geometry_msgs::PoseStamped p;
   p.header.frame_id = "torso_base_link";
   p.pose = target_pose;
   moveit_msgs::Constraints pose_goal = kinematic_constraints::constructGoalConstraints("arm_left_link_7_t",p,0.01,
				   0.01);



   // creating motion plan request
   moveit_msgs::GetMotionPlan motion_plan;
   moveit_msgs::MotionPlanRequest &plan_req = motion_plan.request.motion_plan_request;
   moveit_msgs::MotionPlanResponse &plan_res = motion_plan.response.motion_plan_response;
   
   plan_req.planner_id = "RRTConnectk";
   plan_req.group_name = req.move_group;
   plan_req.goal_constraints.push_back(pose_goal);
   plan_req.allowed_planning_time = 60;
   plan_req.num_planning_attempts = 10;
   plan_req.max_velocity_scaling_factor = 0.5;

    //Updating current robot state
    moveit_msgs::RobotState robot_state;
    robot_state::RobotStatePtr current_state(current_group_->getCurrentState());
    const robot_state::JointModelGroup *joint_model_group = current_state->getJointModelGroup(current_group_->getName());
    current_group_->setStartState(*current_state);

    robot_state::robotStateToRobotStateMsg(*current_state,plan_req.start_state);


moveit::planning_interface::MoveGroupInterface::Plan plan;
 // request motion plan
       bool success = false;
       //moveit::planning_interface::MoveItErrorCode success;
       if(motion_plan_client.call(motion_plan) && plan_res.error_code.val == plan_res.error_code.SUCCESS)
       {
               // saving motion plan results
               plan.start_state_ = plan_res.trajectory_start;
               plan.trajectory_ = plan_res.trajectory;
               success = true;
               
       }
       
       if (success)
       {
          current_group_->execute(plan);
          success = motion_utilities_.waitForRobotToStop();
          
          if (success)
          {
             current_group_->execute(plan);
              res.success = true;
              return true;
          }
          else 
          {
            res.success = false;
            return false;
          }
        }
        else
        {
          res.success = false;
          
          return false;
        }


	

}
/*

bool MotionExecutor::moveToCalibrateShelf(apc16delft_msgs::MoveToCalibrateShelf::Request &, apc16delft_msgs::MoveToCalibrateShelf::Response & res) {
	current_group_ = &tool0_group_;
	current_group_->clearPoseTargets ();
	current_group_->setPlannerId("RRTConnectkConfigDefault");

	current_group_->allowReplanning(true);
	current_group_->setNumPlanningAttempts(5);

	current_group_->setNamedTarget("calibrate_shelf");

	moveit::planning_interface::MoveGroup::Plan motion_plan;
	if (current_group_->plan(motion_plan) != 1) {
		res.error.code    = 1; // TODO
		res.error.message = "No motion plan found";
		return false;
	}

	if (!current_group_->execute(motion_plan)) {
		res.error.code    = 1; // TODO
		res.error.message = "Failed to execute motion plan.";
		return false;
	}

	return true;
}

bool MotionExecutor::moveToCalibrateTote(apc16delft_msgs::MoveToCalibrateTote::Request &, apc16delft_msgs::MoveToCalibrateTote::Response & res) {
	current_group_ = &tool0_group_;
	current_group_->clearPoseTargets ();
	current_group_->setPlannerId("RRTConnectkConfigDefault");

	current_group_->allowReplanning(true);
	current_group_->setNumPlanningAttempts(5);

	current_group_->setNamedTarget("calibrate_tote");

	moveit::planning_interface::MoveGroup::Plan motion_plan;
	if (current_group_->plan(motion_plan) != 1) {
		res.error.code    = 1; // TODO
		res.error.message = "No motion plan found";
		return false;
	}

	if (!current_group_->execute(motion_plan)) {
		res.error.code    = 1; // TODO
		res.error.message = "Failed to execute motion plan.";
		return false;
	}

	return true;
}*/

} // namespace

int main(int argc, char ** argv)
{
	ros::init(argc, argv, "motion_executor");
	
	pirLenny::MotionExecutor motion_executor;

	ros::Rate loop_rate(1000);
	while(ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
}
