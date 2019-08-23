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
	current_moveit_group_(&arm_right_group_),
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
	
	move_to_pose_               = node_handle_.advertiseService("move_to_pose", &MotionExecutor::moveToPose, this);

  create_pick_movements_          = node_handle_.advertiseService("create_pick_movements", &MotionExecutor::createPickMovements, this);
  

  execute_coarse_motion_          = node_handle_.advertiseService("execute_coarse_motion", &MotionExecutor::executeCoarseMotion, this);

  plan_coarse_motion_          = node_handle_.advertiseService("plan_coarse_motion", &MotionExecutor::planCoarseMotion, this);

  plan_execute_fine_motion_          = node_handle_.advertiseService("plan_execute_fine_motion", &MotionExecutor::planExecuteFineMotion, this);

  extendTCP_               = node_handle_.advertiseService("extendTCP", &MotionExecutor::extendTCP, this);

  restoreTCP_               = node_handle_.advertiseService("restoreTCP", &MotionExecutor::restoreTCP, this);

  //plan_fine_motion_          = node_handle_.advertiseService("plan_fine_motion", &MotionExecutor::planFineMotion, this);

  //Motion plan client
  motion_plan_client = node_handle_.serviceClient<moveit_msgs::GetMotionPlan>("plan_kinematic_path");
 

  
	/*move_to_calibrate_shelf_    = node_handle_.advertiseService("move_to_calibrate_shelf", &MotionExecutor::moveToCalibrateShelf, this);
	move_to_calibrate_tote_     = node_handle_.advertiseService("move_to_calibrate_tote", &MotionExecutor::moveToCalibrateTote, this);
	grasp_pose_visualizer_      = node_handle_.advertise<geometry_msgs::PoseStamped>("the_chosen_one", 10, true);
	*/

	// Fill cache with all yaml files from directory.
	//current_moveit_group_->setMaxVelocityScalingFactor(trajectory_velocity_scaling_);
	rs_ = current_moveit_group_->getCurrentState();
  
  
  //Initialize transformation from tcp to wrist
  tf::TransformListener listener;
  
  //TODO: we need to have one tcp_to_wrist_tf per arm
  try
  {
    ///TODO: this is a fixed transformation, this has to be generic too
	  listener.waitForTransform("arm_right_tcp_link", "arm_right_link_7_t",ros::Time::now(),ros::Duration(3.0f));
    listener.lookupTransform("arm_right_tcp_link", "arm_right_link_7_t", ros::Time(0), tcp_to_wrist_tf_right_);
    tcp_to_wrist_tf_=tcp_to_wrist_tf_right_;

  }
  catch (tf::TransformException &ex) 
  {
    ROS_WARN("%s",ex.what());
    ros::Duration(1.0).sleep();
  }

  
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

	current_moveit_group_ = req.target.group_name == apc16delft_msgs::MasterPoseDescriptor::GROUP_TOOL0 ? &tool0_group_ : &tool1_group_;
	current_moveit_group_->setGoalTolerance(0.001);


	if (motion_sanity_checker_.checkTrajectorySanity(cached_trajectory, current_moveit_group_)) {
		// Set the motion plan trajectory to be the found trajectory in cache.
		plan_.trajectory_.joint_trajectory = cached_trajectory;
		current_moveit_group_->execute(plan_);
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
	current_moveit_group_->setStartState(*rs_);
	robot_trajectory::RobotTrajectory rt(rs_->getRobotModel(), current_moveit_group_->getName());

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
	if (!motion_sanity_checker_.checkTrajectorySanity(plan_.trajectory_.joint_trajectory, current_moveit_group_)) {
		res.error.code = MotionExecutor::MOTION_SAFETY_VIOLATION;
		res.error.message = "Motion safety violation!";
		return true;
	}

	trajectory_tracker_.startTracking(plan_.trajectory_.joint_trajectory);
	current_moveit_group_->setGoalTolerance(0.001);
	bool success = current_moveit_group_->execute(plan_);
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

	if (motion_sanity_checker_.checkTrajectorySanity(req.trajectory, current_moveit_group_)) {
		plan_.trajectory_.joint_trajectory = req.trajectory;
		current_moveit_group_->setGoalTolerance(0.001);
		current_moveit_group_->execute(plan_);
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

	current_moveit_group_ = &tool0_group_;
	current_moveit_group_->clearPoseTargets();

	std::vector<double> joint_value_current;
	std::vector<double> joint_value_target;


	robot_state::RobotStatePtr current_state(current_moveit_group_->getCurrentState());
	const robot_state::JointModelGroup *jmg = current_state->getJointModelGroup(current_moveit_group_->getName());

	current_state->copyJointGroupPositions(jmg,joint_value_current);

	const robot_state::RobotState target_state = current_moveit_group_->getJointValueTarget();

	current_moveit_group_->setJointValueTarget(req.calibration_pose, "gripper_tool0");
	current_moveit_group_->setMaxVelocityScalingFactor(trajectory_velocity_scaling_);
	moveit::planning_interface::MoveGroup::Plan calibration_motion_plan;

	current_moveit_group_->setStartState(*current_state);
	current_moveit_group_->setPlannerId("RRTConnectkConfigDefault");

	current_moveit_group_->allowReplanning(true);
	current_moveit_group_->setNumPlanningAttempts(5);

	if (current_moveit_group_->plan(calibration_motion_plan) != 1) {
		ROS_ERROR_STREAM("No motion plan found. Pose rejected.");
		res.error.code    = 1; // TODO
		res.error.message = "No motion plan found..";
		return false;
	}

	if (!checkCalibrationTrajectorySanity(calibration_motion_plan.trajectory_.joint_trajectory, current_moveit_group_)) {
		ROS_INFO_STREAM("Calibration motion not allowed due to configuration change.");
		res.error.code    = 1; // TODO
		res.error.message = "Resulting plan has a configuration change. Rejecting.";
		return false;
	}

	if (!motion_sanity_checker_.checkTrajectorySanity(calibration_motion_plan.trajectory_.joint_trajectory, current_moveit_group_)) {
		ROS_ERROR_STREAM("Calibration motion trajectory sanity check failed!");
		res.error.code    = 1; // TODO
		res.error.message = "Calibration trajectory not safe to execute";
		return false;
	}

	ROS_INFO_STREAM("Executing calibration motion.");
	current_moveit_group_->execute(calibration_motion_plan);
	return true;
}

*/

bool MotionExecutor::moveToPose(lenny_msgs::MoveToPose::Request & req, lenny_msgs::MoveToPose::Response & res)
{
	
	moveit::planning_interface::MoveGroupInterface group(req.move_group);
	current_moveit_group_ = &group;
	current_moveit_group_->setGoalTolerance(0.001);
	current_moveit_group_->clearPoseTargets();
	current_moveit_group_->setPlannerId("RRTConnectkConfigDefault");

	current_moveit_group_->allowReplanning(true);
	current_moveit_group_->setNumPlanningAttempts(10);
	current_moveit_group_->setMaxVelocityScalingFactor(trajectory_velocity_scaling_);

    current_moveit_group_->setStartState(*current_moveit_group_->getCurrentState());

	//geometry_msgs::PoseStamped gripper_current_pose = current_moveit_group_->getCurrentPose("gripper_tool0");

	/*if(gripper_current_pose.pose.position.y > 0.44) {
		ROS_ERROR_STREAM("Preventing the robot from breaking the shelf again.");
		res.error.code = 1;
		res.error.message = "Current robot position too close to shelf.";
		return true;
	}*/

	current_moveit_group_->setNamedTarget(req.pose_name);

    
	moveit::planning_interface::MoveGroupInterface::Plan motion_plan;
  
	if (current_moveit_group_->plan(motion_plan) != 1)
	{
       ROS_ERROR_STREAM("No motion plan found.");
       res.success = false;
        return false;
    
	}


	bool success = (current_moveit_group_->execute(motion_plan)== moveit::planning_interface::MoveItErrorCode::SUCCESS);
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

bool MotionExecutor::checkWayPointReachability(const geometry_msgs::Pose& waypoint) 
{
  
  //Solve IK, attemps 10, timeout for each attempt 0.1 s
  bool found_ik = kinematic_state_->setFromIK(joint_model_group_, waypoint, 10, 0.1);
  
  if(found_ik)
  {
    ROS_INFO_STREAM("Found IK " );
    return true;
    

  }
  else
  {
    ROS_ERROR_STREAM("Did not find IK solution");
    return false;
  }
  
}

bool MotionExecutor::createPickMovements(lenny_msgs::CreatePickMovements::Request & req, lenny_msgs::CreatePickMovements::Response & res) 
{
	ROS_DEBUG_STREAM("Received request to create pick moves " );


  tf::Transform world_to_tcp_tf;
  tf::Transform world_to_object_tf;
 
	object_pose_=req.object_pose;

  
  tf::poseMsgToTF(object_pose_,world_to_object_tf);
         
  tf::Vector3 object_position(object_pose_.position.x, object_pose_.position.y, object_pose_.position.z);
	
  world_to_tcp_tf.setOrigin(object_position);

 
  /* Setting tcp orientation
	   * Inverting the approach direction so that the tcp points towards the object instead of
	   * away from it.*/
  world_to_tcp_tf.setRotation(world_to_object_tf.getRotation());


  
  geometry_msgs::Pose start_pose, target_pose, end_pose;
  std::vector<geometry_msgs::Pose> poses, wrist_pick_poses;

  // creating start pose by applying a translation along +z by approach distance
  // creating start pose by applying a translation along +X by approach distance (nico)
  
  tf::poseTFToMsg(Transform(Quaternion::getIdentity(),Vector3(0,0,0.2))*world_to_tcp_tf,start_pose);
  
 
  // converting target pose
  tf::poseTFToMsg(world_to_tcp_tf,target_pose);
  
  
  //TODO: do this based on the group.. that has to be passed as parameter
  tf::StampedTransform tcp_to_wrist_tf; 
  tcp_to_wrist_tf=tcp_to_wrist_tf_right_; 
  

 
 // creating end pose by applying a translation along +z by retreat distance
  tf::poseTFToMsg(Transform(Quaternion::getIdentity(),Vector3(0,0,0.2))*world_to_tcp_tf,end_pose);
  
  ///TODO: do something if waypoint is not reachable   
  poses.clear();
  poses.push_back(start_pose);
  poses.push_back(target_pose);
  poses.push_back(end_pose);

  //Convert poses to wrist
  //Tf listener
  //I need the trasnformation between wrist and tcp this is fixed
	
	std::vector<geometry_msgs::Pose> wrist_poses;
	wrist_poses.resize(poses.size());

	// applying transform to each tcp poses
	tf::Transform world_to_wrist_tf, world_to_tcp_tf_sec;

	for(unsigned int i = 0; i < poses.size(); i++)
	{
		//tf::poseMsgToTF(poses[i],world_to_tcp_tf_sec);
		
		tf::poseMsgToTF(poses[i],world_to_tcp_tf_sec);
		//std::cout << world_to_tcp_tf_sec.translation << std::endl;
		tf::Transform req_to_target;
		//req_to_target = world_to_tcp_tf_sec * tcp_to_wrist_tf;	
		//wrist_poses[i]=poses[i];
		world_to_wrist_tf = world_to_tcp_tf_sec*tcp_to_wrist_tf;
       
		tf::poseTFToMsg(world_to_wrist_tf,wrist_poses[i]);
		
	}


    
	res.success = true;
	res.robot_movements = wrist_poses;
    	
	return true;
  
}




bool MotionExecutor::executeCoarseMotion(lenny_msgs::ExecuteCoarseMotion::Request & req, lenny_msgs::ExecuteCoarseMotion::Response & res) 
{
  ROS_DEBUG_STREAM("Received request to execute coarse motion " );

	///TODO: do it for the other groups and change the move_home_fuction too
	if(req.move_group=="arms")
		current_moveit_group_ = &arms_group_;
	if(req.move_group=="arm_right")
		current_moveit_group_ = &arm_right_group_;

	moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	
	my_plan.trajectory_.joint_trajectory = req.coarse_trajectory;

		

	moveit_msgs::RobotState robot_state;
		
	const robot_state::JointModelGroup *joint_model_group = current_moveit_group_->getCurrentState()->getJointModelGroup(req.move_group);

	// constructing motion plan goal constraints
	current_moveit_group_->setStartState(*current_moveit_group_->getCurrentState());

    current_moveit_group_->setPlanningTime(10);
    current_moveit_group_->setGoalTolerance(0.01);
    ///TODO: check if it is required to specify both tolerances
    //current_moveit_group_->setGoalOrientationTolerance(0.01);
    //current_moveit_group_->setGoalPositionTolerance(0.01);
    current_moveit_group_->setMaxVelocityScalingFactor(0.5);
    current_moveit_group_->setPlannerId("RRTkConfigDefault");
    current_moveit_group_->setNumPlanningAttempts(5);
    current_moveit_group_->allowReplanning(true);
    
       
   	current_moveit_group_->execute(my_plan);
   	bool stop;
	
  
  stop = motion_utilities_.waitForRobotToStop();
	if(!stop)
	{
	  res.success = false;
	  return false;
	}
	else
	{
	  res.success = true;
	  return true;
 
	}
	

}



bool MotionExecutor::planCoarseMotion(lenny_msgs::PlanCoarseMotion::Request & req, lenny_msgs::PlanCoarseMotion::Response & res) 
{
  ROS_DEBUG_STREAM("Received request to PLAN coarse motion" );

	///TODO: do it for the other groups and change the move_home_fuction too
	if(req.move_group=="arms")
		current_moveit_group_ = &arms_group_;
	if(req.move_group=="arm_right")
		current_moveit_group_ = &arm_right_group_;


 /// kinematic_state_: it is the current kinematic configuration of the robot
  kinematic_state_ = moveit::core::RobotStatePtr(current_moveit_group_->getCurrentState());
  kinematic_state_->setToDefaultValues();
  
  const robot_model::RobotModelConstPtr &kmodel = kinematic_state_->getRobotModel();
  joint_model_group_ = kmodel->getJointModelGroup(req.move_group);
  



	moveit_msgs::RobotState robot_state;
		
	const robot_state::JointModelGroup *joint_model_group = current_moveit_group_->getCurrentState()->getJointModelGroup(req.move_group);

	// constructing motion plan goal constraints
    std::vector<double> position_tolerances(3,0.01);
	std::vector<double> orientation_tolerances(3,0.01);
	geometry_msgs::PoseStamped p;
	p.header.frame_id = "torso_base_link";
	geometry_msgs::Pose target_pose;
	target_pose=req.target_pose;


  //Check reachability
  bool foundIK = false;
  foundIK = checkWayPointReachability(target_pose);
  
  //if(foundIK)
  //{ 
    
      p.pose = target_pose;
      moveit_msgs::Constraints pose_goal = kinematic_constraints::constructGoalConstraints("arm_rigth_link_7_t",p,0.01,
      0.01);

      // creating motion plan request
      moveit_msgs::GetMotionPlan motion_plan;
      moveit_msgs::MotionPlanRequest &plan_req = motion_plan.request.motion_plan_request;
      moveit_msgs::MotionPlanResponse &plan_res = motion_plan.response.motion_plan_response;
      plan_req.start_state = robot_state;
        plan_req.start_state.is_diff = true;
           
      plan_req.planner_id = "RRTkConfigDefault";
      plan_req.group_name = req.move_group;
      plan_req.goal_constraints.push_back(pose_goal);
      plan_req.allowed_planning_time = 60;
      plan_req.num_planning_attempts = 10;
      plan_req.max_velocity_scaling_factor = 0.5;

    //std::cout << target_pose << std::endl;
     
      // request motion plan
      //bool success = false;
      current_moveit_group_->setPoseTarget(target_pose);
        current_moveit_group_->setStartState(*current_moveit_group_->getCurrentState());

      moveit::planning_interface::MoveGroupInterface::Plan my_plan;
      //moveit::planning_interface::MoveItErrorCode success;

      /*if(motion_plan_client.call(motion_plan) && plan_res.error_code.val == plan_res.error_code.SUCCESS)
      {
           // saving motion plan results
           plan.start_state_ = plan_res.trajectory_start;
           plan.trajectory_ = plan_res.trajectory;
           success = true;
           
      }*/
        current_moveit_group_->setPlanningTime(10);
        current_moveit_group_->setGoalTolerance(0.01);
        current_moveit_group_->setGoalOrientationTolerance(0.01);
        current_moveit_group_->setGoalPositionTolerance(0.01);
        current_moveit_group_->setMaxVelocityScalingFactor(0.5);
        current_moveit_group_->setPlannerId("RRTkConfigDefault");
        current_moveit_group_->setNumPlanningAttempts(5);
        current_moveit_group_->allowReplanning(true);
        
       
      bool success = (current_moveit_group_->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

      if(!success)
      {
        ROS_ERROR("No plan found");
        res.success = false;
        return false;
        
      }
      else
      {
        res.success = true;
        res.coarse_trajectory = my_plan.trajectory_.joint_trajectory;
        
        return true;
        
        
      }
  //}
  //else
  //{
  //  ROS_ERROR("No IK solution found for Coarse Point");
  //  res.success = false;
  //  return false;
    
  //}
       
  

}





bool MotionExecutor::planExecuteFineMotion(lenny_msgs::PlanExecuteFineMotion::Request & req, lenny_msgs::PlanExecuteFineMotion::Response & res) 
{
  ROS_DEBUG_STREAM("Received request to PLAN fine motion" );

	///TODO: do it for the other groups and change the move_home_fuction too
	if(req.move_group=="arms")
		current_moveit_group_ = &arms_group_;
	if(req.move_group=="arm_right")
		current_moveit_group_ = &arm_right_group_;
 
   
  moveit_msgs::RobotState robot_state;
		
	const robot_state::JointModelGroup *joint_model_group = current_moveit_group_->getCurrentState()->getJointModelGroup(req.move_group);
	// constructing motion plan goal constraints
	current_moveit_group_->setStartState(*current_moveit_group_->getCurrentState());
  /// kinematic_state_: it is the current kinematic configuration of the robot
  kinematic_state_ = moveit::core::RobotStatePtr(current_moveit_group_->getCurrentState());
  //kinematic_state_->setToDefaultValues();
 
  
  const robot_model::RobotModelConstPtr &kmodel = kinematic_state_->getRobotModel();
  joint_model_group_ = kmodel->getJointModelGroup(req.move_group);
  
  
  current_moveit_group_->setPlanningTime(10);
  current_moveit_group_->allowReplanning(true);
  
  ///TODO: check for pose if it is possible
  ///TODO: load parameters for planning from a yaml including for catesian path
  

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  
  moveit_msgs::RobotTrajectory trajectory;
  
  
  //Parameters
  double plan_time = 5;
  bool moveit_replan = true;
  bool avoid_collision = true;
  
  double cart_step_size = 0.01; //Path interpolation resolution
  //Threshold to prevent jumps in IK solution
  double cart_jump_thr = 0.0; // meters 
  
  
  //Defining poses
  std::vector<geometry_msgs::Pose> waypoints;
        
  //Check reachability
  bool foundIK= false;
  foundIK = checkWayPointReachability(req.target_poses[0]);
 
  if(foundIK)
  {
      waypoints.push_back(req.target_poses[0]);
   
      //Plan cartesian path
      double fraction;
      fraction = current_moveit_group_->computeCartesianPath(waypoints,
      cart_step_size,cart_jump_thr,trajectory,avoid_collision);
      
      robot_trajectory::RobotTrajectory rt(kinematic_state_->getRobotModel(), req.move_group);
      
      rt.setRobotTrajectoryMsg(*kinematic_state_,trajectory);
      
      ROS_INFO_STREAM("Pose reference frame: " << current_moveit_group_->getPoseReferenceFrame());
      
      /// CReate an iterativeParabolicTimeParametrization object
      trajectory_processing::IterativeParabolicTimeParameterization iptp;
      
      bool success = iptp.computeTimeStamps(rt);
      ROS_INFO("Computed time stamps %s ", success? "SUCCEDED":"FAILED");
      
      
      //Get robot trajectory msg
      rt.getRobotTrajectoryMsg(trajectory);
      //Plan the trajectory
      plan.trajectory_= trajectory;
      ROS_INFO("Visualizing plan (cartesian path) (%.2f%% achieved", fraction *100);
      
       
      current_moveit_group_->execute(plan);
      
      
      bool stop;
      stop = motion_utilities_.waitForRobotToStop();
      if(!stop)
      {
        res.success = false;
        return false;
      }
      else
      {
        res.success = true;
        return true;
     
      }
      
  }
  else
  {
    res.success = false;
    return false;
  }

  
  	

}



bool MotionExecutor::extendTCP(lenny_msgs::ExtendTCP::Request & req, lenny_msgs::ExtendTCP::Response & res) 
{
  ROS_DEBUG_STREAM("Received request to EXTEND TCP" );

  ///TODO: do it for the other groups and change the move_home_fuction too
  if(req.arm_name=="arm_right")
  {
    tcp_to_wrist_tf_right_.setOrigin(tf::Vector3(tcp_to_wrist_tf_right_.getOrigin().getX(),tcp_to_wrist_tf_right_.getOrigin().getX(),tcp_to_wrist_tf_right_.getOrigin().getX()-0.1));
    tcp_to_wrist_tf_right_.setRotation(tcp_to_wrist_tf_.getRotation());
  }
 
  static tf::TransformBroadcaster br;
  br.sendTransform(tf::StampedTransform(tcp_to_wrist_tf_right_, ros::Time::now(), "world", "newTCP"));

  res.success = true;
  return true;
  
  //if(arm_name=="arm_left")
  //  tcp_to_wrist_tf_left_.setOrigin(tf::Vector3(tcp_to_wrist_tf_left_.getOrigin().getX(),tcp_to_wrist_tf_left_.getOrigin().getX(),tcp_to_wrist_tf_left_.getOrigin().getX()-0.1));
  
}
           


bool MotionExecutor::restoreTCP(lenny_msgs::RestoreTCP::Request & req, lenny_msgs::RestoreTCP::Response & res) 
{
  ROS_DEBUG_STREAM("Received request to RESTORE TCP" );

  ///TODO: do it for the other groups and change the move_home_fuction too
  if(req.arm_name=="arm_right")
  {
    tcp_to_wrist_tf_right_.setOrigin(tf::Vector3(tcp_to_wrist_tf_right_.getOrigin().getX(),tcp_to_wrist_tf_right_.getOrigin().getX(),tcp_to_wrist_tf_right_.getOrigin().getX()+0.1));
    tcp_to_wrist_tf_right_.setRotation(tcp_to_wrist_tf_.getRotation());
  }
 
  static tf::TransformBroadcaster br;
  br.sendTransform(tf::StampedTransform(tcp_to_wrist_tf_right_, ros::Time::now(), "world", "newTCP"));

  res.success = true;
  return true;
  
  //if(arm_name=="arm_left")
  //  tcp_to_wrist_tf_left_.setOrigin(tf::Vector3(tcp_to_wrist_tf_left_.getOrigin().getX(),tcp_to_wrist_tf_left_.getOrigin().getX(),tcp_to_wrist_tf_left_.getOrigin().getX()-0.1));
  
}









/*

bool MotionExecutor::moveToCalibrateShelf(apc16delft_msgs::MoveToCalibrateShelf::Request &, apc16delft_msgs::MoveToCalibrateShelf::Response & res) {
	current_moveit_group_ = &tool0_group_;
	current_moveit_group_->clearPoseTargets ();
	current_moveit_group_->setPlannerId("RRTConnectkConfigDefault");

	current_moveit_group_->allowReplanning(true);
	current_moveit_group_->setNumPlanningAttempts(5);

	current_moveit_group_->setNamedTarget("calibrate_shelf");

	moveit::planning_interface::MoveGroup::Plan motion_plan;
	if (current_moveit_group_->plan(motion_plan) != 1) {
		res.error.code    = 1; // TODO
		res.error.message = "No motion plan found";
		return false;
	}

	if (!current_moveit_group_->execute(motion_plan)) {
		res.error.code    = 1; // TODO
		res.error.message = "Failed to execute motion plan.";
		return false;
	}

	return true;
}

bool MotionExecutor::moveToCalibrateTote(apc16delft_msgs::MoveToCalibrateTote::Request &, apc16delft_msgs::MoveToCalibrateTote::Response & res) {
	current_moveit_group_ = &tool0_group_;
	current_moveit_group_->clearPoseTargets ();
	current_moveit_group_->setPlannerId("RRTConnectkConfigDefault");

	current_moveit_group_->allowReplanning(true);
	current_moveit_group_->setNumPlanningAttempts(5);

	current_moveit_group_->setNamedTarget("calibrate_tote");

	moveit::planning_interface::MoveGroup::Plan motion_plan;
	if (current_moveit_group_->plan(motion_plan) != 1) {
		res.error.code    = 1; // TODO
		res.error.message = "No motion plan found";
		return false;
	}

	if (!current_moveit_group_->execute(motion_plan)) {
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
