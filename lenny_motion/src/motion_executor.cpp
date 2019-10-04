#include "motion_executor.h"
#include <trajectory_msgs/JointTrajectory.h>

#include <string>
#include <cmath>
#include <limits>
#include <vector>
#include <string>





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
	
  //+++++ SERVICES
	move_to_predefined_pose_ = node_handle_.advertiseService("move_to_predefined_pose", &MotionExecutor::moveToPredefinedPose, this);
  create_pick_movements_ = node_handle_.advertiseService("create_pick_movements", &MotionExecutor::createPickMovements, this);
  execute_coarse_motion_ = node_handle_.advertiseService("execute_coarse_motion", &MotionExecutor::executeCoarseMotion, this);
  plan_coarse_motion_ = node_handle_.advertiseService("plan_coarse_motion", &MotionExecutor::planCoarseMotion, this);
  plan_execute_fine_motion_ = node_handle_.advertiseService("plan_execute_fine_motion", &MotionExecutor::planExecuteFineMotion, this);
  extendTCP_ = node_handle_.advertiseService("extendTCP", &MotionExecutor::extendTCP, this);
  restoreTCP_ = node_handle_.advertiseService("restoreTCP", &MotionExecutor::restoreTCP, this);
  motion_plan_client = node_handle_.serviceClient<moveit_msgs::GetMotionPlan>("plan_kinematic_path");
 
  
  
  //+++++ READ PARAMETERS
	node_handle_.getParam("approach_distance",approach_distance_);
  node_handle_.getParam("retreat_distance",retreat_distance_);
  node_handle_.getParam("trajectory_velocity_scaling",trajectory_velocity_scaling_);
	node_handle_.getParam("planner_id",planner_id_);
  node_handle_.getParam("planning_time",planning_time_);
  node_handle_.getParam("planning_attempts",planning_attemps_);
  node_handle_.getParam("position_tolerance",position_tolerance_);
  node_handle_.getParam("orientation_tolerance",orientation_tolerance_);
  node_handle_.getParam("goal_tolerance", goal_tolerance_);
  node_handle_.getParam("tcp_link_name_arm_left", tcp_link_name_arm_left_);
  node_handle_.getParam("tcp_link_name_arm_right", tcp_link_name_arm_right_);
  node_handle_.getParam("wrist_link_name_arm_right", wrist_link_name_arm_right_);
  node_handle_.getParam("wrist_link_name_arm_left", wrist_link_name_arm_left_);
  node_handle_.getParam("world_frame_id", world_frame_id_);
  
  
  
  
  
	// Fill cache with all yaml files from directory.
	//current_moveit_group_->setMaxVelocityScalingFactor(trajectory_velocity_scaling_);
	//rs_ = current_moveit_group_->getCurrentState();
  
  
  
  //Initialize the transformation from TCP to Wrist
  //This transformations are used to estimate the pick poses, based on the object's position
  
  tf::TransformListener listener;
  try
  {
    ///TODO: this is a fixed transformation, this has to be generic too
	  listener.waitForTransform(tcp_link_name_arm_right_, wrist_link_name_arm_right_,ros::Time::now(),ros::Duration(3.0f));
    listener.lookupTransform(tcp_link_name_arm_right_, wrist_link_name_arm_right_, ros::Time(0), tcp_to_wrist_tf_right_);
  }
  catch (tf::TransformException &ex) 
  {
    ROS_WARN("%s",ex.what());
    ros::Duration(1.0).sleep();
  }
  
  try
  {
    ///TODO: this is a fixed transformation, this has to be generic too
	  listener.waitForTransform(tcp_link_name_arm_left_, wrist_link_name_arm_left_,ros::Time::now(),ros::Duration(3.0f));
    listener.lookupTransform(tcp_link_name_arm_left_, wrist_link_name_arm_left_, ros::Time(0), tcp_to_wrist_tf_left_);
  }
  catch (tf::TransformException &ex) 
  {
    ROS_WARN("%s",ex.what());
    ros::Duration(1.0).sleep();
  }


  ///INITIALIZE planning parameters for all groups
  setMoveitGroupParams(&arms_group_);
  setMoveitGroupParams(&arm_right_group_);
  setMoveitGroupParams(&arm_left_group_);
	setMoveitGroupParams(&torso_group_);
  setMoveitGroupParams(&sda10f_group_);
  
      
}

bool MotionExecutor::setMoveitGroupParams(moveit::planning_interface::MoveGroupInterface * moveit_group)
{
  moveit_group->setPlannerId(planner_id_);
	moveit_group->allowReplanning(true);
	moveit_group->setNumPlanningAttempts(planning_attemps_);
	moveit_group->setMaxVelocityScalingFactor(trajectory_velocity_scaling_);
  moveit_group->setPlanningTime(planning_time_);
  moveit_group->setGoalTolerance(goal_tolerance_);
  return true;
	      
}


bool MotionExecutor::moveToPredefinedPose(lenny_msgs::MoveToPredefinedPose::Request & req, lenny_msgs::MoveToPredefinedPose::Response & res)
{
	ROS_DEBUG_STREAM("Received request to move to Predefined POSE " );
  
  //To set the planning group
  if(req.move_group=="arms")
		current_moveit_group_ = &arms_group_;
	if(req.move_group=="arm_right")
		current_moveit_group_ = &arm_right_group_;
  if(req.move_group=="arm_left")
		current_moveit_group_ = &arm_left_group_;
  if(req.move_group=="sda10f")
		current_moveit_group_ = &sda10f_group_;
  
	
  current_moveit_group_->setStartState(*current_moveit_group_->getCurrentState());

  //Setting up the pose target according to the ones available in the SDRF of the moveit package
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
	
  //Object pose
  object_pose_=req.object_pose;
  
 
  tf::poseMsgToTF(object_pose_,world_to_object_tf);
         
  tf::Vector3 object_position(object_pose_.position.x, object_pose_.position.y, object_pose_.position.z);
	
  world_to_tcp_tf.setOrigin(object_position);

 
  /* Setting tcp orientation
	   * Inverting the approach direction so that the tcp points towards the object instead of
	   * away from it.*/
  world_to_tcp_tf.setRotation(world_to_object_tf.getRotation());


  //Create 3 poses, approach, pick, and retreat poses
  geometry_msgs::Pose approach_pose, target_pose, retreat_pose;
  std::vector<geometry_msgs::Pose> poses, wrist_pick_poses;

  
  // creating approach pose by applying a translation along +z (the Z world reference frame is pointing upwards)
  tf::poseTFToMsg(Transform(Quaternion::getIdentity(),Vector3(0,0,approach_distance_))*world_to_tcp_tf,approach_pose);
   
  // converting target pose
  tf::poseTFToMsg(world_to_tcp_tf,target_pose);

 // creating end pose by applying a translation along +z by retreat distance
  tf::poseTFToMsg(Transform(Quaternion::getIdentity(),Vector3(0,0,0.2))*world_to_tcp_tf,retreat_pose);
  
  ///TODO: do something if waypoint is not reachable. Not sure if it is here where we will have to check this.   
  poses.clear();
  poses.push_back(approach_pose);
  poses.push_back(target_pose);
  poses.push_back(retreat_pose);

  //Convert poses from world coordinate frame to wrist
  
  //We need the trasnformation between wrist and tcp this is fixed
  //This depends on the move_group
  tf::StampedTransform tcp_to_wrist_tf;
  if (req.move_group == "arm_right")
    tcp_to_wrist_tf=tcp_to_wrist_tf_right_; 
  if (req.move_group == "arm_left")
    tcp_to_wrist_tf=tcp_to_wrist_tf_left_; 
    
  
	std::vector<geometry_msgs::Pose> wrist_poses;
	wrist_poses.resize(poses.size());

	//Applying transform to each tcp poses
	tf::Transform world_to_wrist_tf, world_to_tcp_tf_sec;

	for(unsigned int i = 0; i < poses.size(); i++)
	{
		//tf::poseMsgToTF(poses[i],world_to_tcp_tf_sec);
		
		tf::poseMsgToTF(poses[i],world_to_tcp_tf_sec);
		//std::cout << world_to_tcp_tf_sec.translation << std::endl;
		tf::Transform req_to_target;
		world_to_wrist_tf = world_to_tcp_tf_sec*tcp_to_wrist_tf;
    
    //Saving the transformations. A pose wrt wrist link   
		tf::poseTFToMsg(world_to_wrist_tf,wrist_poses[i]);
		
	}


    
	res.success = true;
	res.robot_movements = wrist_poses;
    	
	return true;
  
}




bool MotionExecutor::executeCoarseMotion(lenny_msgs::ExecuteCoarseMotion::Request & req, lenny_msgs::ExecuteCoarseMotion::Response & res) 
{
  ROS_DEBUG_STREAM("Received request to execute coarse motion " );

  //To set the planning group
  if(req.move_group=="arms")
		current_moveit_group_ = &arms_group_;
	if(req.move_group=="arm_right")
		current_moveit_group_ = &arm_right_group_;
  if(req.move_group=="arm_left")
		current_moveit_group_ = &arm_left_group_;
  if(req.move_group=="sda10f")
		current_moveit_group_ = &sda10f_group_;
  
	
 
	moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	
	my_plan.trajectory_.joint_trajectory = req.coarse_trajectory;

		
	moveit_msgs::RobotState robot_state;
		
	const robot_state::JointModelGroup *joint_model_group = current_moveit_group_->getCurrentState()->getJointModelGroup(req.move_group);

	// constructing motion plan goal constraints
	current_moveit_group_->setStartState(*current_moveit_group_->getCurrentState());
  
       
  current_moveit_group_->execute(my_plan);
  
  
  bool stop;

  //This function waits for the robot to stop. With it we avoid the problem
  //of the error: "motion does not start at current position.."
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

	 //To set the planning group
  if(req.move_group=="arms")
		current_moveit_group_ = &arms_group_;
	if(req.move_group=="arm_right")
  {
		current_moveit_group_ = &arm_right_group_;
    wrist_link_name_=wrist_link_name_arm_right_;
  }
  if(req.move_group=="arm_left")
  {
		current_moveit_group_ = &arm_left_group_;
    wrist_link_name_=wrist_link_name_arm_left_;
  }
  if(req.move_group=="sda10f")
		current_moveit_group_ = &sda10f_group_;
  
	

 /// kinematic_state_: it is the current kinematic configuration of the robot
  kinematic_state_ = moveit::core::RobotStatePtr(current_moveit_group_->getCurrentState());
  kinematic_state_->setToDefaultValues();
  
  const robot_model::RobotModelConstPtr &kmodel = kinematic_state_->getRobotModel();
  joint_model_group_ = kmodel->getJointModelGroup(req.move_group);
  

	moveit_msgs::RobotState robot_state;		
	const robot_state::JointModelGroup *joint_model_group = current_moveit_group_->getCurrentState()->getJointModelGroup(req.move_group);

  geometry_msgs::Pose target_pose;
	target_pose=req.target_pose;

  geometry_msgs::PoseStamped p;
	

  //Check reachability
  bool foundIK = false;
  foundIK = checkWayPointReachability(target_pose);
  
  //if(foundIK)
  //{ 
      /*
      p.header.frame_id = world_frame_id_;
      p.pose = target_pose;
      moveit_msgs::Constraints pose_goal = kinematic_constraints::constructGoalConstraints(wrist_link_name_,p,position_tolerance_,
      orientation_tolerance_);

      // creating motion plan request
      moveit_msgs::GetMotionPlan motion_plan;      
      moveit_msgs::MotionPlanRequest &plan_req = motion_plan.request.motion_plan_request;
      moveit_msgs::MotionPlanResponse &plan_res = motion_plan.response.motion_plan_response;
      
      plan_req.start_state = robot_state;
      plan_req.start_state.is_diff = true;
           
      plan_req.planner_id = planner_id_;
      plan_req.group_name = req.move_group;
      plan_req.goal_constraints.push_back(pose_goal);
      plan_req.allowed_planning_time = planning_time_;
      plan_req.num_planning_attempts = planning_attemps_;
      plan_req.max_velocity_scaling_factor = trajectory_velocity_scaling_;
      */
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

	if(req.move_group=="arms")
		current_moveit_group_ = &arms_group_;
	if(req.move_group=="arm_right")
		current_moveit_group_ = &arm_right_group_;
  if(req.move_group=="arm_left")
		current_moveit_group_ = &arm_left_group_;
  if(req.move_group=="sda10f")
		current_moveit_group_ = &sda10f_group_;
    
 //Move faster in cartesian space
 current_moveit_group_->setMaxVelocityScalingFactor(0.8);
 
   
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

  
 
  //Initialize transformation from tcp to wrist
  tf::TransformListener listener;
  
  //TODO: we need to have one tcp_to_wrist_tf per arm
  try
  {
    ///TODO: do it for the other groups and change the move_home_fuction too
    if(req.arm_name=="arm_right")
    {
      //tcp_to_wrist_tf_right_.setOrigin(tf::Vector3(tcp_to_wrist_tf_right_.getOrigin().getX(),tcp_to_wrist_tf_right_.getOrigin().getX(),tcp_to_wrist_tf_right_.getOrigin().getX()-0.1));
      //tcp_to_wrist_tf_right_.setRotation(tcp_to_wrist_tf_.getRotation());
    
      ///TODO: this is a fixed transformation, this has to be generic too
      listener.waitForTransform("gripper3f_tcp_tool", "arm_right_link_7_t",ros::Time::now(),ros::Duration(3.0f));
      listener.lookupTransform("gripper3f_tcp_tool", "arm_right_link_7_t", ros::Time(0), tcp_to_wrist_tf_);
      
    }
    
    if(req.arm_name=="arm_left")
    {
      //tcp_to_wrist_tf_right_.setOrigin(tf::Vector3(tcp_to_wrist_tf_right_.getOrigin().getX(),tcp_to_wrist_tf_right_.getOrigin().getX(),tcp_to_wrist_tf_right_.getOrigin().getX()-0.1));
      //tcp_to_wrist_tf_right_.setRotation(tcp_to_wrist_tf_.getRotation());
    
      ///TODO: this is a fixed transformation, this has to be generic too
      listener.waitForTransform("gripper2f_tcp_tool", "arm_left_link_7_t",ros::Time::now(),ros::Duration(3.0f));
      listener.lookupTransform("gripper2f_tcp_tool", "arm_left_link_7_t", ros::Time(0), tcp_to_wrist_tf_);
      
    }  

  }
  catch (tf::TransformException &ex) 
  {
    ROS_WARN("%s",ex.what());
    ros::Duration(1.0).sleep();
  }


  //static tf::TransformBroadcaster br;
  //br.sendTransform(tf::StampedTransform(tcp_to_wrist_tf_right_, ros::Time::now(), "world", "newTCP"));

  res.success = true;
  return true;
  
  //if(arm_name=="arm_left")
  //  tcp_to_wrist_tf_left_.setOrigin(tf::Vector3(tcp_to_wrist_tf_left_.getOrigin().getX(),tcp_to_wrist_tf_left_.getOrigin().getX(),tcp_to_wrist_tf_left_.getOrigin().getX()-0.1));
  
}
           


bool MotionExecutor::restoreTCP(lenny_msgs::RestoreTCP::Request & req, lenny_msgs::RestoreTCP::Response & res) 
{
  ROS_DEBUG_STREAM("Received request to RESTORE TCP" );
 
  //Initialize transformation from tcp to wrist
  tf::TransformListener listener;
  
  //TODO: we need to have one tcp_to_wrist_tf per arm
  try
  {
    ///TODO: do it for the other groups and change the move_home_fuction too
    if(req.arm_name=="arm_right")
    {
      tcp_to_wrist_tf_=tcp_to_wrist_tf_right_;
     
    }
    
    if(req.arm_name=="arm_left")
    {
      tcp_to_wrist_tf_=tcp_to_wrist_tf_left_;
    }  

  }
  catch (tf::TransformException &ex) 
  {
    ROS_WARN("%s",ex.what());
    ros::Duration(1.0).sleep();
  }
















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

//} // namespace

int main(int argc, char ** argv)
{
	ros::init(argc, argv, "motion_executor");
	
	MotionExecutor motion_executor;

	ros::Rate loop_rate(1000);
	while(ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
}
