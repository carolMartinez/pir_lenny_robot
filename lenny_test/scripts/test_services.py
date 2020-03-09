#!/usr/bin/env python

import rospy
import roslib
import actionlib

from lenny_msgs.srv import *
from pir_vision_msgs.srv import *
        
def main():
  
  rospy.init_node('lenny_services_test')
  
  error=0
  tool_pose = geometry_msgs.msg.Pose()
  approach_pose = geometry_msgs.msg.Pose()
  
  trajectory = trajectory_msgs.msg.JointTrajectory()
  
           
  ##DETECT TOOL
  rospy.wait_for_service('/pir_vision_utils_rviz/get_pose_object')
  try:
    detect_tool_pose = rospy.ServiceProxy('/pir_vision_utils_rviz/get_pose_object', PirPositionObject)
   
    req = PirPositionObjectRequest()
    req.object_type = "tool"
    req.object_name = "tool_2"

    resp = detect_tool_pose(req)
    print(resp.status)
    #TODO: change Sucesfull for successful
    if(resp.status=='sucesfull'):
      tool_pose = resp.object_pose
      tool_pose.orientation.w=0.0;
      #tool_pose.orientation.x=0.0;
      #tool_pose.orientation.y=3.14;
      #tool_pose.orientation.z=0.0;
      
      #tool_pose.position.x=-0.27;
      #tool_pose.position.y=0.5;
      #tool_pose.position.z=0.99;
      
      print("Object pose succesfully retreaved")
    else:
      error=1
      print("Error retreaving Object pose") 

  except rospy.ServiceException as exc:
     rospy.loginfo('pyr_vision_system Service did not process request: %s', exc)  
     error=1       
 
 
 
  if(error==0):
    
    rospy.loginfo('CREATE PICK MOVES')
    ##CREATE PICK MOVE
    rospy.wait_for_service('/motion_executor/create_pick_movements')
    
    try:
      create_pick_movements = rospy.ServiceProxy('/motion_executor/create_pick_movements', CreatePickMovements)
                  
      resp = create_pick_movements(tool_pose)
      approach_pose = resp.robot_movements[0]
      
      if(resp.success):
        print("Approach pose --> success")
      else:
        error=1
        print("Approach pose --> ERROR")
       
    except rospy.ServiceException as exc:
      error=1
      rospy.loginfo('Create PICK MOVE service did not process request: %s', exc)  

    
    if(error==0):
      ##PLAN 
      rospy.loginfo('PLAN COARSE MOTION')
      rospy.wait_for_service('/motion_executor/plan_coarse_motion')
      
      try:
        plan_coarse_motion=rospy.ServiceProxy('/motion_executor/plan_coarse_motion',PlanCoarseMotion)

        req = PlanCoarseMotionRequest()
        req.target_pose.position = approach_pose.position
        req.target_pose.orientation = approach_pose.orientation
        
        req.move_group = "arm_right"
        
        resp = plan_coarse_motion(req)

        trajectory = resp.coarse_trajectory
        
        if(resp.success):
           print("Plan --> success")
        else:
          error=1
          print("Plan --> error") 
          
      except rospy.ServiceException as exc:
        rospy.loginfo('Plan Service did not process request: %s', exc)  
        return 'error'  
                
      
      
      if(error==0):
        ##MOVE 
        rospy.wait_for_service('/motion_executor/execute_coarse_motion')
        execute_coarse_move=rospy.ServiceProxy('/motion_executor/execute_coarse_motion',ExecuteCoarseMotion)
        req = ExecuteCoarseMotionRequest()
        req.coarse_trajectory = trajectory
        req.move_group = "arm_right"
      
        resp = execute_coarse_move(req)
      
        rospy.loginfo('CREATE COARSE MOVES')
        
        if(resp.success):
          print("Execute --> success")
        else:
          print("Execute --> error") 

  
  return  
  
    

if __name__ == '__main__':
    main()
