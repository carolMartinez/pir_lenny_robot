#!/usr/bin/env python

import rospy
import roslib
import actionlib

from lenny_msgs.srv import *

from lenny_msgs.srv import ExecuteCoarseMove, ExecuteCoarseMoveRequest

        
def main():
  
  rospy.init_node('lenny_services_test')

  rospy.wait_for_service('/motion_executor/execute_coarse_move')
  
  
  box_pose = geometry_msgs.msg.Pose()
 

  box_pose.orientation.w = -0.011
  box_pose.orientation.x =  0.7090
  box_pose.orientation.y = 0.7050
  box_pose.orientation.z =  0.01055
  box_pose.position.x =  0.17
  box_pose.position.y = 0.56
  box_pose.position.z =  0.61

  #req = ExecuteCoarseMoveRequest()

  
  #req.target_pose = box_pose
  
  #req.move_group = "arms"
  
  req = ExecuteCoarseMoveRequest(box_pose,"arms")
  
  execute_coarse_move=rospy.ServiceProxy('/motion_executor/execute_coarse_move',ExecuteCoarseMove)
  resp = execute_coarse_move(req)
  print(resp.success)
     
  if (resp):
    rospy.loginfo('CREATE PICK MOVES')
  else:
    rospy.loginfo('error')
  
  return  
  
    

if __name__ == '__main__':
    main()
