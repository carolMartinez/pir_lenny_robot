import rospy
import roslib
import smach
import time
import moveit_commander
import tf2_ros

from lenny_msgs.srv import *


class MoveHomePlaceBottles(smach.State):
  def __init__(self,dataSM):
    smach.State.__init__(self, outcomes=['success','error'],
                                input_keys=['print_input'],
                                output_keys=['print_output'])
    self.dataSM = dataSM
    
  def execute(self, userdata):
    rospy.loginfo('MOVE HOME PLACE BOTTLES STATE')
    rospy.wait_for_service('/motion_executor/move_to_predefined_pose')
    
    try:
          #Moving ARM to a HOME-PLACE position to place the bottle
          move_to_wait = rospy.ServiceProxy('/motion_executor/move_to_predefined_pose', MoveToPredefinedPose)
          resp = move_to_wait("OVERHEAD_WAIT",self.dataSM.task_for_arm)
          
          time.sleep(2)
          #Moving TORSO to HOME
          #Using movegroup python interface to move only the toros.
          #this because the go to pose goal did not work in c++
          move_group = moveit_commander.MoveGroupCommander('sda10f')
          joint_goal = move_group.get_current_joint_values()
          joint_goal[0] = 0
          plan=move_group.go(joint_goal,wait=True)
          #move_group.stop()
          #move_group.clear_pose_targets()
          
          #move_to_wait = rospy.ServiceProxy('/motion_executor/move_to_predefined_pose', MoveToPredefinedPose)
            
          #resp = move_to_wait("HOME","torso")
          move_group.stop()
          move_group.clear_pose_targets()
          time.sleep(2)
          
          if(resp.success):
            return 'success'
          else:
            return 'error' 
          
          
            
            
          if(resp.success):
            return 'success'
          else:
            return 'error' 
          
       
    except rospy.ServiceException as exc:
           rospy.loginfo('Service did not process request: %s', exc)  
           return 'error'   
      
    if self.preempt_requested():
      self.service_preempt()
      return 'error'
    


class RotateTorsoPlaceBottles(smach.State):
  def __init__(self,dataSM):
    smach.State.__init__(self, outcomes=['success','error'])
    self.dataSM = dataSM
    
  def execute(self, userdata):
    
    #Rotate torso to to place the bottle.
    #This because the bin is difficult to reach.
    print("Picking Bottle: %s",self.dataSM.bottle_type)
    
    if ((self.dataSM.bottle_type == "WHITE HDPE") or (self.dataSM.bottle_type == "PET")  ):
      if (self.dataSM.planning_group_robot == "arm_left" or self.dataSM.planning_group_robot == "arm_right"):
    
        rospy.loginfo('MOVE TORSO PLACE BOTTLES STATE')
        rospy.wait_for_service('/motion_executor/move_to_predefined_pose')
        
        try:
              #move_group = moveit_commander.MoveGroupCommander('torso')
              #joint_goal = move_group.get_current_joint_values()
              
              move_to_wait = rospy.ServiceProxy('/motion_executor/move_to_predefined_pose', MoveToPredefinedPose)
              
              if (self.dataSM.bottle_type == "WHITE HDPE") and (self.dataSM.planning_group_robot == "arm_left"):
                resp = move_to_wait("LEFT_ROTATION_POSE","sda10f")
                if(resp.success):
                  return 'success'
                else:
                  return 'error' 

                #joint_goal[0] = 0.6
              else:
              
                if (self.dataSM.bottle_type == "PET") and (self.dataSM.planning_group_robot == "arm_right"):
                  resp = move_to_wait("RIGHT_ROTATION_POSE","sda10f")
                  if(resp.success):
                    return 'success'
                  else:
                    return 'error' 
                else:
                  return 'success'
                    
                #joint_goal[0] = -0.6
              
              #plan=move_group.go(joint_goal,wait=True)
              #move_group.stop()
              #move_group.clear_pose_targets()
              time.sleep(2)
              
              
           
        except rospy.ServiceException as exc:
               rospy.loginfo('Service did not process request: %s', exc)  
               return 'error'  
               
    else:
      return 'success'
      
    if self.preempt_requested():
      self.service_preempt()
      rospy.loginfo('Preempt Service did not process request: %s', exc) 
      return 'error'
    

        
      
        
        
