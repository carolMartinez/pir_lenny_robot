import rospy
import roslib
import smach
import time
import moveit_commander
import tf2_ros

from lenny_msgs.srv import *


class MoveHomePickBottles(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['success','error'],
                                input_keys=['print_input'],
                                output_keys=['print_output'])

  def execute(self, userdata):
    rospy.loginfo('MOVE HOME PICK BOTTLES STATE')
    rospy.wait_for_service('/motion_executor/move_to_predefined_pose')
    
    try:
          #Moving ARMS to a HOME position to pick up the tool
          move_to_wait = rospy.ServiceProxy('/motion_executor/move_to_predefined_pose', MoveToPredefinedPose)
          resp = move_to_wait("PICK_WAIT","arms")
           
          #Moving TORSO to HOME
          #Using movegroup python interface to move only the toros.
          #this because the go to pose goal did not work in c++
          move_group = moveit_commander.MoveGroupCommander('torso')
          joint_goal = move_group.get_current_joint_values()
          joint_goal[0] = 0
          plan=move_group.go(joint_goal,wait=True)
          
          if(plan):
            return 'success'
          else:
            return 'error' 
          
       
    except rospy.ServiceException as exc:
           rospy.loginfo('Service did not process request: %s', exc)  
           return 'error'   
      
    if self.preempt_requested():
      self.service_preempt()
      return 'error'
    


class InitPickBottles(smach.State):
  def __init__(self,dataSM):
    smach.State.__init__(self, outcomes=['success','error'])
    self.dataSM = dataSM

  def execute(self, userdata):
    
    rospy.loginfo('INIT PICK BOTTLES')
    
    self.dataSM.attach_object_name = "bottle_1"
    
    return 'success'
      
    if self.preempt_requested():
      self.service_preempt()
      return 'error'
    



        
      
        
        

        
      
        
        
