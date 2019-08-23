import rospy
import roslib
import smach
import time
import moveit_commander
import tf2_ros


from lenny_msgs.srv import *
from pir_vision_msgs.srv import *



class DetectToolMagazine(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['success','error'],
                              input_keys=['place_pose_output'],
                              output_keys=['place_pose_output'])

  def execute(self, userdata):
   
    userdata.place_pose_output.position.x = -0.12
    userdata.place_pose_output.position.y = 0.61
    userdata.place_pose_output.position.z = 0.99
    userdata.place_pose_output.orientation.x = 0.0
    userdata.place_pose_output.orientation.y = 3.14
    userdata.place_pose_output.orientation.z = 0.0
    userdata.place_pose_output.orientation.w = 1.0
    
    return 'success'
     
  
    if self.preempt_requested():
      self.service_preempt()
      return 'error'
    else:
      return 'success'
  

  


class ReinitTCP(smach.State):
  def __init__(self):
    smach.State.__init__(self,
                outcomes = ['success', 'error'])

  def execute(self, userdata):
    rospy.loginfo('Calling Service Change TCP')
    rospy.wait_for_service('/motion_executor/restoreTCP')
    
    try:
          #Moving ARMS to a HOME position to pick up the tool
          change_TCP= rospy.ServiceProxy('/motion_executor/restoreTCP', ExtendTCP)
          
          ##TODO: Change to make it generic, arm_right should be a variable.... passed
          ##throug the State Machine.
          req = ExtendTCPRequest()
          req.arm_name = "arm_right"
          
          resp = change_TCP(req)
           
          if(resp.success):
            return 'success'
          else:
            return 'error' 
          
    except rospy.ServiceException as exc:
           rospy.loginfo('Extend_TCP service did not process request: %s', exc)  
           return 'error'   
      
    if self.preempt_requested():
      self.service_preempt()
      return 'error'
    else:
      return 'success'
        




  
class MoveHomePlaceTools(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['success','error'])

  def execute(self, userdata):
    #userdata.print_input = None
    rospy.wait_for_service('/motion_executor/move_to_pose')
    
    try:
          #Moving ARMS to a HOME position to pick up the tool
          move_to_home = rospy.ServiceProxy('/motion_executor/move_to_pose', MoveToPose)
          resp = move_to_home("PICK_WAIT","arms")
           
          #Moving TORSO to HOME
          #Using movegroup python interface to move only the toros.
          #this because the go to pose goal did not work in c++
          move_group = moveit_commander.MoveGroupCommander('sda10f')
          joint_goal = move_group.get_current_joint_values()
          joint_goal[0] = 2.9000
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
    else:
      return 'success'
  

      
        
        
