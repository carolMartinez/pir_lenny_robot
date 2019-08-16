import rospy
import roslib
import smach
import time
import moveit_commander
import tf2_ros

from lenny_msgs.srv import *



  
class MoveHomePickTools(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['pick','place','error'],
                                input_keys=['print_input'],
                                output_keys=['print_output'])

  def execute(self, userdata):
    rospy.wait_for_service('/motion_executor/move_to_pose')
    
    try:
          #Moving ARMS to a HOME position to pick up the tool
          move_to_home = rospy.ServiceProxy('/motion_executor/move_to_pose', MoveToPose)
          resp = move_to_home("HOME_TOOL","arms")
           
          #Moving TORSO to HOME
          #Using movegroup python interface to move only the toros.
          #this because the go to pose goal did not work in c++
          move_group = moveit_commander.MoveGroupCommander('sda10f')
          joint_goal = move_group.get_current_joint_values()
          joint_goal[0] = 1.9188
          plan=move_group.go(joint_goal,wait=True)
          
          if(plan):
            return 'pick'
          else:
            return 'error' 
          
       
      
    
    except rospy.ServiceException as exc:
           rospy.loginfo('Service did not process request: %s', exc)  
           return 'error'   
      
    if self.preempt_requested():
      self.service_preempt()
      return 'error'
    else:
      return 'pick'
  

class InitializeToolMasterSM(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['done','error'])

  def execute(self, userdata):
    rospy.loginfo('INITIALIZE TOOL MASTER STATE')
    rospy.sleep(5)
    if self.preempt_requested():
      self.service_preempt()
      return 'error'
    else:
      return 'done'



class DetectTool(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['success','error'])

  def execute(self, userdata):
    rospy.loginfo('DETECT TOOL')
    rospy.sleep(5)
    if self.preempt_requested():
      self.service_preempt()
      return 'error'
    else:
      return 'success'




        
      
        
        
