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
    rospy.wait_for_service('/motion_executor/move_to_pose')
    
    try:
          #Moving ARMS to a HOME position to pick up the tool
          move_to_home = rospy.ServiceProxy('/motion_executor/move_to_pose', MoveToHome)
          resp = move_to_home("PICK_WAIT","arms")
           
          #Moving TORSO to HOME
          #Using movegroup python interface to move only the toros.
          #this because the go to pose goal did not work in c++
          move_group = moveit_commander.MoveGroupCommander('sda10f')
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




        
      
        
        
