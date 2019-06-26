import rospy
import roslib
import smach
import moveit_commander

from lenny_msgs.srv import *
from tf import TransformListener

## TODO:change this class, this is only for testing purposes

class MoveRobotHome(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['done','error'],
                                input_keys=['print_input'],
                                output_keys=['print_output'])

  def execute(self, userdata):
    rospy.loginfo('MOVE ROBOT HOME STATE')
    rospy.sleep(5)
    if self.preempt_requested():
      self.service_preempt()
      return 'error'
    else:
      return 'done'
  
class MoveHomePickTools(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['pick','place','error'],
                                input_keys=['print_input'],
                                output_keys=['print_output'])

  def execute(self, userdata):
    rospy.wait_for_service('/motion_executor/move_to_home');
    
    try:
          #Moving ARMS to a HOME position to pick up the tool
          move_to_home = rospy.ServiceProxy('/motion_executor/move_to_home', MoveToHome)
          resp = move_to_home("HOME_TOOL","arms")
           
          #Moving TORSO to HOME
          #Using movegroup python interface to move only the toros.
          #this because the go to pose goal did not work in c++
          move_group = moveit_commander.MoveGroupCommander('sda10f');
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
          
class MoveHomePickBottles(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['success','error'],
                                input_keys=['print_input'],
                                output_keys=['print_output'])

  def execute(self, userdata):
    rospy.loginfo('MOVE HOME PICK BOTTLES STATE')
    rospy.sleep(5)
    if self.preempt_requested():
      self.service_preempt()
      return 'error'
    else:
      return 'success'
        
      

class WaitFake(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['success','need_tool','error'])

  def execute(self, userdata):
    rospy.loginfo('FAKE WAIT STATE')
    rospy.sleep(5)
    if self.preempt_requested():
      self.service_preempt()
      return 'error'
    else:
      return 'need_tool'

      

class WaitFake2(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['success','error'])

  def execute(self, userdata):
    rospy.loginfo('FAKE WAIT 2 STATE')
    rospy.sleep(5)
    if self.preempt_requested():
      self.service_preempt()
      return 'error'
    else:
      return 'success'


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




class CreatePickMoves(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['success','error'])
    self.tf = TransformListener()

  def execute(self, userdata):
    
    rospy.wait_for_service('/motion_executor/crete_pick_moves');
    
    
    create_pick_moves = rospy.ServiceProxy('/motion_executor/create_pick_moves', CreatePickMoves)
    
    #Listen transformation
    if self.tf.frameExists("/torso_base_link") and self.tf.frameExists("/object_link"):
            t = self.tf.getLatestCommonTime("/torso_base_link", "/object_link")
            position, quaternion = self.tf.lookupTransform("/torso_base_link", "/object_link", t)
            
            pose = geometry_msgs.Pose()
            pose.position = position
            pose.orientation = quaternion
            
            
            
            resp = create_pick_moves(pose)
          
            rospy.loginfo('CREATE PICK MOVES')
            
            if(resp):
              return 'pick'
            else:
              return 'error' 
            
                
    if self.preempt_requested():
      self.service_preempt()
      return 'error'
    else:
      return 'success'




class PlanCoarseMotion(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['success','error'])

  def execute(self, userdata):
    rospy.loginfo('PLAN COARSE MOTION')
    rospy.sleep(5)
    if self.preempt_requested():
      self.service_preempt()
      return 'error'
    else:
      return 'success'


class MoveCoarseMotion(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['success','error'])

  def execute(self, userdata):
    rospy.loginfo('MOVE COARSE MOTION')
    rospy.sleep(5)
    if self.preempt_requested():
      self.service_preempt()
      return 'error'
    else:
      return 'success'


class PlanFineMotion(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['success','error'])

  def execute(self, userdata):
    rospy.loginfo('PLAN FINE MOTION')
    rospy.sleep(5)
    if self.preempt_requested():
      self.service_preempt()
      return 'error'
    else:
      return 'success'

 
class MoveFineMotion(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['success','error'])

  def execute(self, userdata):
    rospy.loginfo('MOVE FINE MOTION')
    rospy.sleep(5)
    if self.preempt_requested():
      self.service_preempt()
      return 'error'
    else:
      return 'success'
     
        
        
      
        
        
