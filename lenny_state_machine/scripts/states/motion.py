import rospy
import roslib
import smach

from lenny_msgs.srv import *

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
    ROS_INFO(' XXXMOVE HOME PICK TOOLS')
    try:
          move_to_home = rospy.ServiceProxy('/motion_executor/move_to_home', MoveToHome)
          req = MoveToHomeRequest()
          req.pose_name = "HOME_TOOL"
          req.move_group = "torso"
        
          resp = move_to_home(req)
          if(resp==true):
              #return 'pick'
            else:
              #return 'error'  
      
    
    except rospy.ServiceException as exv:
           rospy.loginfo('Service did not process request: ', + srt(exec))     
      
    if self.preempt_requested():
      self.service_preempt()
      #return 'error'
    else:
      #return 'pick'

          
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
      return 'success'

      

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
     
        
        
      
        
        
