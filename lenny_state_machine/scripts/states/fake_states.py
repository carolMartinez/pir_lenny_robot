import rospy
import roslib
import smach


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
      
        
      

class WaitFake(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['need_tool','error'])

  def execute(self, userdata):
    rospy.loginfo('FAKE WAIT STATE')
    rospy.sleep(5)
    if self.preempt_requested():
      self.service_preempt()
      return 'error'
    else:
      return 'need_tool'
      
        
        
