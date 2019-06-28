import rospy
import roslib
import smach
import moveit_commander
import tf2_ros

from lenny_msgs.srv import *
from lenny_msgs.srv import ExecuteCoarseMotion


#from tf import TransformListener

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
    rospy.wait_for_service('/motion_executor/move_to_home')
    
    try:
          #Moving ARMS to a HOME position to pick up the tool
          move_to_home = rospy.ServiceProxy('/motion_executor/move_to_home', MoveToHome)
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
    smach.State.__init__(self, outcomes=['success','error'],
    output_keys=['robot_movements_output'])
    
    
  def execute(self, userdata):
    
    rospy.wait_for_service('/motion_executor/create_pick_movements')
    
    
    create_pick_movements = rospy.ServiceProxy('/motion_executor/create_pick_movements', CreatePickMovements)

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    trans = geometry_msgs.msg.Transform()
    
    try:
       trans = tfBuffer.lookup_transform('torso_base_link', 'object_link', rospy.Time(), rospy.Duration(3.0))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
       rospy.loginfo('tf Transformation not available')


    pose = geometry_msgs.msg.Pose()
    translation = trans.transform.translation
    rotation =  trans.transform.rotation
    pose.position = geometry_msgs.msg.Point(translation.x,translation.y,translation.z)
    pose.orientation = rotation
    print(pose)     
                
    resp = create_pick_movements(pose)
    userdata.robot_movements_output = pose
    rospy.loginfo('CREATE PICK MOVES')

    if(resp.success):
        return 'success'
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


class ExecuteCoarseMotion(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['success','error'], input_keys=['robot_movements_input'])

  def execute(self, userdata):
    
    
    rospy.wait_for_service('/motion_executor/execute_coarse_move')
    
    execute_coarse_move=rospy.ServiceProxy('/motion_executor/execute_coarse_move',ExecuteCoarseMove)

    target_pose = geometry_msgs.msg.Pose()
     
    target_pose = userdata.robot_movements_input
    print(target_pose)

    TODO: this part is left for testing.. and error occurs when passing the parameters
    #resp = execute_coarse_move(target_pose,"sda10f")
    
    
    rospy.loginfo('CREATE PICK MOVES')
    
    #if(resp.success):
      
    return 'success'
    #else:
    #  return 'error' 
            
                
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
     
        
        
      
        
        
