import rospy
import roslib
import smach
import time
import moveit_commander
import tf2_ros


from lenny_msgs.srv import *
from pir_vision_msgs.srv import *



class DetectTool(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['success','error'], 
                              input_keys=['tool_pose_output'],
                              output_keys=['tool_pose_output'])

  def execute(self, userdata):
   
    rospy.wait_for_service('/pir_vision_utils_rviz/get_pose_object')
    #rospy.wait_for_service('/pir_vision_utils_rviz/get_pose_tcp_object')
  
    try:
          #Moving ARMS to a HOME position to pick up the tool
          detect_tool_pose = rospy.ServiceProxy('/pir_vision_utils_rviz/get_pose_object', PirPositionObject)
          #detect_tool_pose = rospy.ServiceProxy('/pir_vision_utils_rviz/get_pose_tcp_object', PirPositionObject)
          
          req = PirPositionObjectRequest()
          req.object_type = "tool"
          req.object_name = "tool_2"
  
          resp = detect_tool_pose(req)
          print(resp.status)
          #TODO: change Sucesfull for successful
          if(resp.status=='sucesfull'):
            #userdata.tool_pose_output = geometry_msgs.msg.Pose()
            userdata.tool_pose_output = resp.object_pose
            userdata.tool_pose_output.orientation.w=0.0;
            #userdata.tool_pose_output.orientation = resp.object_pose.orientation
            
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
  
  
  
  
class MoveHomePickTools(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['pick','place','error'])

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
  


class ChangeTCP(smach.State):
  def __init__(self):
    smach.State.__init__(self,
                outcomes = ['success', 'error'])

  def execute(self, userdata):
    rospy.loginfo('Calling Service Change TCP')
    rospy.wait_for_service('/motion_executor/extendTCP')
    
    try:
          #Moving ARMS to a HOME position to pick up the tool
          change_TCP= rospy.ServiceProxy('/motion_executor/extendTCP', ExtendTCP)
          
          ##TODO: Change to make it generic, arm_right should be a variable.... passed
          ##throug the State Machine.
          req = ExtendTCPRequest()
          req.arm_name = "arm_right"
          req.distance = 0.7
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



      
        
        
