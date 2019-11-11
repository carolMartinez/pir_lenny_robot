import rospy
import roslib
import smach
import time
import moveit_commander
import tf2_ros


from lenny_msgs.srv import *
from pir_vision_msgs.srv import *



class DetectTool(smach.State):
  def __init__(self,dataSM):
    smach.State.__init__(self, outcomes=['success','error'], 
                              input_keys=['tool_pose_output', 'tool_name_input'],
                              output_keys=['tool_pose_output'])
    self.dataSM = dataSM

  def execute(self, userdata):
   
   #Variables required by the service that is going to detect the tool
    if (self.dataSM.fake_vision == "true"):
        tool_type = "simulation"
    else:
        tool_type = "real"
    
    #Defined from in the file
    tool_name =  self.dataSM.tool_name
    self.dataSM.attach_object_name = self.dataSM.tool_name 
   
    rospy.wait_for_service('/pir_vision_utils_rviz/get_pose_object')
    #rospy.wait_for_service('/pir_vision_utils_rviz/get_pose_tcp_object')
  
    try:
          #Moving ARMS to a HOME position to pick up the tool
          detect_tool_pose = rospy.ServiceProxy('/pir_vision_utils_rviz/get_pose_object', PirPositionObject)
          #detect_tool_pose = rospy.ServiceProxy('/pir_vision_utils_rviz/get_pose_tcp_object', PirPositionObject)
          
          
          
          req = PirPositionObjectRequest()
          req.object_type = tool_type
          req.object_name = tool_name
  
          resp = detect_tool_pose(req)
          print("after service")
          #TODO: check because nan return successfull too
          print(resp.status)
          print(resp.object_pose.position.x )
          
          
          #TODO: change Sucesfull for successful
          #if(resp.status=='sucesfull'):
          if(resp.object_pose.position.x == 0 and resp.object_pose.position.y == 0 and resp.object_pose.position.z == 0 ):
            print("ERROR: object position is 0")
            return 'error' 
          
          else:
            #userdata.tool_pose_output = geometry_msgs.msg.Pose()
            userdata.tool_pose_output = resp.object_pose
            userdata.tool_pose_output.orientation.w=0.0
            #userdata.tool_pose_output.orientation = resp.object_pose.orientation
            return 'success'

    except rospy.ServiceException as exc:
           rospy.loginfo('Service did not process request: %s', exc)  
           return 'error'   
      
    if self.preempt_requested():
      self.service_preempt()
      return 'error'
    else:
      return 'success'
  
  
  
  
class MoveHomePickTools(smach.State):
  def __init__(self, dataSM):
    smach.State.__init__(self, outcomes=['pick','error'])
    self.dataSM = dataSM

  def execute(self, userdata):
    #userdata.print_input = None
    rospy.wait_for_service('/motion_executor/move_to_predefined_pose')
    
    try:
          #Moving ARMS to a HOME position to pick up the tool
          move_to_wait = rospy.ServiceProxy('/motion_executor/move_to_predefined_pose', MoveToPredefinedPose)
          resp = move_to_wait("OVERHEAD_WAIT","arms")
           
          #Moving TORSO to HOME
          #Using movegroup python interface to move only the toros.
          #this because the go to pose goal did not work in c++
          move_group = moveit_commander.MoveGroupCommander('sda10f')
          #joint_goal = move_group.get_current_joint_values()
          move_group = moveit_commander.MoveGroupCommander('torso')
          joint_goal = move_group.get_current_joint_values()
          #joint_goal[0] = 0
          
          if(self.dataSM.planning_group_robot == "arm_right"):
            joint_goal[0] = 2.9000
          if(self.dataSM.planning_group_robot == "arm_left"):
            joint_goal[0] = 1.6000
            
            
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
  def __init__(self, dataSM):
    smach.State.__init__(self,
                outcomes = ['success', 'error'])
    self.dataSM = dataSM

  def execute(self, userdata):
    rospy.loginfo('Calling Service Change TCP')
    rospy.wait_for_service('/motion_executor/extendTCP')
    
    try:
          change_TCP= rospy.ServiceProxy('/motion_executor/extendTCP', ExtendTCP)
          
          ###TODO: change for Wilson service
          req = ExtendTCPRequest()
          req.arm_name = self.dataSM.planning_group_robot
          resp = change_TCP(req)
           
          #Update info about the tool that was picked up.
          self.dataSM.tool_in_arm = self.dataSM.planning_group_robot
           
           
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



      
        
        
