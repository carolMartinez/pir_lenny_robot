import rospy
import roslib
import smach
import time
import moveit_commander
import tf2_ros


from lenny_msgs.srv import *
from pir_vision_msgs.srv import *




class DetectBottlesToPick(smach.State):
  def __init__(self,dataSM):
    smach.State.__init__(self, outcomes=['pick_bottle','error', 'need_tool','leave_tool'],
     output_keys=['tool_name_output'])
    self.dataSM = dataSM

  def execute(self, userdata):
    
    #if I read a ros param and says that the robot has the tool it also
    #says in which arm
    #FAKE MESSAGE THAT WILL BE PASSED BY WILSON SERVICE TaskDefinition.msg
    class TaskDefinition:
      pick_pose = geometry_msgs.msg.Pose()
      place_pose = geometry_msgs.msg.Pose()
      object_type = "PET" # "HDPE-COLOR etc"
      task_for_arm = "arm_left" #"arm_left"
      tool_type = "gripper" #"vacuum" 
    
    robot_config = "single" #"dual"
    object1 = TaskDefinition()
    object2 = TaskDefinition()
    
    ## I will fill the data the same way wilson will pass it
    object1.pick_pose = geometry_msgs.msg.Pose()
    object1.place_pose = geometry_msgs.msg.Pose()
    object1.object_type = "PET" # "HDPE-COLOR etc"
    object1.task_for_arm = "arm_left" #"arm_left"
    object1.tool_type = "vacuum" #"vacuum" 
    
    object2.pick_pose = geometry_msgs.msg.Pose()
    object2.place_pose = geometry_msgs.msg.Pose()
    object2.object_type = "PET" # "HDPE-COLOR etc"
    object2.task_for_arm = "arm_right" #"arm_left"
    object2.tool_type = "vacuum" #"vacuum" 
    
    
    ##For testing place this at the begining of the state machine.
    self.dataSM.tool_in_arm = "arm_right"
    self.dataSM.tool_name = "tool_2"

    ## This data will be filled out when wilson message arrive
    self.dataSM.task_for_arm = object1.task_for_arm
    
    
    self.dataSM.task_tool_type = object1.tool_type


    tool_in_arm = self.dataSM.tool_in_arm
    tool_name = self.dataSM.tool_name
    
   
    if (robot_config == "single"):
      
      if(self.dataSM.task_for_arm == "arm_right"):
        self.planning_group_tool = "gripper_3f"
      
      if(self.dataSM.task_for_arm == "arm_left"):
        self.planning_group_tool = "gripper_2f"
        
    
    
      #LEo el primer dato cada uno de los dats que viene de wilson.
      if(self.dataSM.task_tool_type == "vacuum") and (tool_in_arm == " "):
        #It means we require the tool
        self.dataSM.change_tool_hand = False
        self.dataSM.planning_group = self.dataSM.task_for_arm
        return 'need_tool' 
     
      if(self.dataSM.task_tool_type == "vacuum") and (tool_in_arm == self.dataSM.task_for_arm):
        #It means we do not need to pick up the tool it is already in its hand
        self.dataSM.planning_group = self.dataSM.task_for_arm
        self.dataSM.change_tool_hand = False
        return 'pick_bottle' 
        
      if(self.dataSM.task_tool_type == "vacuum") and (tool_in_arm != self.dataSM.task_for_arm) and (tool_in_arm != " "):
        #It means we need the vacuum but it is in the other hand. So we have to leave the tool
        #and then pick it up again
        self.dataSM.change_tool_hand = True
        self.dataSM.planning_group = self.dataSM.tool_in_arm
        return 'leave_tool'
    
    
    
    
    #if (self.dataSM.tool_in_arm == " " and tool_type == "vacuum"):
    #  self.dataSM.planning_group = "arm_right"
    #  return 'need_tool' 
   
   
    #if (self.dataSM.tool_in_arm == "right"):
    #  planning_group = "arm_right"
    #  return 'leave_tool'
       
   
      
    #rospy.wait_for_service('/pir_vision_utils_rviz/get_pose_object')
    
    #try:
    #      #Moving ARMS to a HOME position to pick up the tool
    #      detect_tool_pose = rospy.ServiceProxy('/pir_vision_utils_rviz/get_pose_object', PirPositionObject)
    #      #detect_tool_pose = rospy.ServiceProxy('/pir_vision_utils_rviz/get_pose_tcp_object', PirPositionObject)
          
    #      req = PirPositionObjectRequest()
    #      req.object_type = "tool"
    #      req.object_name = "tool_2"
  
    #      resp = detect_tool_pose(req)
    #      print(resp.status)
    #      #TODO: change Sucesfull for successful
    #      if(resp.status=='sucesfull'):
    #        #userdata.tool_pose_output = geometry_msgs.msg.Pose()
    #        userdata.tool_pose_output = resp.object_pose
    #        userdata.tool_pose_output.orientation.w=0.0;
    #        #userdata.tool_pose_output.orientation = resp.object_pose.orientation
            
    #        return 'success'
    #      else:
    #        return 'error' 

    #except rospy.ServiceException as exc:
    #       rospy.loginfo('Service did not process request: %s', exc)  
    #       return 'error'   
      
    if self.preempt_requested():
      self.service_preempt()
      return 'error'
    else:
      return 'success'
  
  
  
      
        
        
