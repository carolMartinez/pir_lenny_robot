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
    
    self.dataSM.tool_in_arm = " "
    self.dataSM.tool_name = "tool_1"
    
    need_tool = 1
  
    
    if (self.dataSM.tool_in_arm == " " and need_tool == 1):
      self.dataSM.planning_group = "arm_right"
      return 'need_tool' 
   
   
    if (self.dataSM.tool_in_arm == "right"):
      planning_group = "arm_right"
      return 'leave_tool'
       
   
      
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
  
  
  
      
        
        
