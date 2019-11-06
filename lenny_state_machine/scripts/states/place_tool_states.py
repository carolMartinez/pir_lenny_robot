import rospy
import roslib
import smach
import time
import moveit_commander
import tf2_ros


from lenny_msgs.srv import *
from pir_vision_msgs.srv import *



class DetectToolMagazine(smach.State):
  def __init__(self,dataSM):
    smach.State.__init__(self, outcomes=['success','error'],
                              input_keys=['place_pose_output'],
                              output_keys=['place_pose_output'])
    self.dataSM = dataSM 

  def execute(self, userdata):
   
    
    parameterName = "/pir_vision_back_pose/" + self.dataSM.tool_name
    
    print(parameterName)
    if(parameterName == "error"):
      print("empty")
    
    
   ##TODO: MAke it generic
    vector_place_pose=rospy.get_param(parameterName)
    userdata.place_pose_output.position.x = vector_place_pose[0]
    userdata.place_pose_output.position.y = vector_place_pose[1]
    userdata.place_pose_output.position.z = vector_place_pose[2]
    userdata.place_pose_output.orientation.x = vector_place_pose[3]
    userdata.place_pose_output.orientation.y = vector_place_pose[4]
    userdata.place_pose_output.orientation.z = vector_place_pose[5]
    userdata.place_pose_output.orientation.w = 0.0
    ##TODO: change this check with wilson the tool detection problem
    #userdata.place_pose_output.orientation.w = vector_place_pose[6]
    
    self.dataSM.attach_object_name = self.dataSM.tool_name 
    
    return 'success'
     
  
    if self.preempt_requested():
      self.service_preempt()
      return 'error'
    else:
      return 'success'
  

  


class ReinitTCP(smach.State):
  def __init__(self, dataSM):
    smach.State.__init__(self,
                outcomes = ['success', 'error', 'pick'])
    self.dataSM = dataSM

  def execute(self, userdata):
    rospy.loginfo('Calling Service Change TCP')
    rospy.wait_for_service('/motion_executor/restoreTCP')
    
    try:
          #Moving ARMS to a HOME position to pick up the tool
          change_TCP= rospy.ServiceProxy('/motion_executor/restoreTCP', RestoreTCP)
          
          ##TODO: Change to make it generic, arm_right should be a variable.... passed
          ##throug the State Machine.
          req = RestoreTCPRequest()
          req.arm_name = self.dataSM.planning_group_robot
          
          resp = change_TCP(req)
           
          if(resp.success):
            
            ## Update data to know if it has to pick the bottle or the tool
            if(self.dataSM.change_tool_hand == True):
              ## tengo que cambiar el grupo y la salida.
              self.dataSM.planning_group_robot = self.dataSM.task_for_arm
              if(self.dataSM.task_for_arm == "arm_right"):
                self.planning_group_tool = "gripper_3f"
              
              if(self.dataSM.task_for_arm == "arm_left"):
                self.planning_group_tool = "gripper_2f"
              
              #Now there is no tool in arm
              self.dataSM.tool_in_arm = " "
              
              return 'pick'
            else:
              self.dataSM.tool_in_arm = " "
              return 'success'
                
          else:
            return 'error' 
          
    except rospy.ServiceException as exc:
           rospy.loginfo('Restore_TCP service did not process request: %s', exc)  
           return 'error'   
      
    if self.preempt_requested():
      self.service_preempt()
      return 'error'
    else:
      return 'success'
        


  
class MoveHomePlaceTools(smach.State):
  def __init__(self, dataSM):
    smach.State.__init__(self, outcomes=['success','error'])
    self.dataSM = dataSM

  def execute(self, userdata):
    #userdata.print_input = None
    rospy.wait_for_service('/motion_executor/move_to_predefined_pose')
    
    try:
          #Moving ARMS to a HOME position to pick up the tool
          move_to_wait = rospy.ServiceProxy('/motion_executor/move_to_predefined_pose', MoveToPredefinedPose)
          resp = move_to_wait("PICK_WAIT","arms")
           
          #Moving TORSO to HOME
          #Using movegroup python interface to move only the toros.
          #this because the go to pose goal did not work in c++
          move_group = moveit_commander.MoveGroupCommander('sda10f')
          joint_goal = move_group.get_current_joint_values()
         
          if(self.dataSM.planning_group_robot == "arm_right"):
            joint_goal[0] = 2.9000
          if(self.dataSM.planning_group_robot == "arm_left"):
            joint_goal[0] = 1.6000
            
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
    else:
      return 'success'
  

      
        
        
