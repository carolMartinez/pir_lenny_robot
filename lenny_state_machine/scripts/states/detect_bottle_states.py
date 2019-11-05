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
    
    class TaskDefinition:
      pick_pose = geometry_msgs.msg.Pose()
      place_pose = geometry_msgs.msg.Pose()
      object_type = "PET" # "HDPE-COLOR etc"
      arm_name = rospy.get_param("lenny_task/task_for_arm")
      tool_type = rospy.get_param("lenny_task/tool_type")
    
    ##TODO: change by Wilson parameter
    robot_config = rospy.get_param("lenny_task/robot_config") #"single" "dual"
    object1 = TaskDefinition()
    object2 = TaskDefinition()
    task = TaskDefinition()
   
    
    if (self.dataSM.fake_vision == "true"):
      
      ###TODO: change name of the service
      rospy.wait_for_service('/pir_vision_fake_simulation/star_bottle_detection')
      
      try:
        
        bottle_detection = rospy.ServiceProxy('/pir_vision_fake_simulation/star_bottle_detection', PirTaskDefinition)
      
        req = PirTaskDefinition()
        
        req.action = "start"
        
        resp = bottle_detection("start")
        
        #TODO: change sucessfull
        if (resp.status == "sucesfull"):
          object1.pick_pose = resp.arm_left.pick_pose
          object1.place_pose = resp.arm_left.place_pose
          object1.object_type = resp.arm_left.object_type
          object1.arm_name = resp.arm_left.arm_name
          object1.tool_type = resp.arm_left.tool_type
          
          object2.pick_pose = resp.arm_right.pick_pose
          object2.place_pose = resp.arm_right.place_pose
          object2.object_type = resp.arm_right.object_type
          object2.arm_name = resp.arm_right.arm_name
          object2.tool_type = resp.arm_right.tool_type
          
           
          #print(object2)
          ## TODO REMOVE THIS IS ONLY FOR TESTING
          object2.tool_type = "gripper"
          object1.tool_type = "gripper"
          
        else:
          return 'error'
      
      except rospy.ServiceException as exc:
           rospy.loginfo('Service did not process request: %s', exc)  
           return 'error'         
    else:
      rospy.loginfo('Real Robot')  
             
      
    #if I read a ros param and says that the robot has the tool it also
    #says in which arm
    #FAKE MESSAGE THAT WILL BE PASSED BY WILSON SERVICE TaskDefinition.msg
    
    
    
    
    
    
    ### I will fill the data the same way wilson will pass it
    ##object1.pick_pose = geometry_msgs.msg.Pose()
    ##object1.place_pose = geometry_msgs.msg.Pose()
    ##object1.object_type = "PET" # "HDPE-COLOR etc"
    ##object1.task_for_arm = rospy.get_param("lenny_task/task_for_arm")
    ##object1.tool_type = rospy.get_param("lenny_task/tool_type")
    
    ##object2.pick_pose = geometry_msgs.msg.Pose()
    ##object2.place_pose = geometry_msgs.msg.Pose()
    ##object2.object_type = "PET" # "HDPE-COLOR etc"
    ##object2.task_for_arm = "arm_right" #"arm_left"
    ##object2.tool_type = "vacuum" #"vacuum" 
    
    
    
    
    
    ##For testing, place this at the begining of the state machine.
    #self.dataSM.tool_in_arm = " "
    #self.dataSM.tool_name = "tool_1"

    
    ## This data will be filled out when wilson message arrive
    #self.dataSM.task_for_arm = object1.task_for_arm
    #self.dataSM.task_tool_type = object1.tool_type


    #tool_in_arm = self.dataSM.tool_in_arm
    #tool_name = self.dataSM.tool_name
    
   
    if (robot_config == "single"):
      
      #+++++++ANALYZING TASK MESSAGE
      if (object1.arm_name == " " or object1.arm_name == ''):
        task = object2
       
      if (object2.arm_name == " "  or object2.arm_name == ''):
        task = object1
      
      print("BBB=", task.place_pose)
      #+++++++ANALYZING TASK INFORMATION
      
      ## TODO: wilson change problem with w value
      self.dataSM.place_pose = task.place_pose
      self.dataSM.place_pose.orientation.w=20.0;
      self.dataSM.pick_pose = task.pick_pose
      self.dataSM.pick_pose.orientation.w=20.0;
      self.dataSM.pick_pose.orientation.z=0.0;
      
      
         
       
      #Already filled from yaml
      if(task.arm_name == "arm_right"):
        self.dataSM.planning_group_tool = self.dataSM.ee_arm_right
      if(task.arm_name == "arm_left"):
        self.dataSM.planning_group_tool = self.dataSM.ee_arm_left
      
      #  self.dataSM.planning_group_tool = "gripper_3f"
      #  print(self.dataSM.planning_group_tool)
       
      
      ##todo resolver problema que sigue poniendo group tool 2f 
    
      #LEo el primer dato cada uno de los dats que viene de wilson.
      if (self.dataSM.tool_in_arm == " "):
        if (task.tool_type == "tool"):
          #It means we require the tool
          self.dataSM.change_tool_hand = False
          self.dataSM.planning_group_robot = task.arm_name
          self.dataSM.task_for_arm = task.arm_name
          return 'need_tool' 
          
        if (task.tool_type == "gripper"):
          #It means we are ready to pick up bottles
          self.dataSM.change_tool_hand = False
          self.dataSM.planning_group_robot = task.arm_name
          self.dataSM.task_for_arm = task.arm_name
          return 'pick_bottle' 
       
      
      if (self.dataSM.tool_in_arm == task.arm_name):
         
       #If the tool is in the same arm we need
       if(task.tool_type == "tool"): 
        #It means we do not need to pick up the tool it is already in its hand
        self.dataSM.planning_group_robot = task.arm_name
        self.dataSM.change_tool_hand = False
        self.dataSM.task_for_arm = task.arm_name
        return 'pick_bottle' 
        
       #If the tool is in the same arm we need But wee need to use the
       #gripper, then go and leave the tool
       if(task.tool_type == "gripper"): 
        self.dataSM.change_tool_hand = False
        self.dataSM.planning_group_robot = self.dataSM.tool_in_arm
        self.dataSM.task_for_arm = task.arm_name
        return 'leave_tool'
       
       
      if(self.dataSM.tool_in_arm != task.arm_name) and self.dataSM.tool_in_arm != " ":
         
       # The tool is in another arm.. So, first go a leave the tool and then
       # go and pick it up with the other arm
       if(task.tool_type == "tool"): 
       
        self.dataSM.planning_group_robot = self.dataSM.tool_in_arm
        self.dataSM.task_for_arm = task.arm_name
        self.dataSM.change_tool_hand = True
        self.dataSM.task_for_arm = task.arm_name
        return 'leave_tool' 
       
       if(task.tool_type == "gripper"): 
        self.dataSM.planning_group_robot = task.arm_name
        self.dataSM.change_tool_hand = False
        self.dataSM.task_for_arm = task.arm_name
        return 'pick_bottle' 
          
    
      
      
    
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
  
  
  
      
        
        
