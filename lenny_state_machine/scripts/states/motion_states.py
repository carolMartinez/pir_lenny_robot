## This script contains generic motion class that can be used in 
## any application that requires moving the robot


import rospy
import roslib
import smach
import time
import moveit_commander
import tf2_ros

from lenny_msgs.srv import *
from pir_vision_msgs.srv import PirAttachObject, PirAttachObjectRequest
from pir_vision_msgs.srv import PirDetachObject, PirDetachObjectRequest




class MoveRobotHome(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['done','error'],
                                input_keys=['print_input'],
                                output_keys=['print_output'])

  def execute(self, userdata):
    rospy.loginfo('MOVE ROBOT INIT STATE')
    rospy.wait_for_service('/motion_executor/move_to_pose')
    
    try:
          #Moving ARMS to a HOME position to pick up the tool
          move_to_home = rospy.ServiceProxy('/motion_executor/move_to_pose', MoveToHome)
          resp = move_to_home("PICK_WAIT","arms")
           
          #Moving TORSO to HOME
          #Using movegroup python interface to move only the toros.
          #this because the go to pose goal did not work in c++
          move_group = moveit_commander.MoveGroupCommander('sda10f')
          joint_goal = move_group.get_current_joint_values()
          joint_goal[0] = 0
          plan=move_group.go(joint_goal,wait=True)
          
          if(plan):
            return 'done'
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
    

class CreatePickMoves(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['success','error'], input_keys=['object_pose_input'],
    output_keys=['robot_movements_output_approach', 'robot_movements_output_preGrasp',
    'robot_movements_output_grasp','robot_movements_output_retreat'])
    
    
  def execute(self, userdata):
    
    error=0
    
    rospy.wait_for_service('/motion_executor/create_pick_movements')
    
    try:
      create_pick_movements = rospy.ServiceProxy('/motion_executor/create_pick_movements', CreatePickMovements)

      #tfBuffer = tf2_ros.Buffer()
      #listener = tf2_ros.TransformListener(tfBuffer)

      #trans = geometry_msgs.msg.Transform()
      
      #try:
      #   trans = tfBuffer.lookup_transform('torso_base_link', 'object_link', rospy.Time(), rospy.Duration(3.0))
      #except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
      #   rospy.loginfo('tf Transformation not available')
      #   error = 1

      #if (error==0):
      #pose = geometry_msgs.msg.Pose()
      pose = userdata.object_pose_input
      #translation = trans.transform.translation
      #rotation =  trans.transform.rotation
      #pose.position = geometry_msgs.msg.Point(translation.x,translation.y,translation.z)
      #pose.orientation = rotation
      #print(pose)     
                  
      resp = create_pick_movements(pose)

      #userdata.robot_movements_output = [geometry_msgs.msg.Pose(), geometry_msgs.msg.Pose(), geometry_msgs.msg.Pose()]

    
      #userdata.robot_movements_output[0].position = resp.robot_movements[0].position

  #    userdata.robot_movements_output.append(resp.robot_movements[0])
      userdata.robot_movements_output_approach = resp.robot_movements[0]
      userdata.robot_movements_output_preGrasp = resp.robot_movements[1]
      userdata.robot_movements_output_grasp = resp.robot_movements[1]
      userdata.robot_movements_output_retreat = resp.robot_movements[2]
      
      
      
      #userdata.robot_movements_output[0].position = resp.robot_movements[0].position
      #userdata.robot_movements_output[2] = resp.robot_movements[2]
      
      
      print(resp.robot_movements[0].position)
      
      
      rospy.loginfo('CREATE PICK MOVES')
      
      if(resp.success):
        return 'success'
      else:
        rospy.loginfo('ERROR creating pick moves')
        return 'error'
       
    except rospy.ServiceException as exc:
           rospy.loginfo('Service did not process request: %s', exc)  
           return 'error'   
       
                
    if self.preempt_requested():
      self.service_preempt()
      return 'error'
    else:
      return 'success'





class PlanCoarseMove(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['success','error'], input_keys=['robot_movements_input_approach'], output_keys=['coarse_trajectory_output'])

  def execute(self, userdata):
    
    rospy.loginfo('PLAN COARSE MOTION')
    rospy.wait_for_service('/motion_executor/plan_coarse_motion')
    
    try:
      plan_coarse_motion=rospy.ServiceProxy('/motion_executor/plan_coarse_motion',PlanCoarseMotion)

      req = PlanCoarseMotionRequest()
      req.target_pose.position = userdata.robot_movements_input_approach.position
      req.target_pose.orientation = userdata.robot_movements_input_approach.orientation
      
      req.move_group = "arm_right"
      
      resp = plan_coarse_motion(req)

      userdata.coarse_trajectory_output = resp.coarse_trajectory
      

      
      if(resp.success):
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


class ExecuteCoarseMove(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['success','error'], input_keys=['coarse_trajectory_input'])

  def execute(self, userdata):
    
    
    rospy.wait_for_service('/motion_executor/execute_coarse_motion')
    
    execute_coarse_move=rospy.ServiceProxy('/motion_executor/execute_coarse_motion',ExecuteCoarseMotion)

    

    req = ExecuteCoarseMotionRequest()
    req.coarse_trajectory = userdata.coarse_trajectory_input
    req.move_group = "arm_right"
    
    resp = execute_coarse_move(req)
    
    
    rospy.loginfo('CREATE COARSE MOVES')
    
    if(resp.success):
		return 'success'
    else:
      return 'error' 
            
                
    if self.preempt_requested():
      self.service_preempt()
      return 'error'
    else:
      return 'success'



class PlanExecutePickFineMove(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['success','error'], input_keys=['robot_movements_input_grasp',
    'robot_movements_input_retreat'])

  def execute(self, userdata):
    
    ##TODO: make it generic group and object name must be passed from SM
    move_group_= "arm_right"
    move_group_tool_="gripper_3f"
    object_name_= "tool_2"
    
    #Variable to track errors during the different FINE movements
    error=0
    
    ##-------------------
    ##APPROACH OBJECT
    rospy.wait_for_service('/motion_executor/plan_execute_fine_motion')
    plan_execute_fine_move = rospy.ServiceProxy('/motion_executor/plan_execute_fine_motion',PlanExecuteFineMotion)


    #Creating the request to approach object
    req = PlanExecuteFineMotionRequest()
    req.target_poses.append(userdata.robot_movements_input_grasp)
    req.move_group = move_group_
    resp = plan_execute_fine_move(req)
    
    if(resp.success):
      rospy.loginfo('FINE MOTION APPROACH OBJECT')
      ##-------------------
      ##GRASP THE OBJECT
      
      rospy.loginfo('CLOSE GRIPPER')
      ##add here function
      time.sleep(2)
    
      
      ##-------------------
      ##ATTACH OBJECT
      rospy.wait_for_service('/pir_vision_utils_rviz/attached_object')
      
      try:
        attach_object = rospy.ServiceProxy('/pir_vision_utils_rviz/attached_object',PirAttachObject)

        req = PirAttachObjectRequest()
        req.group_name = move_group_tool_
        req.object_name = object_name_
        resp = attach_object(req)
        print(resp.status)
        
        #TODO: change Sucesfull for successful
        if(resp.status=='sucesfull'):
          print("Object ATTACHED")
        else:
          return 'error'  
          print("Error attaching object") 

      except rospy.ServiceException as exc:
        rospy.loginfo('pyr_vision_system attach_object Service did not process request: %s', exc)  
        return 'error'        
 
      
      rospy.loginfo('OBJECT ATTACHED')
      
      ##-------------------
      ##RETREAT WITH OBJECT IN HAND
      rospy.wait_for_service('/motion_executor/plan_execute_fine_motion')
      plan_execute_fine_move = rospy.ServiceProxy('/motion_executor/plan_execute_fine_motion',PlanExecuteFineMotion)


      #Creating the request to approach object
      req2 = PlanExecuteFineMotionRequest()
      req2.target_poses.append(userdata.robot_movements_input_retreat)
      req2.move_group = "arm_right"
      resp = plan_execute_fine_move(req2)
    
      if(resp.success):
        rospy.loginfo('FINE MOTION RETREAT WITH OBJECT')
      else:
        rospy.loginfo('ERROR ---> FINE MOTION RETREAT WITH OBJECT')
        error=1; 
      
      
      
    else:
      rospy.loginfo('ERROR   ---> FINE MOTION APPROACH OBJECT')
      error=1; 

    
    
    
    if(error==0):
      return 'success'
    else:
      return 'error' 
            
                
    if self.preempt_requested():
      self.service_preempt()
      return 'error'
    else:
      return 'success'



class PlanExecutePlaceFineMove(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['success','error'], input_keys=['robot_movements_input_grasp',
    'robot_movements_input_retreat'])

  def execute(self, userdata):
    
    ##TODO: make it generic group and object name must be passed from SM
    move_group_= "arm_right"
    move_group_tool_="gripper_3f"
    object_name_= "tool_2"
    
    #Variable to track errors during the different FINE movements
    error=0
    
    ##-------------------
    ##APPROACH OBJECT
    rospy.wait_for_service('/motion_executor/plan_execute_fine_motion')
    plan_execute_fine_move = rospy.ServiceProxy('/motion_executor/plan_execute_fine_motion',PlanExecuteFineMotion)


    #Creating the request to approach object
    req = PlanExecuteFineMotionRequest()
    req.target_poses.append(userdata.robot_movements_input_grasp)
    req.move_group = move_group_
    resp = plan_execute_fine_move(req)
    
    if(resp.success):
      rospy.loginfo('FINE MOTION APPROACH OBJECT')
      ##-------------------
      ##GRASP THE OBJECT
      
      rospy.loginfo('OPEN GRIPPER')
      ##add here function
      time.sleep(2)
    
      
      ##-------------------
      ##DETACH OBJECT
      rospy.wait_for_service('/s_vision_utils_rviz/detach_object')
      
      try:
        attach_object = rospy.ServiceProxy('/pir_vision_utils_rviz/detach_object',PirDetachObject)

        req = PirDetachObjectRequest()
        req.group_name = move_group_tool_
        req.object_name = object_name_
        resp = attach_object(req)
        print(resp.status)
        
        #TODO: change Sucesfull for successful
        if(resp.status=='sucesfull'):
          print("Object DETACHED")
        else:
          return 'error'  
          print("Error detaching object") 

      except rospy.ServiceException as exc:
        rospy.loginfo('pyr_vision_system detach_object Service did not process request: %s', exc)  
        return 'error'        
 
      
      rospy.loginfo('OBJECT DETACHED')
      
      ##-------------------
      ##RETREAT WITH OBJECT IN HAND
      rospy.wait_for_service('/motion_executor/plan_execute_fine_motion')
      plan_execute_fine_move = rospy.ServiceProxy('/motion_executor/plan_execute_fine_motion',PlanExecuteFineMotion)


      #Creating the request to approach object
      req2 = PlanExecuteFineMotionRequest()
      req2.target_poses.append(userdata.robot_movements_input_retreat)
      req2.move_group = "arm_right"
      resp = plan_execute_fine_move(req2)
    
      if(resp.success):
        rospy.loginfo('FINE MOTION RETREAT WITH OBJECT')
      else:
        rospy.loginfo('ERROR ---> FINE MOTION RETREAT WITH OBJECT')
        error=1; 
      
      
      
    else:
      rospy.loginfo('ERROR   ---> FINE MOTION APPROACH OBJECT')
      error=1; 

    
    
    
    if(error==0):
      return 'success'
    else:
      return 'error' 
            
                
    if self.preempt_requested():
      self.service_preempt()
      return 'error'
    else:
      return 'success'



     
        
        
      
        
        
