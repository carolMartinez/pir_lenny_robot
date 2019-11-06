#!/usr/bin/env python

"""@package Main Task Coordinator

This scrip creates the main state machine that will be in charge of
coordinating the task to be developed by the robot.

August 16/2019
Currently it has the foolwing states:
1. INIT: this state initializes the initMasterSM that contains some
        initialization states, for example move the robot to start
        position.
2. FAKE_STATE: This is a fake state created to transition from the
        INIT to the TOOL STATE state machine.

3. PICK_PLACE_TOOL: This is the state in charge of coordinating the 
        pick and place routine of the tool.

4. FAKE_STATE2: This is a fake state created to transition from the
        PICK_PLACE_TOOL to the END of the state machine.
         
        
        
        


"""

import rospy
import roslib
import smach 
import smach_ros 
import actionlib



from state_machines.initialize_master_sm import makeInitMasterSM
from state_machines.pick_tool_sm import makePickToolSM
from state_machines.place_tool_sm import makePlaceToolSM
from state_machines.pick_bottle_sm import makePickBottleSM
from state_machines.place_bottle_sm import makePlaceBottleSM

from states.detect_bottle_states import DetectBottlesToPick

from states.fake_states import WaitFake
from states.fake_states import WaitFake2
    
from geometry_msgs.msg import Pose
    


class DataBetweenStates:
  def __init__(self):
    
    #Read ROS parameter server to get the parameters that define the task
    
    #This parameter is to define the robot configuration: single or dual_arm
    self.robot_config = rospy.get_param("lenny_task/robot_config")
    
    #Name of the vacuum tool. It is used to request the position when calling
    #the appropriate service
    self.tool_name = rospy.get_param("lenny_task/tool_name")
    
  
    
    #Name of the end effector of the right arm
    self.ee_arm_right = rospy.get_param("lenny_task/ee_arm_right")
    #Name of the end effector of the left arm
    self.ee_arm_left = rospy.get_param("lenny_task/ee_arm_left")
    
    #Parameter created in the launch file to define if working in simulation mode
    self.fake_vision = rospy.get_param("lenny_task/fake_vision")
    
    #Parameter created in the launch file to define if working with real grippers
    self.fake_gripper = rospy.get_param("lenny_task/fake_gripper")
    
    #These parameters are the ones that define the task.
    #Planning group
    self.planning_group_robot = "arm_left"
    
    #TODO: add when dual-arm is used
    if(self.planning_group_robot == "arm_left"):
      self.planning_group_tool = self.ee_arm_left
    
    if(self.planning_group_robot == "arm_right"):
      self.planning_group_tool = self.ee_arm_right
    
    #This is where the pose of the object to pick
    self.pick_pose = Pose()
    #This is where the pose of the object to place
    self.place_pose = Pose()
    
    
    #Variable that defines if the robot has to change the tool
    self.change_tool_hand = False

    #Variable to know in which arm the tool is
    self.tool_in_arm = rospy.get_param("lenny_task/tool_in_arm") #"single" "dual"
    
    #To know which object attach to the robot (tool, bottle)
    self.attach_object_name = ""



        
def main():
  
    rospy.init_node('master_sm')

    sm_root = smach.StateMachine(outcomes=[ 'done', 'error'])
    
    sm_root.userdata.sm_user_object_pick_pose = Pose()
    sm_root.userdata.sm_user_object_place_pose = Pose()
    
    
    
    dataSM = DataBetweenStates()              
    
    with sm_root:
      
        smach.StateMachine.add('INIT_MASTER_SM', makeInitMasterSM(dataSM),
							transitions={'success': 'DETECT_BOTTLES',
                            'aborted' : 'error'})
                                         
        smach.StateMachine.add('DETECT_BOTTLES', DetectBottlesToPick(dataSM),
							transitions={'pick_bottle':'PICK_BOTTLE_SM',
											'need_tool': 'PICK_TOOL_SM',
                      'leave_tool': 'PLACE_TOOL_SM',
											'error' : 'error'},
              remapping = {'object_pick_pose_output' : 'sm_user_object_pick_pose',
                        'object_place_pose_output' : 'sm_user_object_place_pose'})
                      
        smach.StateMachine.add('PICK_TOOL_SM',makePickToolSM(dataSM),
							transitions = {'success':'PICK_BOTTLE_SM',
											'error':'error'})
                      
        smach.StateMachine.add('PLACE_TOOL_SM',makePlaceToolSM(dataSM),
							transitions = {'success':'PICK_BOTTLE_SM',
                      'need_tool' : 'PICK_TOOL_SM',
											'error':'error'})              
                                    
        smach.StateMachine.add('PICK_BOTTLE_SM', makePickBottleSM(dataSM),
							transitions = {'success':'PLACE_BOTTLE_SM',
											'error':'error'},
               remapping = {'object_pick_pose_input' : 'sm_user_object_pick_pose'})
				
        smach.StateMachine.add('PLACE_BOTTLE_SM', makePlaceBottleSM(dataSM),
							transitions = {'success':'DETECT_BOTTLES',
											'error':'error'},
               remapping = {'object_place_pose_input' : 'sm_user_object_place_pose'})
								
		
    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('my_smach_introspection_server', sm_root, '/SM_ROOT')
    sis.start()
    
    outcome = sm_root.execute()
    
    if (outcome == 'error'):
        sis.stop()
   

if __name__ == '__main__':
    main()
