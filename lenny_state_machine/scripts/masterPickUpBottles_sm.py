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

from states.detect_bottle_states import DetectBottlesToPick

from states.fake_states import WaitFake
from states.fake_states import WaitFake2
    
from geometry_msgs.msg import Pose
    


class DataBetweenStates:
  def __init__(self):

    self.tool_name = "tool_1"
    self.ee_arm_right = "gripper_3f"
    self.ee_arm_left = "gripper_2f"
    self.planning_group_robot = "arm_left"
    self.planning_group_tool = "gripper_2f"
    
    self.tool_in_arm = "no"
    self.tool_pose = Pose()
    self.object_to_pick = Pose()
    
        
def main():
  
    rospy.init_node('master_sm')

    sm_root = smach.StateMachine(outcomes=[ 'done', 'error'])
    
    sm_root.userdata.sm_tool_name = " "
    
    
    dataSM = DataBetweenStates()              
    
    with sm_root:
      
        
        smach.StateMachine.add('INIT_MASTER_SM', makeInitMasterSM(dataSM),
							transitions={'succeeded': 'DETECT_BOTTLES',
                            'aborted' : 'error'})
                                         
        smach.StateMachine.add('DETECT_BOTTLES', DetectBottlesToPick(dataSM),
							transitions={'pick_bottle':'FAKE_STATE_2',
											'need_tool': 'PICK_TOOL_SM',
                      'leave_tool': 'PLACE_TOOL_SM',
											'error' : 'error'})
                      
        smach.StateMachine.add('PICK_TOOL_SM',makePickToolSM(dataSM),
							transitions = {'success':'FAKE_STATE_2',
											'error':'error'})
                      
        smach.StateMachine.add('PLACE_TOOL_SM',makePlaceToolSM(dataSM),
							transitions = {'success':'FAKE_STATE_2',
											'error':'error'})              
                                    
        smach.StateMachine.add('FAKE_STATE_2', WaitFake2(),
							transitions = {'success':'error',
											'error':'error'})
								
		
    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('my_smach_introspection_server', sm_root, '/SM_ROOT')
    sis.start()
    
    outcome = sm_root.execute()
    
    if (outcome == 'error'):
        sis.stop()
   

if __name__ == '__main__':
    main()
