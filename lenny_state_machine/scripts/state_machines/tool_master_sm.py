import rospy
import roslib
import smach


from states.motion_states import MoveRobotHome
from states.pick_bottle_states import MoveHomePickBottles
from states.fake_states import WaitFake
from states.pick_tool_states import MoveHomePickTools
from states.pick_tool_states import InitializeToolMasterSM
from state_machines.pick_tool_sm import makePickToolSM
     
        
def makeToolMasterSM():
  
    
	sm = smach.StateMachine(outcomes=['success','error'])
	
	with sm:
		
		

		smach.StateMachine.add(
				'MOVE_HOME_PICK_TOOLS',MoveHomePickTools(),
				transitions = {
					'pick':'PICK_TOOL_SM',
					'place':'PICK_TOOL_SM',
					'error':'error'}
		)
		
		smach.StateMachine.add(
				'PICK_TOOL_SM',makePickToolSM(),
				transitions = {
					'success':'MOVE_HOME_PICK_BOTTLES',
					'error':'error'}
		)
				
		smach.StateMachine.add(
				'MOVE_HOME_PICK_BOTTLES',MoveHomePickBottles(),
				transitions = {
					'success':'success',
					'error':'error'}
		)
		## TODO: change for makePlaceToolSM
		#smach.StateMachine.add(
		#		'PLACE_TOOL_SM',makePickToolSM(),
		#		transitions = {
		#			'success':'FAKE_STATE',
		#			'error':'error'}
		#)
	
	#outcome = sm_root.execute()

	return sm
    
    #if (outcome == 'error'):
    #    sis.stop()
        
	
#if __name__ == '__main__':
#    main()

