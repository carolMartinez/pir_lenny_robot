import rospy
import roslib
import smach


from states.fake_states import MoveRobotHome
from states.fake_states import MoveHomePickTools
from states.fake_states import MoveHomePickBottles


from states.fake_states import WaitFake
from states.fake_states import InitializeToolMasterSM
from states.pick_tool_sm import makePickToolSM
     
        
def makeToolMasterSM():
  
    
    #rospy.init_node('tool_master_sm')

	sm = smach.StateMachine(outcomes=['success','error'])
	
	with sm:
		
		
		
		# Move robot to home position.
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

