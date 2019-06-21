import rospy
import roslib
import smach


from states.fake_states import MoveRobotHome
from states.fake_states import WaitFake
from states.fake_states import InitializeToolMasterSM

     
        
def main():
  
    rospy.init_node('tool_master_sm')

	sm = smach.StateMachine(outcomes=['success','error'])
	
	with sm:
		
		smach.StateMachine.add(
				'INITIALIZE',InitializeToolMasterSM(),
				transitions = {
					'done':'MOVE_HOME_PICK_TOOLS',
					'error':'error'}
		)
		
		# Move robot to home position.
		smach.StateMachine.add(
				'MOVE_HOME_PICK_TOOLS',MoveHomePickTools(),
				transitions = {
					'pick':'PICK_TOOL_SM',
					'place':'PLACE_TOOL_SM',
					'error':'error'}
		)
		
		smach.StateMachine.add(
				'PICK_TOOL_SM',makePickToolSM(),
				transitions = {
					'success':'success',
					'error':'error'}
		)
				
		smach.StateMachine.add(
				'MOVE_HOME_PICK_BOTTLES',MoveRobotHome(),
				transitions = {
					'success':'success',
					'error':'error'}
		)
		## TODO: change for makePlaceToolSM
		smach.StateMachine.add(
				'PLACE_TOOL_SM',makePickToolSM(),
				transitions = {
					'success':'success',
					'error':'error'}
		)
	
	outcome = sm_root.execute()
    
    if (outcome == 'error'):
        sis.stop()
        
	
if __name__ == '__main__':
    main()

