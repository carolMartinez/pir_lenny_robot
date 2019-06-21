import rospy
import roslib
import smach


from states.fake_states import *

        
        
def makePickToolSM():
	sm = smach.StateMachine(outcomes=['success','error'])
	
	with sm:
		
		# Move robot to home position.
		smach.StateMachine.add(
				'PLAN_COARSE',PlanCoarseMotion(),
				transitions = {
					'success':'MOVE_COARSE',
					'error':'error'}
		)
		
		smach.StateMachine.add(
				'MOVE_COARSE',MoveCoarseMotion(),
				transitions = {
					'success':'PLAN_FINE',
					'error':'error'}
		)
				
		smach.StateMachine.add(
				'PLAN_FINE',PlanFineMotion(),
				transitions = {
					'success':'MOVE_FINE',
					'error':'error'}
		)
		## TODO: change for makePlaceToolSM
		smach.StateMachine.add(
				'MOVE_FINE',MoveFineMotion(),
				transitions = {
					'success':'success',
					'error':'error'}
		)

	return sm
