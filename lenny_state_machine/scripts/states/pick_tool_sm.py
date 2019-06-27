import rospy
import roslib
import smach


from states.fake_states import *

        
        
def makePickToolSM():
	sm = smach.StateMachine(outcomes=['success','error'], input_keys=['sm_robot_movements_input'],output_keys=['sm_robot_movements_output'])
	
	with sm:
		
		# Move robot to home position.
#		smach.StateMachine.add(
#				'DETECT TOOL', DetectTool(),
#				transitions = {
#				'success':'PLAN_COARSE',
#				'error':'error'}
#		)
          
		smach.StateMachine.add(
				'CREATE_PICK_MOVES',CreatePickMoves(),
				transitions = {
					'success':'MOVE_COARSE',
					'error':'error'},
          remapping = {'robot_movements_output' : 'sm_robot_movements_output'}
		)
		
    https://stackoverflow.com/questions/48121371/how-to-correctly-pass-mutable-objects-using-ros-smach-fsm
    
		smach.StateMachine.add(
				'MOVE_COARSE',ExecuteCoarseMotion(),
				transitions = {
					'success':'PLAN_FINE',
					'error':'error'},
          remapping = {'robot_movements_input': 'sm_robot_movements_output'}	
          
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
