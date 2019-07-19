import rospy
import roslib
import smach


from states.fake_states import *
from trajectory_msgs.msg import JointTrajectory
        
        
def makePickToolSM():
	sm = smach.StateMachine(outcomes=['success','error'])

	sm.userdata.sm_user_pose = []
  #   = geometry_msgs.msg.Pose()
	sm.userdata.sm_trajectory    = trajectory_msgs.msg.JointTrajectory()
	
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
					'success':'PLAN_COARSE',
					'error':'error'},
          remapping = {'robot_movements_output' : 'sm_user_pose'}
		)
		
 
    
		smach.StateMachine.add(
				'PLAN_COARSE',PlanCoarseMove(),
				transitions = {
					'success':'MOVE_COARSE',
					'error':'error'},
				remapping = {'robot_movements_input' : 'sm_user_pose',
					'coarse_trajectory_output' : 'sm_trajectory'} 
		)
		
		smach.StateMachine.add(
				'MOVE_COARSE',ExecuteCoarseMove(),
				transitions = {
					'success':'PLAN_MOVE_FINE',
					'error':'error'},
				remapping = {'coarse_trajectory_input' : 'sm_trajectory'} 
		)
    
				
		smach.StateMachine.add(
				'PLAN_MOVE_FINE',PlanExecuteFineMove(),
				transitions = {
					'success':'MOVE_FINE',
					'error':'error'},
          remapping = {'robot_movements_input' : 'sm_user_pose'} 
		)
		## TODO: change for makePlaceToolSM
		smach.StateMachine.add(
				'MOVE_FINE',MoveFineMotion(),
				transitions = {
					'success':'success',
					'error':'error'}
		)

	return sm
