import rospy
import roslib
import smach


from states.fake_states import *
from trajectory_msgs.msg import JointTrajectory
        
        
def makePickToolSM():
	
	sm = smach.StateMachine(outcomes=['success','error'])

	sm.userdata.sm_user_pose_approach = geometry_msgs.msg.Pose();
	sm.userdata.sm_user_pose_preGrasp = geometry_msgs.msg.Pose();
	sm.userdata.sm_user_pose_grasp = geometry_msgs.msg.Pose();
	sm.userdata.sm_user_pose_retreat = geometry_msgs.msg.Pose();

	
	sm.userdata.sm_trajectory = trajectory_msgs.msg.JointTrajectory()
	
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
          remapping = {'robot_movements_output_approach' : 'sm_user_pose_approach',
          'robot_movements_output_preGrasp' : 'sm_user_pose_preGrasp',
          'robot_movements_output_grasp' : 'sm_user_pose_grasp',
          'robot_movements_output_retreat' : 'sm_user_pose_retreat'}
		)

		
    
		smach.StateMachine.add(
				'PLAN_COARSE',PlanCoarseMove(),
				transitions = {
					'success':'MOVE_COARSE',
					'error':'error'},
				remapping = {'robot_movements_input_approach' : 'sm_user_pose_approach',
					'coarse_trajectory_output' : 'sm_trajectory'} 
		)
		
		smach.StateMachine.add(
				'MOVE_COARSE',ExecuteCoarseMove(),
				transitions = {
					'success':'PICK_TOOL',
					'error':'error'},
				remapping = {'coarse_trajectory_input' : 'sm_trajectory'} 
		)
    
				
		smach.StateMachine.add(
				'PICK_TOOL',PlanExecuteFineMove(),
				transitions = {
					'success':'success',
					'error':'error'},
          remapping = {'robot_movements_input_preGrasp' : 'sm_user_pose_preGrasp',
			  'robot_movements_input_grasp' : 'sm_user_pose_grasp',
			  'robot_movements_input_retreat' : 'sm_user_pose_retreat'
			  } 
		)
		## TODO: change for makePlaceToolSM
		#smach.StateMachine.add(
		#		'MOVE_FINE',MoveFineMotion(),
		#		transitions = {
		#			'success':'success',
		#			'error':'error'}
		#)

	return sm
