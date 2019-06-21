import rospy
import roslib
import smach


from states.fake_states import MoveRobotHome
from states.fake_states import WaitFake


        
        
def makeInitMasterSM():
	sm = smach.StateMachine(outcomes=['success','error'])
	
	with sm:
		# Configure camera node.
		#smach.StateMachine.add(
		#		'Configure Gripper Camera',
		#		ServiceState('/gripper_camera/lifecycle/configure',
		#			RequestTransition,
		#			request=RequestTransitionRequest()),
		#		transitions = {
		#			'succeeded':'Activate Gripper Camera',
		#			'aborted':'Activate Gripper Camera',
		#			'preempted':'error'}
		#)
		
		
		
		# Configure gripper driver.
		#smach.StateMachine.add(
		#		'Configure Gripper Driver',
		#		ServiceState('/gripper_driver/lifecycle/configure',
		#			RequestTransition,
		#			request=RequestTransitionRequest()),
		#		transitions = {
		#			'succeeded':'Activate Gripper Driver',
		#			'aborted':'Activate Gripper Driver',
		#			'preempted':'error'}
		#)
		
		
		# Activate pose estimator.
		#smach.StateMachine.add(
		#		'Activate Pose Estimator',
		#		ServiceState('/pose_estimation/lifecycle/activate',
		#			RequestTransition,
		#			request=RequestTransitionRequest()),
		#		transitions = {
		#			'succeeded':'Configure Manipulation Planner',
		#			'aborted':'Configure Manipulation Planner',
		#			'preempted':'error'}
		#)
		#
    
		
		# Move robot to home position.
		smach.StateMachine.add(
				'MOVE_HOME',MoveRobotHome(),
				transitions = {
					'done':'success',
					'error':'error'}
		)
		
	return sm
