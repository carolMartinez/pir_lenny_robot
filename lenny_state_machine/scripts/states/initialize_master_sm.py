import rospy
import roslib
import smach


from states.fake_states import MoveRobotHome
from states.fake_states import WaitFake
from states.tool_master_sm import makeToolMasterSM
from lenny_msgs.srv import *
from smach_ros import ServiceState
        
        
def makeInitMasterSM():
	
	sm = smach.StateMachine(outcomes=['succeeded','aborted'])

	req = MoveToHomeRequest()
	req.pose_name = "HOME"
	req.move_group = "sda10f"

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
    
		# Move robot to home position

		## Make sure all the robot is in home positon..
		smach.StateMachine.add('MOVE_HOME',ServiceState('/motion_executor/move_to_home',MoveToHome,request=req),transitions = {
					'succeeded':'succeeded',
					'aborted':'aborted',
					'preempted':'aborted'}
		)



		


					
	return sm
