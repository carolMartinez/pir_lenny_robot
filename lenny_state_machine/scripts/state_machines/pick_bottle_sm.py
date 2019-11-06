import rospy
import roslib
import smach


#from states.fake_states import *
#from states.pick_bottle_states import *
from states.motion_states import *
from trajectory_msgs.msg import JointTrajectory
from states.pick_bottle_states import InitPickBottles
from states.place_bottle_states import MoveHomePlaceBottles

from geometry_msgs.msg import Pose
       
        
def makePickBottleSM(dataSM):
  
  sm = smach.StateMachine(outcomes=['success','error'],
    input_keys=['object_pick_pose_input'])
   

  sm.userdata.sm_user_pose_approach = geometry_msgs.msg.Pose()
  sm.userdata.sm_user_pose_preGrasp = geometry_msgs.msg.Pose()
  sm.userdata.sm_user_pose_grasp = geometry_msgs.msg.Pose()
  sm.userdata.sm_user_pose_retreat = geometry_msgs.msg.Pose()
  sm.userdata.sm_user_trajectory = trajectory_msgs.msg.JointTrajectory()

  ##TODO: this will change for dual arm..Better to initialize it when the bottle 
  ### detection message arrive
         
          

  with sm:
    
    
           
         
          smach.StateMachine.add('INIT_PICK_BOTTLE', InitPickBottles(dataSM),
							transitions={'success': 'CREATE_PICK_MOVES',
                            'error' : 'error'})
        
          
          smach.StateMachine.add('CREATE_PICK_MOVES',CreatePickMoves(dataSM),
                        transitions = {
                        'success':'PLAN_COARSE',
                        'error':'error'},
                        remapping = {'object_pose_input' : 'object_pick_pose_input',
                        'robot_movements_output_approach' : 'sm_user_pose_approach',
                        'robot_movements_output_preGrasp' : 'sm_user_pose_preGrasp',
                        'robot_movements_output_grasp' : 'sm_user_pose_grasp',
                        'robot_movements_output_retreat' : 'sm_user_pose_retreat'}
                        )
                        
          smach.StateMachine.add('PLAN_COARSE',PlanCoarseMove(dataSM),
                        transitions = {
                        'success':'MOVE_COARSE',
                        'error':'error'},
                        remapping = {'robot_movements_input_approach' : 'sm_user_pose_approach',
                        'coarse_trajectory_output' : 'sm_user_trajectory'} 
                        )

          smach.StateMachine.add('MOVE_COARSE',ExecuteCoarseMove(dataSM),
                        transitions = {
                        'success':'PICK_BOTTLE',
                        'error':'error'},
                        remapping = {'coarse_trajectory_input' : 'sm_user_trajectory'} 
                        )
                        
          smach.StateMachine.add('PICK_BOTTLE',PlanExecutePickFineMove(dataSM),
                        transitions = {
                        'success':'MOVE_HOME_PLACE_BOTTLES',
                        'error':'error'},
                        remapping = {'robot_movements_input_preGrasp' : 'sm_user_pose_preGrasp',
                        'robot_movements_input_grasp' : 'sm_user_pose_grasp',
                        'robot_movements_input_retreat' : 'sm_user_pose_retreat'} 
                        )
                        
                        
          smach.StateMachine.add('MOVE_HOME_PLACE_BOTTLES',MoveHomePlaceBottles(dataSM),
                        transitions = {'success':'success',
                        'error':'error'})
         
          return sm
