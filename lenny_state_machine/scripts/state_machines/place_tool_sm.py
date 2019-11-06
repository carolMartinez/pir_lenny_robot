import rospy
import roslib
import smach


#from states.fake_states import *
from states.place_tool_states import *
from states.motion_states import *
from trajectory_msgs.msg import JointTrajectory
from states.pick_bottle_states import MoveHomePickBottles        
        
def makePlaceToolSM(dataSM):
  
  sm = smach.StateMachine(outcomes=['success','error','need_tool'])

  sm.userdata.sm_user_place_pose = geometry_msgs.msg.Pose()
  sm.userdata.sm_user_place_pose_approach = geometry_msgs.msg.Pose()
  sm.userdata.sm_user_place_pose_preGrasp = geometry_msgs.msg.Pose()
  sm.userdata.sm_user_place_pose_grasp = geometry_msgs.msg.Pose()
  sm.userdata.sm_user_place_pose_retreat = geometry_msgs.msg.Pose()
  sm.userdata.sm_user_trajectory = trajectory_msgs.msg.JointTrajectory()
  
  dataSM.attach_object_name = dataSM.tool_name 

  with sm:
    
          
          smach.StateMachine.add('MOVE_HOME_PLACE_TOOLS',MoveHomePlaceTools(dataSM),
                        transitions = {
                        'success':'DETECT_PLACE_TOOL'}
                        )

          smach.StateMachine.add('DETECT_PLACE_TOOL',DetectToolMagazine(dataSM),
                        transitions = {
                        'success':'RESTORE_TCP_1',
                        'error':'error'},
                        remapping = {
                        'place_pose_output' : 'sm_user_place_pose'}
                        )
                        
          smach.StateMachine.add('RESTORE_TCP_1',ReinitTCP(dataSM),
                        transitions = {
                        'success':'CREATE_PLACE_MOVES',
                        'error':'error',
                        'pick': 'need_tool'}
                        )              

          smach.StateMachine.add('CREATE_PLACE_MOVES',CreatePickMoves(dataSM),
                        transitions = {
                        'success':'PLAN_COARSE',
                        'error':'error'},
                        remapping = {'object_pose_input' : 'sm_user_place_pose',
                        'robot_movements_output_approach' : 'sm_user_place_pose_approach',
                        'robot_movements_output_preGrasp' : 'sm_user_place_pose_preGrasp',
                        'robot_movements_output_grasp' : 'sm_user_place_pose_grasp',
                        'robot_movements_output_retreat' : 'sm_user_place_pose_retreat'}
                        )
          print(sm.userdata.sm_user_place_pose_approach)             
          smach.StateMachine.add('PLAN_COARSE',PlanCoarseMove(dataSM),
                        transitions = {
                        'success':'MOVE_COARSE',
                        'error':'error'},
                        remapping = {'robot_movements_input_approach' : 'sm_user_place_pose_approach',
                        'coarse_trajectory_output' : 'sm_user_trajectory'} 
                        )

          smach.StateMachine.add('MOVE_COARSE',ExecuteCoarseMove(dataSM),
                        transitions = {
                        'success':'PLACE_TOOL',
                        'error':'error'},
                        remapping = {'coarse_trajectory_input' : 'sm_user_trajectory'} 
                        )
                        
          smach.StateMachine.add('PLACE_TOOL',PlanExecutePlaceFineMove(dataSM),
                        transitions = {
                        'success':'RESTORE_TCP',
                        'error':'error'},
                        remapping = {'robot_movements_input_preGrasp' : 'sm_user_place_pose_preGrasp',
                        'robot_movements_input_grasp' : 'sm_user_place_pose_grasp',
                        'robot_movements_input_retreat' : 'sm_user_place_pose_retreat'} 
                        )
          
          smach.StateMachine.add('RESTORE_TCP',ReinitTCP(dataSM),
                        transitions = {
                        'success':'MOVE_HOME_PICK_BOTTLES',
                        'error':'error',
                        'pick': 'need_tool' }
                        )              
          
          smach.StateMachine.add('MOVE_HOME_PICK_BOTTLES',MoveHomePickBottles(),
                        transitions = {'success':'success',
                        'error':'error'})
                        
          
          
          
         
          return sm
