#!/usr/bin/env python

import rospy
import roslib
import smach 
import smach_ros 
import actionlib



from states.initialize_master_sm import makeInitMasterSM
from states.fake_states import WaitFake
#from states.tool_master_sm import *

from states.tool_master_sm import makeToolMasterSM


        
def main():
  
    rospy.init_node('master_sm')

    sm_root = smach.StateMachine(outcomes=[ 'done', 'error'])
    
  
                  
    with sm_root:
      
        
        smach.StateMachine.add('INIT_MASTER_SM', makeInitMasterSM(),
                              transitions={'success': 'FAKE_STATE',
                                            'error' : 'error'})
                                         
        smach.StateMachine.add('FAKE_STATE', WaitFake(),
                              transitions={'need_tool': 'TOOL_STATE',
                                            'error' : 'error'})
        smach.StateMachine.add('TOOL_STATE',makeToolMasterSM(),
				transitions = {
					'success':'FAKE_STATE',
					'error':'error',
					'continue': 'error'})

		#smach.StateMachine.add('DONE',FinishSM(),
		#		transitions = {
		#			'success':'error',
		#			'error':'error'}
					
		#)                         
                                        

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('my_smach_introspection_server', sm_root, '/SM_ROOT')
    sis.start()
    
    outcome = sm_root.execute()
    
    if (outcome == 'error'):
        sis.stop()
    #else:
	#	rospy.spin()
    

if __name__ == '__main__':
    main()
