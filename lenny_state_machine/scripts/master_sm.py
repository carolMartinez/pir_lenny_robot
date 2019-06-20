#!/usr/bin/env python

import rospy
import roslib
import smach 
import smach_ros 
import actionlib



from states.init_master import makeInitStateMachineMaster
from states.fake_states import WaitFake




        
def main():
  
    rospy.init_node('maste_smr')

    sm_root = smach.StateMachine(outcomes=[ 'done', 'error'])
    
  
                  
    with sm_root:
      
        
        smach.StateMachine.add('INIT', makeInitMasterSM(),
                              transitions={'success': 'FAKE_STATE',
                                            'error' : 'error'})
        smach.StateMachine.add('FAKE_STATE', WaitFake(),
                              transitions={'need_tool': 'error',
                                            'error' : 'error'})
                              
                                        

    # Create and start the introspection server
    intro_server = smach_ros.IntrospectionServer('my_smach_introspection_server', sm_root, '/SM_ROOT')
    intro_server.start()
    
    outcome = sm_root.execute()
    
    if (outcome == 'error'):
        siss.stop()
        
    

if __name__ == '__main__':
    main()
