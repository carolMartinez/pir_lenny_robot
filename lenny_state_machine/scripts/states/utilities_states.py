import rospy
import roslib
import smach
import time
import moveit_commander
import tf2_ros

from pir_vision_msgs.srv import PirDetachAll, PirDetachAllRequest 
from pir_vision_msgs.srv import PirDeleteObject, PirDeleteObjectRequest 
from pir_vision_msgs.srv import PirGetAttachObject


        
      

class DetachObjects(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['success','error'])

  def execute(self, userdata):
    rospy.loginfo('DETACH OBJECT STATE')
    
    ##GET ATTACHed OBJECT
    rospy.wait_for_service('/pir_vision_utils_rviz/get_attached_object')
   
    
    try:
       
      get_attached_objects = rospy.ServiceProxy('/pir_vision_utils_rviz/get_attached_object',PirGetAttachObject)

      respGetObjects = get_attached_objects()
      
      if(respGetObjects.number_objects>0):
        ##DETACH ALL OBJECTS
        rospy.wait_for_service('/pir_vision_utils_rviz/detach_all')
        try:
          attach_object = rospy.ServiceProxy('/pir_vision_utils_rviz/detach_all',PirDetachAll)

          req = PirDetachAllRequest()
          req.group_names = ["gripper_2f","gripper_3f"]
          resp = attach_object(req)
          print(resp.status)
          
          #TODO: change Sucesfull for successful
          if(resp.status=='sucesfull'):
            
            for x in respGetObjects.object_names:
              ##DELETE OBJECT
              rospy.wait_for_service('/pir_vision_utils_rviz/delete_object')
              try:
                delete_object = rospy.ServiceProxy('/pir_vision_utils_rviz/delete_object',PirDeleteObject)
                req = PirDeleteObjectRequest()
                req.reference = ""
                req.object_name = x
                resp = delete_object(req)
                
                #TODO: change Sucesfull for successful
                if(resp.status=='sucesfull'):
                  print("Object DELETED")
                
              except rospy.ServiceException as exc:
                rospy.loginfo('pyr_vision_system delete_object Service did not process request: %s', exc)  
                return 'error' 
          else:
            return 'error'  
            print("Error detaching object") 
          
        except rospy.ServiceException as exc:
          rospy.loginfo('pyr_vision_system detach_all Service did not process request: %s', exc)  
          return 'error'      
      else:
        print("No objects attached") 
        
    except rospy.ServiceException as exc:
      rospy.loginfo('pyr_vision_system get_attach_objects Service did not process request: %s', exc)  
      return 'error'  

    
    
    
    if self.preempt_requested():
      self.service_preempt()
      return 'error'
    else:
      return 'success'

      

        
        
