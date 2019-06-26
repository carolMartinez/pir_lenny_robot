
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit_msgs/ExecuteTrajectoryActionResult.h>
#include <moveit/robot_state/robot_state.h>
#include <trajectory_msgs/JointTrajectory.h>


class MotionUtilities
{
  private:
          boost::shared_ptr<sensor_msgs::JointState const> sda10f_state, all_states;
          boost::shared_ptr<moveit_msgs::ExecuteTrajectoryActionResult const> trajectory_result_;
          
          int current_result_val;
          //moveit_msgs::ExecuteTrajectoryResult trajectory_result_;
          //_action_result_type trajectory_result_;

          
          double topic_time_out;
  public:

          MotionUtilities () : topic_time_out(0.5)
          {
          };
        
           bool waitForRobotToStop()
           {
              all_states =  ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states", ros::Duration(topic_time_out));
              
              if (!all_states)
              {
                  ROS_ERROR_STREAM("Joint states have not been published");
                  return false;
              }
              
              if( (*all_states).position.size() == 35)
              {
				  ///TODO: check this and its advantages
				  // Using simulated robot. ??? reallly?
				 ROS_INFO("Using simulated robot "); 
                 return true;     
              }
              else
              {
                  //Get the actual joint states of the robot
                  while(true)
                  {
                      //rail_state_ = ros::topic::waitForMessage<sensor_msgs::JointState>("/sia20f/sia20f_b1_controller/joint_states", ros::Duration(topic_time_out_));

						///TODO: check if by only listening /joint_states topic is enough
                      //sda10f_state = ros::topic::waitForMessage<sensor_msgs::JointState>("/sda10f/sda10f_r2_controller/joint_states", ros::Duration(topic_time_out));
						sda10f_state = ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states", ros::Duration(topic_time_out));
                    
                      if (!sda10f_state) 
                      {
                        ROS_ERROR_STREAM("Robot states have not been published");
                        continue;
                      }
                      break;
                    
                    
                  }
                  
                  bool robot_has_stopped = false;
      
                  std::vector<double> current_positions;
                  int i=0;
                  while(!robot_has_stopped) 
                  {
                    current_positions = sda10f_state->position;

                    //sda10f_state = ros::topic::waitForMessage<sensor_msgs::JointState>("/sda10f/sda10f_r2_controller/joint_states", ros::Duration(topic_time_out));
                    sda10f_state = ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states", ros::Duration(topic_time_out));
                    
                    if (!sda10f_state) 
                    {
                      ROS_ERROR_STREAM("Robot states have not been published");
                      continue;
                    }
                    if(getDistance(current_positions, sda10f_state->position) < 1e-10) 
                    {
                      robot_has_stopped = true;
                      ROS_INFO_STREAM("Robot has stopped. " << i << "dist: " << getDistance(current_positions, sda10f_state->position));
                    
                   
                    }
                    i=i+1;
                    
                     ROS_INFO_STREAM("Distance " << getDistance(current_positions, sda10f_state->position));
                    
                  }
                  return (robot_has_stopped);
                  
              }
                
           } 
           
           
           
          double getDistance (std::vector<double> const & joints_p1, std::vector<double> const & joints_p2) 
          {
              double distance = 0.0;
              double diff = 0.0;
              if (joints_p1.size() != joints_p2.size()) 
              {
                  ROS_ERROR_STREAM ("Comparing points of different sizes.");
                  distance = 1e10;
                  return distance;
              }
              for (size_t pos_idx = 0; pos_idx < joints_p1.size(); pos_idx++) 
              {
                diff = joints_p1[pos_idx] - joints_p2[pos_idx];
                distance += diff*diff;
              }
              
              return sqrt(distance);
          }
           
	
};

