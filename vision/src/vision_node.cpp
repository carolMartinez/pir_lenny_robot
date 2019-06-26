#include "ros/ros.h"
#include "std_msgs/String.h"

///Including message header
#include "vision/bottle_data.h"

#include <signal.h>

#include <sstream>

#define KEYCODE_Q 0x71
#define KEYCODE_P 0x70

/**
 * This clase is in charge of sending messages based on keyboard actions
 */
class TeleopKeyboard
{
    public:
        TeleopKeyboard();
        void loop(); /// Loop for publishing keyboard commands
    private:
        ros::NodeHandle nh; ///Node handler
        ros::Publisher bottle_data_pub;           /// Variable to publis pose data


};

TeleopKeyboard::TeleopKeyboard()
{

    bottle_data_pub = nh.advertise<vision::bottle_data>("vision/bottle_data_arm_right",1000);

}


/**
 * This node is in charge of publishing pose data
 */
int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "vision_pose_publisher");

  TeleopKeyboard teleop_robot;
  teleop_robot.loop();



  return(0);

  ///TODO: to create a namespace and check if it is working properly, create the listener.. and create the proper messages
}


void TeleopKeyboard::loop()
{
    char c;
int type;
    bool key_pressed = false;
    vision::bottle_data bottle_info;
    int quit = 0;

    while(quit==0)
    {
 	   std::cout << "---------------------------" << "\n";
            std::cout << "Ready to send pose?" << "\n";
            std::cout << "what type of bottle is on the table? (0,1,2):";
            std::cin >> type;
            std::cout << type;
	    std::cout << "Press 'p' to send a predefined pose or'q' to quit :";
	    std::cin >> c;
            switch (c)
            {
                case KEYCODE_Q:
                    quit=1;
                    break;
		            case KEYCODE_P:
                
                        //Left arm
                        /*bottle_info.P.pose.orientation.w = -0.011;
                        bottle_info.P.pose.orientation.x =  0.7090;
                        bottle_info.P.pose.orientation.y = 0.7050;
                        bottle_info.P.pose.orientation.z =  0.01055;
                        bottle_info.P.pose.position.x =  0.7;
                        bottle_info.P.pose.position.y = 0.4;
                        bottle_info.P.pose.position.z =  1.05;
                        bottle_info.height = 0.229;
                        bottle_info.width = 0.0099;
                        bottle_info.type = type;*/
                        
                        //Right arm
                        bottle_info.P.pose.orientation.w = -0.011;
                        bottle_info.P.pose.orientation.x =  0.7090;
                        bottle_info.P.pose.orientation.y = 0.7050;
                        bottle_info.P.pose.orientation.z =  0.01055;
                        bottle_info.P.pose.position.x =  0.7;
                        bottle_info.P.pose.position.y = -0.4;
                        bottle_info.P.pose.position.z =  1.05;
                        bottle_info.length = 0.229;
                        bottle_info.width = 0.0099;
                        bottle_info.type = type;


                        bottle_data_pub.publish(bottle_info);

                        std::cout << "Publishing" << bottle_info;
                        quit=0;
                    break;
                    
                default:
                        quit=1;
                    break;
            }
           

    }
}
