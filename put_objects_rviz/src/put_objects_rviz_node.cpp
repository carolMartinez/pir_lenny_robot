#include <ros/ros.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/PlanningScene.h>
#include <geometric_shapes/shape_operations.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AllowedCollisionMatrix.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <moveit/kinematic_constraints/utils.h>
#include <tf/transform_listener.h>
#include <boost/assign/std/vector.hpp>
#include <iostream>

#include <vision/bottle_data.h>

    class CollisionObjectAdder
    {
        public:
            CollisionObjectAdder();
            void addCollisionObject(const vision::bottle_data::ConstPtr &bottle_msg);
            void addCollisionObject();
            
            void publishTf();

        private:

            ros::NodeHandle nh;
            ros::Publisher add_collision_object_pub;
            ros::Publisher visionData_pub;
            
            tf2_ros::TransformBroadcaster tfb;
            geometry_msgs::TransformStamped transformStamped;
            geometry_msgs::Pose box_pose, box_pose_rotated;
            ros::Subscriber subPose;
            int receivedMessage;

         //ros::Publisher planning_scene_diff_pub;
    };



       CollisionObjectAdder::CollisionObjectAdder()
       {
            add_collision_object_pub = nh.advertise<moveit_msgs::CollisionObject>("collision_object", 1000);
            subPose = nh.subscribe("vision/bottle_data_arm_right",1000,&CollisionObjectAdder::addCollisionObject,this);

            box_pose.orientation.w = 0;
                   box_pose.orientation.x =  0;
                   box_pose.orientation.y = 0;
                   box_pose.orientation.z =  0;
                   box_pose.position.x =  0;
                   box_pose.position.y = 0;
                   box_pose.position.z =  0;

                   receivedMessage = 0;
                   ROS_INFO("ACAAAAA");

       }

       void CollisionObjectAdder::addCollisionObject(const vision::bottle_data::ConstPtr &bottle_msg)
       {
           
           
           sleep(1.0); // To make sure the node can publish
           moveit_msgs::CollisionObject co;
           moveit_msgs::CollisionObject co2;
           moveit_msgs::CollisionObject bandaCO;


           moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

           box_pose = bottle_msg->P.pose;



           receivedMessage = 1;
///For broadcasting transformation
	  


    /*      shapes::Mesh* banda = shapes::createMeshFromResource("package://put_objects_rviz/stl/BANDA.STL");
           shape_msgs::Mesh bandaMesh;
           shapes::ShapeMsg banda_mesh_msg;
           shapes::constructMsgFromShape(banda,banda_mesh_msg);
           //bandaCO.mesh_poses.resize(1);
           bandaMesh = boost::get<shape_msgs::Mesh>(banda_mesh_msg);

           bandaCO.mesh_poses.resize(1);
            ROS_INFO("mesh loaded");
         // bandaCO.meshes[0] = bandaMesh;
               ROS_INFO("mesh loaded2");
           bandaCO.meshes.resize(2);
           //bandaCO.mesh_poses.resize(1);

           bandaCO.mesh_poses[0].position.x = 0.0;
           bandaCO.mesh_poses[0].position.y = 0.0;
           bandaCO.mesh_poses[0].position.z = 0.0;
           bandaCO.mesh_poses[0].orientation.w= 0.0;
           bandaCO.mesh_poses[0].orientation.x= 0.0;
           bandaCO.mesh_poses[0].orientation.y= 0.0;
           bandaCO.mesh_poses[0].orientation.z= 0.0;
           bandaCO.header.frame_id = "/base_link";
           bandaCO.header.stamp = ros::Time::now();
           bandaCO.id = "BANDA";


           /* A pose for the box (specified relative to frame_id) */
/*           geometry_msgs::Pose banda_pose;
           banda_pose.orientation.w = 0;
           banda_pose.position.x =  0;
           banda_pose.position.y = 0;
           banda_pose.position.z =  0;


           bandaCO.operation = bandaCO.ADD;
           bandaCO.meshes.push_back(bandaMesh);
           bandaCO.mesh_poses.push_back(banda_pose);

           add_collision_object_pub.publish(bandaCO);
*/
           shape_msgs::SolidPrimitive primitive;
           primitive.type = primitive.BOX;
           primitive.dimensions.resize(3);
           primitive.dimensions[0] = 0.2;
           primitive.dimensions[1] = 3.0;
           primitive.dimensions[2] = 0.5;

 /*          co.header.frame_id = "/base_link";
           co.header.stamp = ros::Time::now();
           co.id = "TABLE";

          /* A pose for the box (specified relative to frame_id) */
/*          geometry_msgs::Pose table_pose;
          table_pose.orientation.w = 0;
          table_pose.position.x =  0.5;
          table_pose.position.y = 0;
          table_pose.position.z =  0;

   //     NEW CODE, SOLUTION TO THE PROBLEM For the mesh
         /* co.mesh_poses[0].position.x = 0.0;
          co.mesh_poses[0].position.y = 0.0;
          co.mesh_poses[0].position.z = -10.0;
          co.mesh_poses[0].orientation.w= 1.0;
          co.mesh_poses[0].orientation.x= 0.0;
          co.mesh_poses[0].orientation.y= 0.0;
          co.mesh_poses[0].orientation.z= 0.0;   */

          //co.meshes.push_back(co_mesh);
          //co.mesh_poses.push_back(co.mesh_poses[0]);
/*
          co.operation = co.ADD;
          co.primitives.push_back(primitive);
          co.primitive_poses.push_back(table_pose);

          add_collision_object_pub.publish(co);
*/
          //ADDING SECOND OBJECT
          shape_msgs::SolidPrimitive primitive2;
          primitive2.type = primitive.CYLINDER;
          primitive2.dimensions.resize(2);
          primitive2.dimensions[0] = 0.2; //height
          primitive2.dimensions[1] = 0.005; //radius
          primitive2.dimensions[2] = 0.3;


         co2.header.frame_id = "/torso_base_link";

         co2.header.stamp = ros::Time::now();
         co2.id = "bottle_right_arm";

         /* A pose for the box (specified relative to frame_id) */
         //Carol's simulated bottle
        /* box_pose.orientation.w = 0.7071;
         box_pose.orientation.x =  0;
         box_pose.orientation.y = 0.7071;
         box_pose.orientation.z =  0;
        box_pose.position.x =  -0.1;
        box_pose.position.y = 0.7;
        box_pose.position.z =  0.7;
*/


         //Nicolas bottle
        box_pose.orientation.w = -0.011;
         box_pose.orientation.x =  0.7090;
         box_pose.orientation.y = 0.7050;
         box_pose.orientation.z =  0.01055;
         box_pose.position.x =  0.17;
         box_pose.position.y = 0.56;
         box_pose.position.z =  0.61;


         tf::Transform box_rotated_tf; ///Objecto coor wrt world
         tf::Transform box_tf; ///Objecto coor wrt world

         tf::poseMsgToTF(box_pose,box_tf);


         box_rotated_tf = box_tf*tf::Transform(tf::Quaternion(0.7071,0,0,0.7071),tf::Vector3(0,0,0));

         box_pose_rotated.orientation.w = box_rotated_tf.getRotation().getW();
         box_pose_rotated.orientation.x = box_rotated_tf.getRotation().getX();
         box_pose_rotated.orientation.y = box_rotated_tf.getRotation().getY();
         box_pose_rotated.orientation.z = box_rotated_tf.getRotation().getZ();
         box_pose_rotated.position.x =  box_rotated_tf.getOrigin()[0];
         box_pose_rotated.position.y = box_rotated_tf.getOrigin()[1];
         box_pose_rotated.position.z =  box_rotated_tf.getOrigin()[2];


      /*  transformStamped.transform.translation.x = box_pose.position.x;
        transformStamped.transform.translation.y = box_pose.position.y;
        transformStamped.transform.translation.z = box_pose.position.z;
        tf2::Quaternion q;
        q.setRPY(0 ,0,0);
        transformStamped.transform.rotation.x =  box_pose.orientation.x;
        transformStamped.transform.rotation.y =  box_pose.orientation.y;
        transformStamped.transform.rotation.z =  box_pose.orientation.z;
        transformStamped.transform.rotation.w = box_pose.orientation.w;
*/
       //ros::Time::init();

         co2.operation = co2.ADD;
         co2.primitives.push_back(primitive2);
        // co2.primitive_poses.push_back(box_pose);
         co2.primitive_poses.push_back(box_pose_rotated);

         add_collision_object_pub.publish(co2);

   

          ROS_INFO("Collision object published");
          ros::Duration(0.5).sleep();

          ROS_INFO("Add an object into the world");
          std::vector<moveit_msgs::CollisionObject> collision_objects;
          collision_objects.push_back(co2);
           co2.operation = co2.MOVE;
          collision_objects.push_back(co2);

planning_scene_interface.addCollisionObjects(collision_objects); //add objects to planning interface
  
          /* Sleep so we have time to see the object in RViz */
          sleep(100.0);





       }

       void CollisionObjectAdder::publishTf()
       {
           ///Converts pose message to tf Do not know why
           tf::Transform box_tf; ///Objecto coor wrt world
           tf::Transform box_rotated_tf; ///Objecto coor wrt world
           tf::Transform box_rotation; ///Objecto coor wrt world

           tf::poseMsgToTF(box_pose,box_tf);

            if (receivedMessage == 1)
            {

              //NOObox_rotated_tf = box_tf*tf::Transform(tf::Quaternion(0.5,-0.5,0.5,0.5),tf::Vector3(0,0,0));
               //box_rotated_tf = box_tf*tf::Transform(tf::Quaternion(0,0,-0.7071,1.0),tf::Vector3(0,0,0));
              box_rotated_tf = box_tf*tf::Transform(tf::Quaternion(0,0,0,1.0),tf::Vector3(0,0,0));


               ///For broadcasting tfs
               transformStamped.header.frame_id = "torso_base_link";
               transformStamped.child_frame_id = "object_link";
               transformStamped.transform.translation.x =  box_rotated_tf.getOrigin()[0];
               transformStamped.transform.translation.y = box_rotated_tf.getOrigin()[1];
               transformStamped.transform.translation.z = box_rotated_tf.getOrigin()[2];
               tf2::Quaternion q;
               q.setRPY(0 ,0,0);
               transformStamped.transform.rotation.x =  box_rotated_tf.getRotation().getX();
               transformStamped.transform.rotation.y =  box_rotated_tf.getRotation().getY();
               transformStamped.transform.rotation.z =  box_rotated_tf.getRotation().getZ();
               transformStamped.transform.rotation.w =  box_rotated_tf.getRotation().getW();


               transformStamped.header.stamp = ros::Time::now();

                tfb.sendTransform(transformStamped);
            }



       }



       void CollisionObjectAdder::addCollisionObject()
       {
         
           box_pose.orientation.w = 0;
           box_pose.orientation.x =  0;
           box_pose.orientation.y = 0;
           box_pose.orientation.z =  0;
           box_pose.position.x =  0;
           box_pose.position.y = 0;
           box_pose.position.z =  0;

           receivedMessage = 1;
         
           sleep(1.0); // To make sure the node can publish
           
           moveit_msgs::CollisionObject co;
           moveit_msgs::CollisionObject co2;
           moveit_msgs::CollisionObject bandaCO;


           moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

           
	         shape_msgs::SolidPrimitive primitive;
           primitive.type = primitive.BOX;
           primitive.dimensions.resize(3);
           primitive.dimensions[0] = 0.2;
           primitive.dimensions[1] = 3.0;
           primitive.dimensions[2] = 0.5;
    
          shape_msgs::SolidPrimitive primitive2;
          primitive2.type = primitive.CYLINDER;
          primitive2.dimensions.resize(2);
          primitive2.dimensions[0] = 0.2; //height
          primitive2.dimensions[1] = 0.005; //radius
          //primitive2.dimensions[2] = 0.3;


         
         co2.header.frame_id = "/torso_base_link";

         co2.header.stamp = ros::Time::now();
         co2.id = "bottle_right_arm";
          
         std::cout<<"Box Pose Created" << std::endl;


         //Nicolas bottle
         box_pose.orientation.w = -0.011;
         box_pose.orientation.x =  0.7090;
         box_pose.orientation.y = 0.7050;
         box_pose.orientation.z =  0.01055;
         box_pose.position.x =  0.47;
         box_pose.position.y = 0.02;
         box_pose.position.z =  1.05;


         tf::Transform box_rotated_tf; ///Objecto coor wrt world
         tf::Transform box_tf; ///Objecto coor wrt world

         tf::poseMsgToTF(box_pose,box_tf);


         box_rotated_tf = box_tf*tf::Transform(tf::Quaternion(0.7071,0,0,0.7071),tf::Vector3(0,0,0));
          
         
         box_pose_rotated.orientation.w = box_rotated_tf.getRotation().getW();
         box_pose_rotated.orientation.x = box_rotated_tf.getRotation().getX();
         box_pose_rotated.orientation.y = box_rotated_tf.getRotation().getY();
         box_pose_rotated.orientation.z = box_rotated_tf.getRotation().getZ();
         box_pose_rotated.position.x =  box_rotated_tf.getOrigin()[0];
         box_pose_rotated.position.y = box_rotated_tf.getOrigin()[1];
         box_pose_rotated.position.z =  box_rotated_tf.getOrigin()[2];


      /*  transformStamped.transform.translation.x = box_pose.position.x;
        transformStamped.transform.translation.y = box_pose.position.y;
        transformStamped.transform.translation.z = box_pose.position.z;
        tf2::Quaternion q;
        q.setRPY(0 ,0,0);
        transformStamped.transform.rotation.x =  box_pose.orientation.x;
        transformStamped.transform.rotation.y =  box_pose.orientation.y;
        transformStamped.transform.rotation.z =  box_pose.orientation.z;
        transformStamped.transform.rotation.w = box_pose.orientation.w;
*/
       //ros::Time::init();

         co2.operation = co2.ADD;
         co2.primitives.push_back(primitive2);
        // co2.primitive_poses.push_back(box_pose);
         co2.primitive_poses.push_back(box_pose_rotated);

         add_collision_object_pub.publish(co2);


          ROS_INFO("Collision object published");
          ros::Duration(0.5).sleep();

          ROS_INFO("Add an object into the world");
          std::vector<moveit_msgs::CollisionObject> collision_objects;
          collision_objects.push_back(co2);
           co2.operation = co2.MOVE;
          collision_objects.push_back(co2);

          
  planning_scene_interface.addCollisionObjects(collision_objects); //add objects to planning interface
  
  

          /* Sleep so we have time to see the object in RViz */
          sleep(5.0);


                  



       }


    int main(int argc, char** argv)
    {
      
            int options = 2; 
            // 1 for using a published pose from vision system
            // 2 for using a defined pose
            
            
            ros::init(argc, argv, "add_collision_object");
            ros::NodeHandle node;
            std::cout<<"Initialized..." << std::endl;
            CollisionObjectAdder coAdder;

            if (options == 1)
            {
              ros::AsyncSpinner spinner(3);
              spinner.start();
            }
             

            if (options == 2)
            {
            
              coAdder.addCollisionObject();
              // coAdder.publishTf();
            //  ros::spin();
            }
            ros::Rate rate(1.0);
            
            while(node.ok()){
                   coAdder.publishTf();
                    rate.sleep();
            }



            ros::spin();

            return 0;
    }
