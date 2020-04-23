//  ///////////////////////////////////////////////////////////
//
// turtlebot_square_node.cpp
// This file contains code for use with MTE 544 Lab 1
// It moves the turtlebot in a square within the Gazebo simulator
// 
// Author: Sean Kim
//
// //////////////////////////////////////////////////////////

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <cmath>

double X; 
double Y;
double yaw;

void pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
	//This function is called when a new position message is received

	X = msg->pose.pose.position.x; // Robot X postition
	Y = msg->pose.pose.position.y; // Robot Y postition
 	yaw = tf::getYaw(msg->pose.pose.orientation); // Robot yaw

}

int main(int argc, char **argv)
{
	//Initialize the ROS framework
    ros::init(argc,argv,"main_control");
    ros::NodeHandle n;

    //Subscribe to the desired topics and assign callbacks
    ros::Subscriber pose_sub = n.subscribe("/amcl_pose", 1, pose_callback);
    
    //Setup topics to Publish from this node
    ros::Publisher velocity_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 1);
   
    //Velocity control variable
    geometry_msgs::Twist vel;

    //Setting starting point variable 
    bool start = false;
    bool cali = false;
    double startX; 
    double startY; 
    double startYaw;
    

    //Set the loop rate
    ros::Rate loop_rate(20);    //20Hz update rate

    while (ros::ok())
    {
    	loop_rate.sleep(); //Maintain the loop rate
    	ros::spinOnce();   //Check for new messages
        
		if (!cali) {
			while (fabs(yaw) > 0.1) {
				vel.angular.z = -0.1;
				vel.linear.x = 0;
				velocity_publisher.publish(vel);
				ros::spinOnce(); //Check for new messages
			}
			cali = true; 
			ROS_INFO("Calibrated yaw Position: %f", yaw);
			ROS_INFO("Abs Value: %f",fabs(yaw)); 
		}	

		if (!start) {
			startX = X; 
			startY = Y;
			startYaw = yaw;
			ROS_INFO("startYaw: %f", startYaw);			
			start = true;
		}
		if (/*fabs(startX-X)<0.5 && fabs(startY-Y)<0.5*/sqrt(pow(startX-X,2)+pow(startY-Y,2))<0.5) {
			vel.linear.x = 0.2;
			vel.angular.z = 0;
			velocity_publisher.publish(vel);
			ROS_INFO("startX: %f, currentX: %f", startX, X);
		}

		else if (fabs(startYaw-yaw)>M_PI) {
			startYaw+=M_PI;	
		}
		else if (fabs(startYaw-yaw)<M_PI/2){ //0.785 ~= pi/4*2
		
		vel.angular.z = -0.2;
		vel.linear.x = 0;
		velocity_publisher.publish(vel);
		ROS_INFO("startYaw: %f, currentYaw: %f", startYaw, yaw);
		}	 
		else {
		ROS_INFO("startYaw: %f, currentYaw: %f", startYaw, yaw); 
		start = false;
		}
		ROS_INFO("startX: %f, currentX: %f", startX, X);
		ROS_INFO("startY: %f, currentY: %f", startY, Y);			
    }

    return 0;
}
