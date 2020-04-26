#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <gazebo_msgs/ModelStates.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <eigen3/Eigen/Dense>
#include <cmath>

using namespace Eigen;

//Velocity control variable
geometry_msgs::Twist vel;

void carrot_controller(ros::Publisher velocity_publisher, int n, std::vector<pose_t> W, std::vector<int>sp, ros::Rate loop_rate) { //n is number of waypoints
  //Constants
  double Kp = 1; 
  double zeta = 0.35; 
  double L = 0.10; // threshold radius for waypoints 

  // Waypoint constants
  double lin_speed = 0.3; // linear speed m/s

  ros::spinOnce();
  ROS_INFO("Initial X Position: %f, Initial Y Position: %f", X(0), X(1));
  ROS_INFO("Size of Path: %d",sp.size()); 
  //Loop through waypoints
  for (int i = 1; i < n-1; i++) {
    double cur_waypt_x = W[sp[i]].x;// waypoint that robot has already passed
    double cur_waypt_y = W[sp[i]].y;
    double next_waypt_x = W[sp[i+1]].x; // waypoint that robot is travelling to 
    double next_waypt_y = W[sp[i+1]].y;

    // Continue until 
    while (!(fabs(X(0)-next_waypt_x) < L && fabs(X(1)-next_waypt_y) < L) ) {
      double ru = sqrt( pow(cur_waypt_x-X(0),2) + pow(cur_waypt_y-X(1),2)); // norm of vector 
      double ang = atan2(next_waypt_y-cur_waypt_y,next_waypt_x-cur_waypt_x); // angle between waypoints
      double angu = atan2(X(1)-cur_waypt_y, X(0)-cur_waypt_x); // angle between robot and waypoint that robot has passed 
      double beta = ang-angu; 

      double r = sqrt(pow(ru,2) - pow(ru*sin(beta),2)); 
      double x_prime = (r+zeta)*cos(ang) + cur_waypt_x; // x-coordinate of carrot 
      double y_prime = (r+zeta)*sin(ang) + cur_waypt_y; // y-coordinate of carrot 

      double steer_des = atan2(y_prime-X(1), x_prime-X(0));
      double error = steer_des - X(2); 

      if (error < -M_PI)
        error += 2*M_PI; 
      else if (error > M_PI)
        error -= 2*M_PI; 

      double u = Kp*error;
      if (fabs(u) > 0.075)
        vel.linear.x = 0;
      else
        vel.linear.x = lin_speed; 
      vel.angular.z = u; 

      velocity_publisher.publish(vel);
      loop_rate.sleep(); //Maintain the loop rate
      point_publisher();
      ros::spinOnce();
      lin_speed = 0.3; 
    }
    ROS_INFO("Current X Position: %f", X(0));
    ROS_INFO("Current Y Position: %f", X(1));
    vel.linear.x = 0; 
    vel.angular.z = 0;
    velocity_publisher.publish(vel);
    ros::spinOnce();
  }
}