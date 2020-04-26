//  ///////////////////////////////////////////////////////////
//
// turtlebot_example.cpp
// This file contains example code for use with ME 597 lab 2
// It outlines the basic setup of a ros node and the various 
// inputs and outputs needed for this lab
// 
// Author: James Servos 
//
// //////////////////////////////////////////////////////////

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <gazebo_msgs/ModelStates.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <eigen3/Eigen/Dense>

#include <algorithm>
#include <utility>
#include <vector>
#include <cmath>
#include <stdint.h>
#include <stdlib.h>

#include "shortestpath.cpp"
#include "carrot_controller.cpp"

typedef struct pose {
  double x;
  double y;
  double theta;
} pose_t;


//  Example usage of std::normal_distribution
#define MEAN 10.0
#define STDDEV 2.0

std::default_random_engine generator; //setup a random number generator
std::normal_distribution<double> distribution(MEAN,STDDEV); //setup the normal distribution
double number = distribution(generator); //get random number

#define TAGID 0
#define NUM_SAMPLES 1000
#define OCCUPIED_THRESHOLD 1
#define NEAREST_MS 80

using namespace Eigen;


typedef struct mapData {
  float res;
  uint32_t w;
  uint32_t h;
  struct pose origin;
  std::string frame_id;
} mapData_t;

ros::Publisher marker_pub;
geometry_msgs::Twist vel;

bool mapRecieved = false;
char **occupancyGrid; // map represented as occupancy probabilities between 0-100
std::vector<std::vector<unsigned char>> edges; // unwieghted graph
MatrixXi edges;
mapData_t mapInfo; // map meta data
MatrixXd mapSamples; // random samples in map
Vector3d X; // pose
std::vector<pose_t> milestones;
MatrixXd ms; // matrix for milestones
std::vector<int> sp;

// Hardcoded waypoints
pose_t wp1 = {4.0, 0.0, 0.0};
pose_t wp2 = {8.0, -4.0, 3.14};
pose_t wp3 = {8.0, 0.0, -1.57};

//bool edge_collision(int x0, int y0, int x1, int y1);
bool edge_collision(double x0, double y0, double x1, double y1);
void bresenham(int x0, int y0, int x1, int y1, std::vector<int> &x, std::vector<int> &y);
void point_publisher();
std::vector<int> shortestpath(MatrixXi edges, int start, int finish);
void carrot_controller(ros::Publisher velocity_publisher, int n, std::vector<pose_t> W, std::vector<int>sp, ros::Rate loop_rate); //n is number of waypoints

short sgn(int x) { return x >= 0 ? 1 : -1; }
double getDist(double x0, double y0, double x1, double y1) {return std::sqrt( std::pow(y1-y0,2) + pow(x1-x0,2));}
double getDist(pose_t p0, pose_t p1) {return std::sqrt( std::pow(p1.y-p0.y,2) + pow(p1.x-p0.x,2));}

//Callback function for the Position topic (LIVE)
void pose_callback(const geometry_msgs::PoseWithCovarianceStamped & msg)
{
  //This function is called when a new position message is received
  double x = msg.pose.pose.position.x; // Robot X position
  double y = msg.pose.pose.position.y; // Robot Y position
  double yaw = tf::getYaw(msg.pose.pose.orientation); // Robot Yaw
  
  X << x, y, yaw;
}

void drawShortPath(double x0, double y0, double x1, double y1) 
{
  // Curves are drawn as a series of stright lines
  // Simply sample your curves into a series of points
  double x = 0;
  double y = 0;
  double steps = 25;
  static int counter = 0;
  visualization_msgs::Marker lines;
  lines.header.frame_id = "/map";
  lines.id = counter; counter++; //each curve must have a unique id or you will overwrite an old ones
  lines.type = visualization_msgs::Marker::LINE_STRIP;
  lines.action = visualization_msgs::Marker::ADD;
  lines.ns = "curves";
  lines.scale.x = 0.1;
  lines.color.r = 0.5;
  lines.color.b = 0.0;
  lines.color.g = 1.0;
  lines.color.a = 1.0;

  geometry_msgs::Point p1;
  p1.x = x0;
  p1.y = y0;
  p1.z = 0;
  geometry_msgs::Point p3;
  p3.x = x1;
  p3.y = y1;
  p3.z = 0;

  lines.points.push_back(p1);
  lines.points.push_back(p3);

  //publish new curve
  marker_pub.publish(lines);

}

void drawCurve(double x0, double y0, double x1, double y1) 
{
  // Curves are drawn as a series of stright lines
  // Simply sample your curves into a series of points
  double x = 0;
  double y = 0;
  double steps = 25;
  static int counter = 0;
  visualization_msgs::Marker lines;
  lines.header.frame_id = "/map";
  lines.id = counter; counter++; //each curve must have a unique id or you will overwrite an old ones
  lines.type = visualization_msgs::Marker::LINE_STRIP;
  lines.action = visualization_msgs::Marker::ADD;
  lines.ns = "curves";
  lines.scale.x = 0.03;
  lines.color.r = 0.2;
  lines.color.b = 1.0;
  lines.color.a = 1.0;

  geometry_msgs::Point p1;
  p1.x = x0;
  p1.y = y0;
  p1.z = 0;
  geometry_msgs::Point p3;
  p3.x = x1;
  p3.y = y1;
  p3.z = 0;

  lines.points.push_back(p1);
  lines.points.push_back(p3);

  //publish new curve
  marker_pub.publish(lines);

}



//Callback function for the map
void map_callback(const nav_msgs::OccupancyGrid& msg)
{
  //This function is called when a new map is received
  //you probably want to save the map into a form which is easy to work with
  mapInfo.frame_id = msg.header.frame_id;
  mapInfo.res = msg.info.resolution;
  mapInfo.w = msg.info.width;
  mapInfo.h = msg.info.height;
  mapInfo.origin.x = msg.info.origin.position.x;
  mapInfo.origin.y = msg.info.origin.position.y;
  mapInfo.origin.theta = tf::getYaw(msg.info.origin.orientation);
  occupancyGrid = (char**)malloc(mapInfo.h*sizeof(char*));

  for (auto i = 0; i < mapInfo.h; i++) {
    occupancyGrid[i] = (char*)malloc(mapInfo.w);
    memcpy(occupancyGrid[i], &msg.data[i*mapInfo.w], mapInfo.w);
  }

  mapRecieved = true;
}

void drawPoints() {

  visualization_msgs::Marker points;
  points.header.stamp = ros::Time::now();
  points.header.frame_id = "/map";
  points.ns = "milestones";
  points.action = visualization_msgs::Marker::ADD;
  points.id = 0;
  points.type = visualization_msgs::Marker::POINTS;
  points.pose.orientation.w = 1.0;
  points.scale.x = 0.07;
  points.scale.y = 0.07;
  points.color.g = 0.2;
  points.color.b = 0.5;
  points.color.r = 1.0;
  points.color.a = 1.0;

  geometry_msgs::Point p;
  for (int j = 0; j < milestones.size(); j++) {
    p.x = milestones[j].x;
    p.y = milestones[j].y;
    p.z = 0;
    points.points.push_back(p);
  }
  marker_pub.publish(points);
}

void prm(ros::Rate &loop_rate) {


  // Waits for map to be recieved
  while (ros::ok() && !mapRecieved) {
    loop_rate.sleep();
    ros::spinOnce();
  }

  ROS_INFO_STREAM("Map Recieved: height = " << mapInfo.h << " width = " << mapInfo.w << " res = " << mapInfo.res <<
          " origin = (" << mapInfo.origin.x << ", " << mapInfo.origin.y << ", " << mapInfo.origin.theta << ")\n");

  // Samples random configurations and stores collision free ones in milestones vector
  mapSamples = MatrixXd::Random(NUM_SAMPLES, 2);
  milestones.push_back(pose_t {X(0), X(1), X(2)});
  milestones.push_back(wp1);
  milestones.push_back(wp2);
  milestones.push_back(wp3);
  MatrixXf::Index maxRow, maxCol;
  double maxC = mapSamples.maxCoeff(&maxRow, &maxCol);

  for (auto i = 0; i < NUM_SAMPLES; i++) {
    mapSamples(i,0) = (mapSamples(i,0) + 1) * (mapInfo.w/2.0);
    mapSamples(i,1) = (mapSamples(i,1) + 1) * (mapInfo.h/2.0);
    auto xSample = ((int)round(mapSamples(i,0)) >= mapInfo.w) ? mapInfo.w - 1 : (int)round(mapSamples(i,0));
    auto ySample = ((int)round(mapSamples(i,1)) >= mapInfo.h) ? mapInfo.h - 1 : (int)round(mapSamples(i,1));
    //ROS_INFO_STREAM("grid values (" << occupancyGrid[xSample][ySample] << ")\n");
    if (!occupancyGrid[ySample][xSample]) {
      auto tfY = (double)((ySample + mapInfo.origin.y/mapInfo.res) * mapInfo.res); 
      auto tfX = (double)((xSample + mapInfo.origin.x/mapInfo.res) * mapInfo.res); 
      
      milestones.push_back({tfX, tfY, 0});
    }
  }

  ROS_INFO_STREAM("Number of milestone: " << milestones.size() << "\n");

  drawPoints();
  /**
   * nnHeap vector
   *
   * Stores the smallest distances and corresponding indices within the milestones array
   *
   * for milestone i:
   *    nnHeap[j].first - distance of jth nearest milestone
   *    nnHeap[j].second - index in milestones array for jth nearest milestone
   **/

  std::vector<std::pair<double, int>> nnHeap; // max heap used for only keeping edges connected to nearest neighbours
  auto nM = milestones.size();
  double d = 0;
  edges = MatrixXi::Zero(nM, nM);

  for (auto i = 0; i < nM; i++) {
    nnHeap.clear();
    for (auto j = 0; j < nM; j++) {
      d = getDist(milestones[i], milestones[j]);
      if (nnHeap.size() < NEAREST_MS && j != i) {
        nnHeap.push_back(std::make_pair(d, j));
        if (nnHeap.size() == NEAREST_MS) std::make_heap(nnHeap.begin(), nnHeap.end());
      } else if (j != i && d < nnHeap[0].first) {
        std::pop_heap(nnHeap.begin(), nnHeap.end());
        nnHeap.pop_back();
        nnHeap.push_back(std::make_pair(d, j));
        std::push_heap(nnHeap.begin(), nnHeap.end());
      }
    }
      std::sort_heap(nnHeap.begin(), nnHeap.end());

  // - Find closest milestones -- DONE
  //    - Check if edge collides with any obstacles
  //    - Only keep edges that are in collision free zones
  //    - Put collision free edges in graph 2d array

    for (auto j = 0; j < nnHeap.size(); j++) {
      if ((i < nnHeap[j].second) ) {
        if (!(edge_collision(milestones[i].x, milestones[i].y, milestones[nnHeap[j].second].x, milestones[nnHeap[j].second].y) ) ) {
          edges(i, nnHeap[j].second) = 1;
          edges(nnHeap[j].second, i) = 1; 
        }
      } 
    }
  }
  
  // Find shortest path from edges and milestones objects
  auto sp1 = shortestpath(edges, 0, 1);
  auto sp2 = shortestpath(edges, 1, 2);
  auto sp3 = shortestpath(edges, 2, 3);
  sp.reserve(sp1.size()+sp2.size()+sp3.size());
  sp.insert(sp.end(), sp1.begin(), sp1.end());
  sp.insert(sp.end(), sp2.begin(), sp2.end());
  sp.insert(sp.end(), sp3.begin(), sp3.end());
  sp.push_back(3);
  ROS_INFO_STREAM("Final Path: ");
  for (auto i = 0; i < sp.size(); i++) {
    ROS_INFO_STREAM(sp[i] << ", ");
  }
  point_publisher();
}

void point_publisher() {
  visualization_msgs::Marker points;
  points.header.stamp = ros::Time::now();
  points.header.frame_id = "/map";
  points.ns = "shortest_path";
  points.action = visualization_msgs::Marker::ADD;
  points.id = 0;
  points.type = visualization_msgs::Marker::POINTS;
  points.pose.orientation.w = 1.0;
  points.scale.x = 0.1;
  points.scale.y = 0.1;
  points.color.g = 1.0f;
  points.color.a = 1.0;

  geometry_msgs::Point p;
  for (int j = 0; j < sp.size(); j++) {
    p.x = milestones[sp[j]].x;
    p.y = milestones[sp[j]].y;
    p.z = 0;
    points.points.push_back(p);
  }
  
  for (int i = 0; i < sp.size()-1; ++i)
  {
    drawShortPath(milestones[sp[i]].x, milestones[sp[i]].y, milestones[sp[i+1]].x, milestones[sp[i+1]].y);
  }

  marker_pub.publish(points);
}

int main(int argc, char **argv)
{
  //Initialize the ROS framework
  ros::init(argc,argv,"main_control");
  ros::NodeHandle n;

  //Subscribe to the desired topics and assign callbacks
  ros::Subscriber map_sub = n.subscribe("/map", 1, map_callback);
  ros::Subscriber pose_sub = n.subscribe("/indoor_pos", 1, pose_callback);
  //Setup topics to Publish from this node
  ros::Publisher velocity_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1, true);
  
  //Velocity control variable
  geometry_msgs::Twist vel;

  //Set the loop rate
  ros::Rate loop_rate(20);    //20Hz update rate
  
  prm(loop_rate);

  while (ros::ok())   {
    loop_rate.sleep(); //Maintain the loop rate
    ros::spinOnce();   //Check for new messages

    carrot_controller(velocity_publisher, sp.size(), milestones, sp, loop_rate);
    break;  
  }
    return 0;
}


//function to detect collision 
// x0 y0 is the starting points and x1 and y1 are the end points
//will return true if collision is detected
bool edge_collision(double x0, double y0, double x1, double y1) {
  std::vector<int> x;
  std::vector<int> y;
  auto x0i = (int)round((x0-mapInfo.origin.x)/mapInfo.res);
  auto y0i = (int)round((y0-mapInfo.origin.y)/mapInfo.res);
  auto x1i = (int)round((x1-mapInfo.origin.x)/mapInfo.res);
  auto y1i = (int)round((y1-mapInfo.origin.y)/mapInfo.res);

  ROS_INFO("start X AND start  Y IS %f and %f, end x and y are %f and %f ",x0,y0,x1,y1);
  bresenham(x0i, y0i, x1i, y1i, x, y);
  for (auto i = 0; i < x.size(); i++) {
    if (occupancyGrid[y[i]][x[i]]) {
      return true;
    }
  }
  return false;
}

//Bresenham line algorithm (pass empty vectors)
// Usage: (x0, y0) is the first point and (x1, y1) is the second point. The calculated
//        points (x, y) are stored in the x and y vector. x and y should be empty
//    vectors of integers and shold be defined where this function is called from.
void bresenham(int x0, int y0, int x1, int y1, std::vector<int> &x, std::vector<int> &y) {

    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    int dx2 = x1 - x0;
    int dy2 = y1 - y0;

    const bool s = abs(dy) > abs(dx);

    if (s) {
        int dx2 = dx;
        dx = dy;
        dy = dx2;
    }

    int inc1 = 2 * dy;
    int d = inc1 - dx;
    int inc2 = d - dx;

    x.push_back(x0);
    y.push_back(y0);

    while (x0 != x1 || y0 != y1) {
        if (s)
            y0 += sgn(dy2);
        else
            x0 += sgn(dx2);
        if (d < 0)
            d += inc1;
        else {
            d += inc2;
            if (s)
                x0 += sgn(dx2);
            else
                y0 += sgn(dy2);
        }

        //Add point to vector
        x.push_back(x0);
        y.push_back(y0);
    }
}

std::vector<int> shortestpath(MatrixXi edges, int start, int finish){
  //int n = nodes.rows();
  auto n = milestones.size();
  MatrixXd dists = MatrixXd::Zero(n,n);
  int count = 0;
  for(int i = 0; i < n; i++){
    for(int j = i; j < n; j++){ 
      if (edges(i,j)) {
        dists(i,j) = getDist(milestones[i], milestones[j]);
        dists(j,i) = dists(i,j);        
      }
    }
  }
  return dijkstra(dists, start, n, finish);  
}
