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

int len(MatrixXd nodes, int i, int j){
  return sqrt(pow(nodes(i,0)-nodes(j,0),2)+pow(nodes(i,1)-nodes(j,1),2));
}

int minimum(MatrixXd array, int index){
  int min = array(0,0);
  int rows =  array.rows();
  for(int i = 1; i < rows; i++){
    if(array(i,index) < min){
      min = array(i,0);
    }
  }
  return min;
}

int minDistance(int dist[], bool sptSet[], int n) 
{ 
  // Initialize min value 
  int min = INT_MAX; 
  int min_index; 

  for (int v = 0; v < n; v++) {
    if (sptSet[v] == false && dist[v] <= min) {
      min = dist[v]; 
      min_index = v;     
    } 
  }
  return min_index; 
} 

// Function to print shortest path from source to j using parent array (debugging purposes)
void printPath(VectorXd parent, int j) 
{ 
    // Base Case : If j is source 
    if (parent(j) == -1) {
      return;
    }
  
    printPath(parent, parent(j)); 
    ROS_INFO("%d ", j);
} 
  
// A utility function to print the constructed distance array 
int printSolution(int dist[], int n, VectorXd parent, int src) 
{ 
    ROS_INFO("Vertex\t Distance\tPath"); 
    for (int i = 1; i < n; i++) 
    { 
        ROS_INFO("%d -> %d \t %d\t\t%d ", src, i, dist[i], src); 
        printPath(parent, i); 
    } 
} 

void path_store(VectorXd &parent, std::vector<int> &path, int j) {
  if (parent(j) == -1) {
    return;
  }
  path_store(parent, path, parent(j));
  path.push_back(parent(j));
}

std::vector<int> dijkstra(MatrixXd graph, int src, int n, int finish) 
{ 
  int j = finish;
    // The output array. dist[i] will hold the shortest distance from src to i 
    int dist[n];  
  
    // sptSet[i] will true if vertex i is included in shortest path tree or shortest distance from src to i is finalized 
    bool sptSet[n]; 
  
    // Parent array to store shortest path tree 
    VectorXd parent = VectorXd::Zero(n);   
  std::vector<int> path;// = VectorXd::Zero(n);

    // Initialize all distances as INFINITE and stpSet[] as false
    parent(src) = -1; 
    //ROS_INFO("src: %f", parent(1));
    for (int i = 0; i < n; i++) 
    { 
        dist[i] = INT_MAX; 
        sptSet[i] = false; 
    } 
    // Distance of source vertex from itself is always 0 
    dist[src] = 0; 
    // Find shortest path for all vertices 
    for (int count = 0; count <n-1; count++) 
    { 
        // Pick the minimum distance vertex from the set of vertices not yet processed.  
        // u is always equal to src in first iteration. 
        int u = minDistance(dist, sptSet, n);
  
        // Mark the picked vertex as processed 
        sptSet[u] = true; 
  
        // Update dist value of the adjacent vertices of the picked vertex. 
        for (int v = 0; v < n; v++) {
          // Update dist[v] only if is not in sptSet, there is an edge from u to v, and total weight of path from src to v through u is smaller than current value of dist[v] 
          if (!sptSet[v] && graph(u,v) && (dist[u] + graph(u,v)) < dist[v]) 
          { 
            parent(v) = u; 
            dist[v] = dist[u] + graph(u,v); 
          }  
        }
    } 
    path_store(parent,path,j);
  return path;
}

std::vector<int> shortestpath(MatrixXi edges, int start, int finish)
{
  auto n = milestones.size();
  MatrixXd dists = MatrixXd::Zero(n, n);
  int count = 0;
  for (int i = 0; i < n; i++)
  {
    for (int j = i; j < n; j++)
    {
      if (edges(i, j))
      {
        dists(i, j) = getDist(milestones[i], milestones[j]); //len(nodes,i,j);
        dists(j, i) = dists(i, j);
      }
    }
  }
  return dijkstra(dists, start, n, finish);
}