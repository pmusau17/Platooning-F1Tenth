#include "ros/ros.h"
#include <iostream>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <rtreach/stamped_ttc.h>
#include <rtreach/reach_tube.h>
#include <rtreach/interval.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <ros/package.h>
#include <ros/console.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <math.h>
#include <cstdlib>

// The following node will receive odometry estimates from a vehicle (velocity and position) and will compute the reachable set bounding box of where it
// could go based on it's current velocity 

const int max_hyper_rectangles = 8000;

extern "C"
{ 
    #include "simulate_obstacle.h"
    HyperRectangle runReachability_obstacle_vis(double* start, double simTime, double wallTimeMs, double startMs,double v_x, double v_y, HyperRectangle VisStates[],int  *total_intermediate,int max_intermediate,bool plot);
    HyperRectangle runReachability_obstacle_vis_uncertain(REAL start[][2], REAL simTime, REAL wallTimeMs, REAL startMs,REAL v_x, REAL v_y, HyperRectangle VisStates[],int  *total_intermediate,int max_intermediate,bool plot);
    double getSimulatedSafeTime(double start[2],double v_x, double v_y);
    HyperRectangle hr_list[max_hyper_rectangles];
}

ros::Publisher vis_pub;
ros::Publisher vis_pub2;
ros::Publisher tube_pub;
ros::Subscriber sub; // markerArray subscriber
ros::Subscriber sub2;
 
// reachability parameters
double sim_time;
double walltime; 
bool bloat_reachset = true;

// rect_count is used by the reachability algorithm to compue the reachsets
int rect_count;
int count = 0;

// initial state for estimate of vehicle position 
double startState[2] = {0.0, 0.0};
double startStateU[2][2] = { {0.0, 0.0}, {0.0, 0.0}};
// double starty[2]

// parameters for visualizing the obstacle reachsets
double display_max;
int display_count = 1;
double display_increment = 1.0;


// odometry message pointer
nav_msgs::Odometry::ConstPtr msg;

void box_pose_callback(const nav_msgs::Odometry::ConstPtr& nav_msg)
{
    
    msg = nav_msg;
    //ROS_WARN("got nav message");
    count++;
}


void visualize_position_uncertainty(const geometry_msgs::PoseArray::ConstPtr& pose_msg)
{
    int num_particles = pose_msg->poses.size();
    double roll, pitch, yaw;
    std::vector<geometry_msgs::Pose> poses = pose_msg->poses;
    double x_min = 1e9;
    double x_max =-1e9;
    double y_min = 1e9;
    double y_max =-1e9;
    double yaw_min =1e9;
    double yaw_max =-1e9;

    for (int i = 0; i< num_particles;i++)
    {

        x_min = std::min(poses.at(i).position.x,x_min);
        x_max = std::max(poses.at(i).position.x,x_max);
        y_min = std::min(poses.at(i).position.y,y_min);
        y_max = std::max(poses.at(i).position.y,y_max);

        // define the quaternion matrix
        tf::Quaternion q(
                poses.at(i).orientation.x,
                poses.at(i).orientation.y,
                poses.at(i).orientation.z,
                poses.at(i).orientation.w);
        tf::Matrix3x3 m(q);
        // convert to rpy
        m.getRPY(roll, pitch, yaw);

        yaw_min = std::min(yaw_min,yaw);
        yaw_max = std::max(yaw_max,yaw);
    }

    visualization_msgs::MarkerArray ma;
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = (x_min+x_max)/2.0;
    marker.pose.position.y = (y_min+y_max)/2.0;
    marker.pose.position.z = 0.3;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1;
    marker.scale.x = x_max-x_min;
    marker.scale.y = y_max-y_min;
    marker.scale.z = 0.05;
    marker.color.a = 1.0; 
    marker.color.r = 153.0/255.0;
    marker.color.g = 51.0/255.0;
    marker.color.b = 1.0;
    marker.lifetime =ros::Duration(0.1); 
    ma.markers.push_back(marker);
    // publish marker
    vis_pub2.publish(ma);

}

void timer_callback(const ros::TimerEvent& event)
{
  using std::cout;
  using std::endl;

  double roll, pitch, yaw;
  double x,y,vx,vy;
  
  //sim_time = 2.0;
  rect_count = 0;
  HyperRectangle hull;
  
  if(count>0)
  {

    // position and velocity
    x = msg-> pose.pose.position.x;
    y = msg-> pose.pose.position.y;
    vx = msg->twist.twist.linear.x;
    vy = msg->twist.twist.linear.y;

    // assign positions to array
    startState[0] = x;
    startState[1] = y;

    startStateU[0][0] = x-0.2;
    startStateU[0][1] = x+0.2;
    startStateU[1][0] = y-0.2;
    startStateU[1][1] = y+0.2;
    
    // compute the reachable set
    // HyperRectangle reach_hull = runReachability_obstacle_vis(startState, sim_time, walltime, 0,vx,vy,hr_list,&rect_count,max_hyper_rectangles,true);

    HyperRectangle reach_hull = runReachability_obstacle_vis_uncertain(startStateU,sim_time, walltime, 0,vx,vy,hr_list,&rect_count,max_hyper_rectangles,true);

    // define the quaternion matrix
    tf::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    // convert to rpy
    m.getRPY(roll, pitch, yaw);

    printf("num_boxes: %d, \n",rect_count);
    visualization_msgs::MarkerArray ma;
    rtreach::reach_tube reach_set;
    display_increment = rect_count  / display_max;
    display_count = std::max(1.0,nearbyint(display_increment));
    cout <<  "display_max: " << display_increment  << ", display count: " << display_count << endl;

    // publish some of the markers,
    // don't freeze gazebo
    for(int i= 0; i<std::min(max_hyper_rectangles,rect_count-1); i+=display_count)
    {
        
        hull = hr_list[i];
        if(bloat_reachset)
        {
            hull.dims[0].min = hull.dims[0].min  - 0.25;
            hull.dims[0].max = hull.dims[0].max  + 0.25;
            hull.dims[1].min = hull.dims[1].min  - 0.15;
            hull.dims[1].max = hull.dims[1].max  + 0.15;
        }
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.id = i;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = (hull.dims[0].max+hull.dims[0].min)/2.0;
        marker.pose.position.y = (hull.dims[1].max+hull.dims[1].min)/2.0;
        marker.pose.position.z = 0.2;
        marker.pose.orientation.x = 0;
        marker.pose.orientation.y = 0;
        marker.pose.orientation.z = 0;
        marker.pose.orientation.w = 0;
        marker.scale.x = (hull.dims[0].max-hull.dims[0].min);
        marker.scale.y = (hull.dims[1].max-hull.dims[1].min);
        marker.scale.z = 0.05;
        marker.color.a = 1.0; 
        marker.color.r = 0.0;
        marker.color.g = 239.0/255.0;
        marker.color.b = 1.0;
        marker.lifetime =ros::Duration(0.1); 
        ma.markers.push_back(marker);
    }

    // publish marker
    vis_pub.publish(ma);
    // publish all the rectangles we have or the max we can
    for(int i= 0; i<std::min(max_hyper_rectangles,rect_count-1); i++)
    {
       hull = hr_list[i];
       if(bloat_reachset)
       {
            hull.dims[0].min = hull.dims[0].min  - 0.25;
            hull.dims[0].max = hull.dims[0].max  + 0.25;
            hull.dims[1].min = hull.dims[1].min  - 0.15;
            hull.dims[1].max = hull.dims[1].max  + 0.15;
       }
       rtreach::interval tube;
       tube.x_min = hull.dims[0].min;
       tube.x_max = hull.dims[0].max;
       tube.y_min = hull.dims[1].min;
       tube.y_max = hull.dims[1].max;
       reach_set.obstacle_list.push_back(tube);
    }
     
    reach_set.header.stamp = ros::Time::now();
    reach_set.count = reach_set.obstacle_list.size();
    tube_pub.publish(reach_set);
  }
  
  


  else
  {
      // visualization_msgs::MarkerArray ma;
      rtreach::reach_tube reach_set;
      reach_set.header.stamp = ros::Time::now();
      reach_set.count = 0;
      // vis_pub.publish(ma);
      tube_pub.publish(reach_set);
  }
  
}

int main(int argc, char **argv)
{
    using namespace message_filters;



    if(argv[1] == NULL)
    {
        std::cout << "Please the name of the dynamic obstacle for which you would like to compute reachsets for e.g (racecar)" << std::endl;
        exit(0);
    }

    // if(argv[2] == NULL)
    // {
    //     std::cout << "Please enter the box size e.g 10" << std::endl;
    //     exit(0);
    // }

    if(argv[2] == NULL)
    {
        std::cout << "Please enter the sim_time e.g 2" << std::endl;
        exit(0);
    }

    if(argv[3] == NULL)
    {
        std::cout << "Please enter the wall time e.g 1" << std::endl;
        exit(0);
    }

    if(argv[4] == NULL)
    {
        std::cout << "Please enter the number of boxes to display e.g 100" << std::endl;
        exit(0);
    }

    std::string obs_name= argv[1];
    //box_size = atof(argv[2]) / 2.0;
    walltime = atoi(argv[2]);
    sim_time = atof(argv[3]);
    display_max = atof(argv[4]);


    // initialize the ros node
    ros::init(argc, argv, "visualize_node_obstacle");

    ros::NodeHandle n;
    
    vis_pub = n.advertise<visualization_msgs::MarkerArray>(obs_name+"/reach_hull_obs", 100 );
    vis_pub2 = n.advertise<visualization_msgs::MarkerArray>(obs_name+"/uncertainty_estimation", 100 );
    tube_pub = n.advertise<rtreach::reach_tube>(obs_name+"/reach_tube",100);
    
  
    sub = n.subscribe(obs_name+"/odom", 1000, box_pose_callback);
    sub2 = n.subscribe("/pf/viz/particles", 1000, visualize_position_uncertainty);
    ros::Timer timer = n.createTimer(ros::Duration(0.01), timer_callback);

    ros::Rate r(80);
    while(ros::ok())
    {
      r.sleep();
      ros::spinOnce();
    }
    // de-allocate obstacles
    return 0; 
}

