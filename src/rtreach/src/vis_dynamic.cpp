#include "ros/ros.h"
#include <ros/package.h>
#include <ros/console.h>

#include <rtreach/angle_msg.h>
#include <rtreach/velocity_msg.h>
#include <rtreach/stamped_ttc.h>
#include <rtreach/obstacle_list.h>

#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <nav_msgs/Odometry.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <message_filters/subscriber.h>
#include <tf/tf.h>


#include <math.h>
#include <iostream>

// The following node will receive messages from the LEC which will be prediction of the steering angle
// It will also receive messages from the speed node which will dictate how fast the car should travel
// Initially the assmumption will be the the car moves at constant velocity

extern "C"
{ 
     #include "bicycle_dynamic_safety.h"
     #include "simulate_bicycle_plots.h"
     HyperRectangle runReachability_bicycle_vis(REAL* start, REAL simTime, REAL wallTimeMs, REAL startMs,REAL heading_input, REAL throttle);
     HyperRectangle hull;
     void println(HyperRectangle * r);
     void allocate_obstacles(int num_obstacles);
     void append_obstacle(int index,double (*box)[2]);
     void print_obstacles();
     void deallocate_obstacles();
}
 
ros::Publisher vis_pub; 

// reachability parameters
double sim_time = 1.0;
double walltime = 25; // 25 ms corresponds to 40 hz 
int markers_allocated = 1;
bool bloat_reachset = true;
double ttc = 0.0;
int num_obstacles = 0;






void callback(const nav_msgs::Odometry::ConstPtr& msg, const rtreach::velocity_msg::ConstPtr& velocity_msg, 
const rtreach::angle_msg::ConstPtr& angle_msg, const rtreach::stamped_ttc::ConstPtr& ttc_msg)//,const rtreach::obstacle_list::ConstPtr& obs_msg)
{
  using std::cout;
  using std::endl;

  double roll, pitch, yaw, lin_speed;
  double x,y,u,delta;
  
  ttc = ttc_msg->ttc;
  
  // the lookahead time should be dictated by the lookahead time
  // since the car is moving at 1 m/s the max sim time is 1.5 seconds
  // sim_time = fmin(1.5*ttc,sim_time);
  sim_time = 1.0;
  std::cout << "sim_time: " << sim_time << endl;

  x = msg-> pose.pose.position.x;
  y = msg-> pose.pose.position.y;

  // define the quaternion matrix
  tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);

  // convert to rpy
  m.getRPY(roll, pitch, yaw);

  // normalize the speed 
  tf::Vector3 speed = tf::Vector3(msg->twist.twist.linear.x, msg->twist.twist.linear.x, 0.0);
  lin_speed = speed.length();

  cout << "x: " << x;
  cout << " y: " << y;
  cout << " yaw: " << yaw;
  cout << " speed: " << lin_speed << endl;


  u = velocity_msg->velocity;
  delta = angle_msg->steering_angle;

  cout << "u: " << u << endl; 
  cout << "delta: " << delta << endl;

  double state[4] = {x,y,lin_speed,yaw};
  double control_input[2] = {u,delta};


  // create the ros message that will be sent to the VESC

  if(markers_allocated>0)
  {
    hull = runReachability_bicycle_vis(state, sim_time, walltime, 0, delta, u);
    printf("num_boxes: %d, ",num_intermediate);
    visualization_msgs::MarkerArray ma;
    for(int i = 0; i<num_intermediate;i++)
    {
      hull = VisStates[i];

      // if we want to bloat the hyper-rectangles for the width of the car
      if(bloat_reachset)
      {
        hull.dims[0].min = hull.dims[0].min  - 0.25;
        hull.dims[0].max = hull.dims[0].max  + 0.25;
        hull.dims[1].min = hull.dims[1].min  - 0.15;
        hull.dims[1].max = hull.dims[1].max  + 0.15;
      }
      
      visualization_msgs::Marker marker;
      marker.header.frame_id = "/map";
      marker.header.stamp = ros::Time::now();
      marker.id = i;
      marker.type = visualization_msgs::Marker::CUBE;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.x = (hull.dims[0].max+hull.dims[0].min)/2.0;
      marker.pose.position.y = (hull.dims[1].max+hull.dims[1].min)/2.0;
      
      marker.pose.position.z = 0.0;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = (hull.dims[0].max-hull.dims[0].min);
      marker.scale.y = (hull.dims[1].max-hull.dims[1].min);
      marker.scale.z = 0.05;
      marker.color.a = 1.0; 
      marker.color.r = (double) rand() / (RAND_MAX);
      marker.color.g = (double) rand() / (RAND_MAX);
      marker.color.b = (double) rand() / (RAND_MAX);
      marker.lifetime =ros::Duration(0.1); 
      ma.markers.push_back(marker);
    }

    // publish marker
    vis_pub.publish( ma );
  }
  
}

int main(int argc, char **argv)
{

    allocate_obstacles(10);
    // get the path to the file containing the wall points 
    // std::string path = ros::package::getPath("rtreach");
    
    // // file describing walls of simulation environment
    // if(argv[1] == NULL)
    // {
    //     std::cout << "Please provide the file containing the obstacle locations (i.e porto_obstacles.txt)" << std::endl;
    //     exit(0);
    // }

    // // whether or not to bloat the reachset for the width of the car
    // if(argv[2] != NULL)
    // {
    //     bloat_reachset = (bool) atoi(argv[2]);
    // }

    // // simulation time argument
    // if(argv[3]!=NULL)
    // {
    //   sim_time= atof(argv[3]);
    // }

    //  // wall time for reachability algorithm
    // if(argv[4]!=NULL)
    // {
    //   walltime= atof(argv[4]);
    // }

    using namespace message_filters;

    // initialize the ros node
    ros::init(argc, argv, "visualize_node_dynamic");

    ros::NodeHandle n;
    // ackermann publisher 
    // a description of how the synchronization works can be found here: 
    // http://wiki.ros.org/message_filters/ApproximateTime

    
    
    // define the subscribers you want 
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub(n, "racecar2/odom", 10);
    message_filters::Subscriber<rtreach::velocity_msg> vel_sub(n, "racecar2/velocity_msg", 10);
    message_filters::Subscriber<rtreach::angle_msg> angle_sub(n, "racecar2/angle_msg", 10);
    message_filters::Subscriber<rtreach::stamped_ttc> ttc_sub(n, "racecar2/ttc", 10);
    // message_filters::Subscriber<rtreach::obstacle_list> interval_sub(n, "racecar2/obstacles", 10);
    

    vis_pub = n.advertise<visualization_msgs::MarkerArray>( "racecar2/reach_hull", 100 );


    typedef sync_policies::ApproximateTime<nav_msgs::Odometry, rtreach::velocity_msg, rtreach::angle_msg,rtreach::stamped_ttc> MySyncPolicy; //, rtreach::obstacle_list> MySyncPolicy;

    // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(100), odom_sub, vel_sub,angle_sub,ttc_sub);//,interval_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2,_3,_4));//,_5));

    ros::Rate r(80);
    while(ros::ok())
    {
      r.sleep();
      ros::spinOnce();
    }

    deallocate_obstacles();


    return 0; 
}