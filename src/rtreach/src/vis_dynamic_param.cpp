#include "ros/ros.h"
#include <iostream>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>

// message files defined within this package
#include <rtreach/reach_tube.h>
#include <rtreach/angle_msg.h>
#include <rtreach/velocity_msg.h>
#include <rtreach/stamped_ttc.h>
#include <rtreach/obstacle_list.h>

#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <ros/package.h>
#include <ros/console.h>
#include "std_msgs/Float32.h"
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <math.h>
#include <cstdlib>
#include <memory>

// The following node will receive messages from the LEC which will be prediction of the steering angle
// It will also receive messages from the speed node which will dictate how fast the car should travel
// Initially the assmumption will be the the car moves at constant velocity

const int max_hyper_rectangles = 2000;
bool debug = false;

extern "C"
{
     #include "bicycle_model_parametrizeable.h"
     // run reachability for a given wall timer (or iterations if negative)
    bool runReachability_bicycle_dyn(REAL* start, REAL simTime, REAL wallTimeMs, REAL startMs,REAL heading, REAL throttle,HyperRectangle VisStates[],int  *total_intermediate,int max_intermediate,bool plot);
    REAL getSimulatedSafeTime(REAL start[4],REAL heading_input, REAL throttle);
    bool check_safety(HyperRectangle* rect, REAL (*cone)[2]);
    HyperRectangle hr_list2[max_hyper_rectangles];
}


int count = 0;
int rect_count = 0;
bool safe=true;

ros::Publisher vis_pub;
rtreach::reach_tube static_obstacles;
double sim_time;
double state[4] = {0.0,0.0,0.0,0.0};
double control_input[2] = {0.0,0.0};
int wall_time;


int markers_allocated = 1;
bool bloat_reachset = true;
double ttc = 0.0;
int num_obstacles = 0;
double display_max;
int display_count = 1;
double display_increment = 1.0;
int color_topic = 0;


void callback(const nav_msgs::Odometry::ConstPtr& msg, const rtreach::velocity_msg::ConstPtr& velocity_msg, 
const rtreach::angle_msg::ConstPtr& angle_msg, const rtreach::stamped_ttc::ConstPtr& ttc_msg)
{
    using std::cout;
    using std::endl;

    double roll, pitch, yaw, lin_speed;
    double x,y,u,delta,qx,qy,qz,qw,uh;
    HyperRectangle hull;
    
    ttc = ttc_msg->ttc;
    
    // the lookahead time should be dictated by the lookahead time
    // since the car is moving at 1 m/s the max sim time is 1.5 seconds
    // sim_time = fmin(1.5*ttc,sim_time);
    
    if(debug)
        std::cout << "sim_time: " << sim_time << endl;

    x = msg-> pose.pose.position.x;
    y = msg-> pose.pose.position.y;

    qx = msg->pose.pose.orientation.x;
    qy = msg->pose.pose.orientation.y;
    qz = msg->pose.pose.orientation.z;
    qw = msg->pose.pose.orientation.w;

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

    if(debug)
    {
        cout << "x: " << x;
        cout << " y: " << y;
        cout << " yaw: " << yaw;
        cout << " speed: " << lin_speed << endl;
    }
  


    u = velocity_msg->velocity;
    delta = angle_msg->steering_angle;

    if(debug)
    {
        cout << "u: " << u << endl; 
        cout << "delta: " << delta << endl;
    }
    

    state[0] = x;
    state[1] = y;
    state[2] = lin_speed;
    state[3] = yaw;

    runReachability_bicycle_dyn(state, sim_time, wall_time, 0,delta,u,hr_list2,&rect_count,max_hyper_rectangles,true);

    if(debug)
        printf("num_boxes: %d, \n",rect_count);
    visualization_msgs::MarkerArray ma;
    display_increment = rect_count  / display_max;
    display_count = std::max(1.0,nearbyint(display_increment));
    
    if(debug)
        cout <<  "display_max: " << display_increment  << ", display count: " << display_count << endl;

    // allocate markers
    for(int i= 0; i<std::min(max_hyper_rectangles-1,rect_count-1); i+=display_increment)
    {
        
        hull = hr_list2[i];
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
        marker.pose.position.z = 0.5; 
        marker.pose.orientation.x = qx;
        marker.pose.orientation.y = qy;
        marker.pose.orientation.z = qz;
        marker.pose.orientation.w = qw;
        marker.scale.x = (hull.dims[0].max-hull.dims[0].min);
        marker.scale.y = (hull.dims[1].max-hull.dims[1].min);
        marker.scale.z = 0.05;
        marker.color.a = 1.0; 

        if(color_topic==0)
        {
            marker.color.r = 0.0; //(double) rand() / (RAND_MAX);
            marker.color.g = 0.0; //(double) rand() / (RAND_MAX);
            marker.color.b = 0.8; //(double) rand() / (RAND_MAX);
        }
        else if(color_topic==1)
        {
            marker.color.r = 0.0; //(double) rand() / (RAND_MAX);
            marker.color.g = 0.8; //(double) rand() / (RAND_MAX);
            marker.color.b = 0.0; //(double) rand() / (RAND_MAX);
        }
        else
        {
            marker.color.r = 0.8; //(double) rand() / (RAND_MAX);
            marker.color.g = 0.0; //(double) rand() / (RAND_MAX);
            marker.color.b = 0.0; //(double) rand() / (RAND_MAX);
        }
        

        
        marker.lifetime =ros::Duration(0.1); 
        ma.markers.push_back(marker);
    }

    vis_pub.publish(ma);




}





int main(int argc, char **argv)
{

    using namespace message_filters;
    int num_dynamic_obstacles;
    // initialize the ros node
    ros::init(argc, argv, "reach_node_param",ros::init_options::AnonymousName);

    // controller topics
    std::string controller_topic;
    std::string result_topic = "racecar2/reach_hull_param";
    
    ros::NodeHandle n;
    

    if(argv[1] == NULL)
    {
        std::cout << "Please give the walltime 10" << std::endl;
        exit(0);
    }

    if(argv[2] == NULL)
    {
        std::cout << "Please give the sim time (e.g) 2" << std::endl;
        exit(0);
    }

    if(argv[3] == NULL)
    {
        std::cout << "Please give the display max(e.g) 100" << std::endl;
        exit(0);
    }

    if(argc >4)
    {
        debug = (bool)atoi(argv[4]);
    }
    else
        debug = false;
        

    if(argc >5)
    {
        controller_topic = argv[5];
        result_topic = controller_topic+"/reachability_hull";
    }
    else
    {
        controller_topic="racecar2/angle_msg";
    }


    if(argc >6)
    {
        color_topic = atoi(argv[6]);
    }


    // wall-time is how long we want the reachability algorithm to run
    wall_time = atoi(argv[1]);
    // sim-time is how far in the future we want the reachability algorithm to look into the future 
    sim_time = atof(argv[2]);
    // as there are numerous boxes computed within the reachability computation, we must limit the number
    // we send to rviz
    display_max = atof(argv[3]);

    // visualization publisher
    vis_pub = n.advertise<visualization_msgs::MarkerArray>( result_topic, 100 );


    
    // Initialize the list of subscribers 
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub(n, "racecar2/odom", 10);
    message_filters::Subscriber<rtreach::velocity_msg> vel_sub(n, "racecar2/velocity_msg", 10);
    message_filters::Subscriber<rtreach::angle_msg> angle_sub(n, controller_topic, 10);
    message_filters::Subscriber<rtreach::stamped_ttc> ttc_sub(n, "racecar2/ttc", 10);


    // message synchronizer 
    typedef sync_policies::ApproximateTime<nav_msgs::Odometry, rtreach::velocity_msg, rtreach::angle_msg,rtreach::stamped_ttc> MySyncPolicy; 

    // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(100), odom_sub, vel_sub,angle_sub,ttc_sub);//,interval_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2,_3,_4));


    while(ros::ok())
    {
      ros::spinOnce();
    }
    return 0; 
}