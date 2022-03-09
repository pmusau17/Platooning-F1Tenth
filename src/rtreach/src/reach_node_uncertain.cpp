#include "ros/ros.h"
#include <iostream>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>

// message files defined within this package
#include <rtreach/reach_tube.h>
#include <rtreach/angle_msg.h>
#include <rtreach/velocity_msg.h>
#include <rtreach/obstacle_list.h>

#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <ros/package.h>
#include <ros/console.h>
#include "std_msgs/Float32.h"
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <math.h>
#include <cstdlib>
#include <memory>

// The following node will receive messages from the LEC which will be prediction of the steering angle
// It will also receive messages from the speed node which will dictate how fast the car should travel
// Initially the assmumption will be the the car moves at constant velocity

const int max_hyper_rectangles = 2000;

extern "C"
{
     #include "bicycle_model_uncertainty.h"
     // run reachability for a given wall timer (or iterations if negative)
    bool runReachability_bicycle_uncertain(REAL start[][2], REAL simTime, REAL wallTimeMs, REAL startMs,REAL heading, REAL throttle, REAL parameter_uncertainty, REAL disturbance[][2], HyperRectangle VisStates[],int  *total_intermediate,int max_intermediate,bool plot);
    bool check_safety(HyperRectangle* rect, REAL (*cone)[2]);
    HyperRectangle hr_list2[max_hyper_rectangles];
    void println(HyperRectangle* r);
}


int count = 0;
int rect_count = 0;
bool safe=true;
bool debug = true;
bool log_console_output = false;
bool use_particles = false;
ros::Publisher res_pub;
ros::Publisher vis_pub;

rtreach::reach_tube static_obstacles;
double sim_time;
double state[4][2] = {{0.0,0.0},{0.0,0.0},{0.0,0.0},{0.0,0.0}};

// disturbances 
double disturbances[4][2] = {{0.0,0.0},{0.0,0.0},{0.0,0.0},{0.0,0.0}}; 


double control_input[2] = {0.0,0.0};
int wall_time;


int markers_allocated = 1;
bool bloat_reachset = true;
double parameter_uncertainty = 0.0;
int num_obstacles = 0;
double display_max;
int display_count = 1;
double display_increment = 1.0;



// Naive O(N^2) check
bool check_obstacle_safety(rtreach::reach_tube obs,HyperRectangle VisStates[],int rect_count)
{   
    bool safe = true;
    HyperRectangle hull;
    double cone[2][2] = {{0.0,0.0},{0.0,0.0}};

    if(log_console_output)
    {
        std::cout << "obs_count: " << obs.count << ", rect_count: "<< rect_count << std::endl;
    }
    
    for (int i=0;i<obs.count;i++)
    {
        if(!safe)
        {
            break;
        }
        cone[0][0] = obs.obstacle_list[i].x_min;
        cone[0][1] = obs.obstacle_list[i].x_max;
        cone[1][0] = obs.obstacle_list[i].y_min;
        cone[1][1] = obs.obstacle_list[i].y_max;
        for(int j = 0; j<rect_count-2;j++)
        {

            hull = VisStates[j];
            hull.dims[0].min = hull.dims[0].min  - 0.25;
            hull.dims[0].max = hull.dims[0].max  + 0.25;
            hull.dims[1].min = hull.dims[1].min  - 0.15;
            hull.dims[1].max = hull.dims[1].max  + 0.15;

            safe = check_safety(&hull,cone);
            if(!safe)
            {   
                break;
            }
        }
    }
    return safe;
}


std::vector<std::vector<double>> get_particle_position(const geometry_msgs::PoseArray::ConstPtr& pose_msg)
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

    std::vector<std::vector<double>> vect
    {
        // x uncertainty
        {x_min, x_max},
       
        // y uncertainty
        {y_min, y_max},
       
        // yaw uncertainty
        {yaw_min,yaw_max}
    };

    return vect;
}


void callback(const nav_msgs::Odometry::ConstPtr& msg, const rtreach::velocity_msg::ConstPtr& velocity_msg, 
              const rtreach::angle_msg::ConstPtr& angle_msg, const rtreach::reach_tube::ConstPtr& obs1, const rtreach::reach_tube::ConstPtr& obs2,
              const rtreach::reach_tube::ConstPtr& wall, const geometry_msgs::PoseArray::ConstPtr& pose_msg)
{
    using std::cout;
    using std::endl;

    double roll, pitch, yaw, lin_speed;
    double x,y,u,delta,qx,qy,qz,qw,uh;
    std::vector<std::vector<double>> position_uncertainty;
    HyperRectangle hull;
    
    if(log_console_output)
    {
        // still need to figure out how to select the sim_time
        std::cout << "sim_time: " << sim_time << endl;
    }

    
    position_uncertainty = get_particle_position(pose_msg);
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

    if(log_console_output)
    {
        cout << "x: " << x;
        cout << " y: " << y;
        cout << " yaw: " << yaw;
        cout << " speed: " << lin_speed << endl;
    }


    u = velocity_msg->velocity;
    delta = angle_msg->steering_angle;

    if(log_console_output)
    {
        cout << "u: " << u << endl; 
        cout << "delta: " << delta << endl;
    }



    // if the positions are determined by uncertainty we can parameterize that 
    // as well, the position is defined by uncertainty and any disturbances 
    // that need to be incorporated as well. Need to thin about how though

    if(use_particles)
    {
        state[0][0] = position_uncertainty[0][0];
        state[0][1] = position_uncertainty[0][1];
        state[1][0] = position_uncertainty[1][0];
        state[1][1] = position_uncertainty[1][1];
        state[2][0] = lin_speed;
        state[2][1] = lin_speed;
        state[3][0] = position_uncertainty[2][0];
        state[3][1] = position_uncertainty[2][1];
    }
    else
    {
        state[0][0] = x;
        state[0][1] = x;
        state[1][0] = y;
        state[1][1] = y;
        state[2][0] = lin_speed;
        state[2][1] = lin_speed;
        state[3][0] = yaw;
        state[3][1] = yaw;
    }
    

    runReachability_bicycle_uncertain(state, sim_time, wall_time, 0, delta, u, parameter_uncertainty, disturbances, hr_list2,&rect_count,max_hyper_rectangles,true);
    if(log_console_output)
    {
        printf("num_boxes: %d, obs1 count: %d, obs2 count: %d, \n",rect_count,obs1->count,obs2->count);
    }
    

    // do the safety checking between the dynamic obstacles here
    if(obs1->count>0)
    {
        safe  = check_obstacle_safety(*obs1,hr_list2,std::min(max_hyper_rectangles,rect_count));
         
    }
    if(obs2->count>0 && safe)
    {
        safe  = check_obstacle_safety(*obs2,hr_list2,std::min(max_hyper_rectangles,rect_count));
    }
    if(wall->count>0 && safe)
    {
        safe  = check_obstacle_safety(*wall,hr_list2,std::min(max_hyper_rectangles,rect_count));
    }
    std_msgs::Float32 res_msg;
    res_msg.data = (double)safe;
    res_pub.publish(res_msg);
    if(log_console_output)
        printf("safe: %d\n",safe);

    // visualization_debugging
    if(debug){
        visualization_msgs::MarkerArray ma;
        // std::min(rect_count-1,max_hyper_rectangles-1)
        for(int i = 0; i<std::min(rect_count,max_hyper_rectangles)-2;i+=10)
        {
          HyperRectangle hull = hr_list2[i];
          //println(&hull);
          hull.dims[0].min = hull.dims[0].min  - 0.15;
          hull.dims[0].max = hull.dims[0].max  + 0.15;
          hull.dims[1].min = hull.dims[1].min  - 0.15;
          hull.dims[1].max = hull.dims[1].max  + 0.15;

          
          visualization_msgs::Marker marker;
          marker.header.frame_id = "map";
          marker.header.stamp = ros::Time::now();
          marker.id = i;
          marker.type = visualization_msgs::Marker::CUBE;
          marker.action = visualization_msgs::Marker::ADD;
          marker.pose.position.x = (hull.dims[0].max+hull.dims[0].min)/2.0;
          marker.pose.position.y = (hull.dims[1].max+hull.dims[1].min)/2.0;

          
          marker.pose.position.z = 0.2;

    
          marker.pose.orientation.x =  msg->pose.pose.orientation.x;
          marker.pose.orientation.y = msg->pose.pose.orientation.y;
          marker.pose.orientation.z = msg->pose.pose.orientation.z;
          marker.pose.orientation.w = msg->pose.pose.orientation.w;
          marker.scale.x = (hull.dims[0].max-hull.dims[0].min);
          marker.scale.y = (hull.dims[1].max-hull.dims[1].min);
          marker.scale.z = 0.1;
          marker.color.a = 1.0; 
          if(safe)
          {
            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 1.0;
          }
          else
          {
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0; 
          }
          marker.lifetime =ros::Duration(0.1); 
          ma.markers.push_back(marker);
        }

        // publish marker
        vis_pub.publish( ma );
    }

}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "reach_uncertainty",ros::init_options::AnonymousName);
    using namespace message_filters;
    int num_dynamic_obstacles;
    std::string controller_topic;
    std::string result_topic = "reachability_result";
    
    
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

    if(argv[4] == NULL)
    {
        std::cout << "Please give the parameter uncertainty" << std::endl;
        exit(0);
    }

    if(argv[5] == NULL)
    {
        std::cout << "Please specify if you want to use ground truth or the particle filter." << std::endl;
        exit(0);
    }

    
    // wall-time is how long we want the reachability algorithm to run
    wall_time = atoi(argv[1]);
    // sim-time is how far in the future we want the reachability algorithm to look into the future 
    sim_time = atof(argv[2]);
    // as there are numerous boxes computed within the reachability computation, we must limit the number
    // we send to rviz
    display_max = atof(argv[3]);

    // what level of parameter to consider in the experiments
    parameter_uncertainty = atof(argv[4]);

    use_particles = (bool)atoi(argv[5]);


 
    // Initialize the list of subscribers 
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub(n, "racecar/odom", 5);
    message_filters::Subscriber<rtreach::velocity_msg> vel_sub(n, "racecar/velocity_msg", 5);
    message_filters::Subscriber<rtreach::angle_msg> angle_sub(n, "racecar/angle_msg", 5);
    message_filters::Subscriber<rtreach::reach_tube> obs1(n,"racecar2/reach_tube",5);
    message_filters::Subscriber<rtreach::reach_tube> obs2(n,"racecar3/reach_tube",5);
    message_filters::Subscriber<rtreach::reach_tube> wall(n,"wallpoints",5);

    // subscriber for GPU based particles
    message_filters::Subscriber<geometry_msgs::PoseArray> particles(n,"/pf/viz/particles",5);

    res_pub = n.advertise<std_msgs::Float32>(result_topic, 1);
    if(debug)
    {
      vis_pub = n.advertise<visualization_msgs::MarkerArray>( "reach_verify", 100 );
    }



    // message synchronizer 
    typedef sync_policies::ApproximateTime<nav_msgs::Odometry, rtreach::velocity_msg, rtreach::angle_msg,rtreach::reach_tube,rtreach::reach_tube,rtreach::reach_tube,geometry_msgs::PoseArray> MySyncPolicy; 

    // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(1), odom_sub, vel_sub,angle_sub,obs1,obs2,wall,particles);//,interval_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2,_3,_4,_5,_6,_7));

    while(ros::ok())
    {
      // call service periodically 
      ros::spinOnce();
    }
    return 0; 
}