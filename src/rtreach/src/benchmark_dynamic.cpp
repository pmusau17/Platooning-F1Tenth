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
#include<ctime>
#include <fstream>
#include <string>

// The following node will receive messages from the LEC which will be prediction of the steering angle
// It will also receive messages from the speed node which will dictate how fast the car should travel
// Initially the assmumption will be the the car moves at constant velocity

const int max_hyper_rectangles = 2000;

extern "C"
{
    #include "bicycle_model_parametrizeable.h"
    #include "face_lift.h"
     // run reachability for a given wall timer (or iterations if negative)
    bool runReachability_bicycle_dyn(REAL* start, REAL simTime, REAL wallTimeMs, REAL startMs,REAL heading, REAL throttle,HyperRectangle VisStates[],int  *total_intermediate,int max_intermediate,bool plot);
    REAL getSimulatedSafeTime(REAL start[4],REAL heading_input, REAL throttle);
    bool check_safety(HyperRectangle* rect, REAL (*cone)[2]);
    HyperRectangle hr_list2[max_hyper_rectangles];
    void println(HyperRectangle* r);
}


int count = 0;
int rect_count = 0;
bool safe=true;
bool debug = false;
ros::Publisher res_pub;

//ros::Publisher res_pub;    // publisher for reachability results
//ros::ServiceClient client; // obstacle_service client
//rtreach::obstacle_list srv;// service call
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

clock_t start, end,total_start,total_end, reach_start, reach_end;
// variable for cumulative moving average
double iter_count = 0.0;
double avg_reach_time = 0.0; 
double avg_iterations = 0.0;
double avg_checking_time = 0.0;
double checking_time = 0.0;
double new_mean;
double differential;
double itq;
double wcet = 0.0;
double wcet_checking =0.0;




// Naive O(N^2) check
bool check_obstacle_safety(rtreach::reach_tube obs,HyperRectangle VisStates[],int rect_count)
{   
    bool safe = true;
    HyperRectangle hull;
    double cone[2][2] = {{0.0,0.0},{0.0,0.0}};
    std::cout << "obs_count: " << obs.count << ", rect_count: "<< rect_count << std::endl;
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
        for(int j = 0; j<rect_count;j++)
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




void callback(const nav_msgs::Odometry::ConstPtr& msg, const rtreach::velocity_msg::ConstPtr& velocity_msg, 
const rtreach::angle_msg::ConstPtr& angle_msg, const rtreach::stamped_ttc::ConstPtr& ttc_msg,const rtreach::reach_tube::ConstPtr& obs1, const rtreach::reach_tube::ConstPtr& obs2,const rtreach::reach_tube::ConstPtr& wall)//,const rtreach::obstacle_list::ConstPtr& obs_msg)
{
    using std::cout;
    using std::endl;

    double roll, pitch, yaw, lin_speed;
    double x,y,u,delta,qx,qy,qz,qw,uh;
    HyperRectangle hull;
    
    ttc = ttc_msg->ttc;
    
    // the lookahead time should be dictated by the lookahead time
    // since the car is moving at 1 m/s the max sim time is 1.5 seconds
    // need to look into this safety specification more earnestly
    // sim_time = fmin(1.5*ttc,sim_time);
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

    cout << "x: " << x;
    cout << " y: " << y;
    cout << " yaw: " << yaw;
    cout << " speed: " << lin_speed << endl;


    u = velocity_msg->velocity;
    delta = angle_msg->steering_angle;

    cout << "u: " << u << endl; 
    cout << "delta: " << delta << endl;

    state[0] = x;
    state[1] = y;
    state[2] = lin_speed;
    state[3] = yaw;

    // do the timing for the reachability 
    reach_start = clock();
    runReachability_bicycle_dyn(state, sim_time, wall_time, 0,delta,u,hr_list2,&rect_count,max_hyper_rectangles,true);
    reach_end = clock();
    double reach_time = double(reach_end - reach_start) / double(CLOCKS_PER_SEC);

    // do the timing for the reachability 
    reach_start = clock();
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
    reach_end = clock();
    checking_time = double(reach_end - reach_start) / double(CLOCKS_PER_SEC);
    
    if(reach_time>wcet)
    {
      wcet = reach_time;
    }
    if(checking_time>wcet_checking)
    {
      wcet_checking = checking_time;
    }
    // calculate exponential moving average
    iter_count++;
    differential = (reach_time - avg_reach_time) / iter_count;
    new_mean = avg_reach_time + differential;
    avg_reach_time = new_mean;

    // calculate exponential moving average of iterations

    itq = (double)iterations_at_quit;
    differential = (iterations_at_quit-avg_iterations) / iter_count;
    new_mean = avg_iterations+differential;
    avg_iterations = new_mean;


    differential = (checking_time - avg_checking_time) / iter_count;
    new_mean = avg_checking_time + differential;
    avg_checking_time = new_mean;

    std_msgs::Float32 res_msg;
    res_msg.data = (double)safe;
    res_pub.publish(res_msg);
    printf("safe: %d\n",safe);
}





int main(int argc, char **argv)
{

    using namespace message_filters;
    int num_dynamic_obstacles;
    std::string controller_topic;
    std::string result_topic = "reachability_result";
    std::string save_path; 
    save_path = ros::package::getPath("rtreach") +"/benchmarking/"+"dynamic_experiments.csv";
    // initialize the ros node
    ros::init(argc, argv, "reach",ros::init_options::AnonymousName);
    
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

    if(argc <4)
    {
        debug = false;
    }
    else
        debug = (bool)atoi(argv[4]);


    if(argc<5)
    {
        controller_topic = "racecar2/angle_msg";
    }
    else
    {
        controller_topic = argv[5];
        result_topic = controller_topic+"/reachability_result";
        if(controller_topic=="racecar2/angle_msg")
            result_topic ="reachability_result";
    }
    std::cout << controller_topic << result_topic << std::endl;


    // wall-time is how long we want the reachability algorithm to run
    wall_time = atoi(argv[1]);
    // sim-time is how far in the future we want the reachability algorithm to look into the future 
    sim_time = atof(argv[2]);
    // as there are numerous boxes computed within the reachability computation, we must limit the number
    // we send to rviz
    display_max = atof(argv[3]);
 
    // Initialize the list of subscribers 
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub(n, "racecar2/odom", 10);
    message_filters::Subscriber<rtreach::velocity_msg> vel_sub(n, "racecar2/velocity_msg", 10);
    message_filters::Subscriber<rtreach::angle_msg> angle_sub(n, controller_topic, 10);
    message_filters::Subscriber<rtreach::stamped_ttc> ttc_sub(n, "racecar2/ttc", 10);
    message_filters::Subscriber<rtreach::reach_tube> obs1(n,"racecar/reach_tube",10);
    message_filters::Subscriber<rtreach::reach_tube> obs2(n,"racecar3/reach_tube",10);
    message_filters::Subscriber<rtreach::reach_tube> wall(n,"wallpoints",10);
    res_pub = n.advertise<std_msgs::Float32>(result_topic, 10);


    // message synchronizer 
    typedef sync_policies::ApproximateTime<nav_msgs::Odometry, rtreach::velocity_msg, rtreach::angle_msg,rtreach::stamped_ttc,rtreach::reach_tube,rtreach::reach_tube,rtreach::reach_tube> MySyncPolicy; 

    // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(100), odom_sub, vel_sub,angle_sub,ttc_sub,obs1,obs2,wall);//,interval_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2,_3,_4,_5,_6,_7));

    while(ros::ok())
    {
      // call service periodically 
      ros::spinOnce();
    }

    std::ofstream outfile(save_path.c_str() , std::ios::app);
    outfile <<  wcet << "," << avg_reach_time << "," << avg_iterations << "," << avg_checking_time << "," << wcet_checking << "\n";
    outfile.close();
    return 0; 
}