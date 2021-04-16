
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_broadcaster.h>
#include "laser_geometry/laser_geometry.h"
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_listener.h>
#include "pcl_ros/transforms.h"




std::string racecar_name = "racecar";


class LaserToPointCloud
{

    private: 
        ros::Publisher pcl_from_scan;
        laser_geometry::LaserProjection projector_;
        tf::TransformListener listener_;
        ros::NodeHandle nh_;
        ros::Subscriber hokuyo_sub;
    public: 
        LaserToPointCloud(ros::NodeHandle* nh);
        ~LaserToPointCloud(){}
        void laser_callback(const sensor_msgs::LaserScan::ConstPtr& scan_in);


};


LaserToPointCloud::LaserToPointCloud(ros::NodeHandle* nh):nh_(*nh)
{
    this->pcl_from_scan = nh_.advertise<sensor_msgs::PointCloud2>(racecar_name+"/filtered_cloud", 1);
    this->hokuyo_sub = nh_.subscribe<sensor_msgs::LaserScan>(racecar_name+"/scan", 10, &LaserToPointCloud::laser_callback,this);
}




void LaserToPointCloud::laser_callback(const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
    if(!this->listener_.waitForTransform(
        scan_in->header.frame_id,
        racecar_name+"/odom",
        scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment),
        ros::Duration(1.0))){
     return;
    }

    sensor_msgs::PointCloud2 cloud;
    //projector.projectLaser(*scan_in, cloud);
    // Publish the new point cloud.
    // cloud.header.frame_id = "ra/laser";
    this->projector_.transformLaserScanToPointCloud(racecar_name+"/odom",*scan_in,
          cloud,listener_);
    // cloud.header.stamp = scan_in->header.stamp;
    this->pcl_from_scan.publish(cloud);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "laserScan_to_pointcloud");
    ros::NodeHandle nh;
    
    if(argv[1] == NULL)
        racecar_name = "racecar";
    else 
        racecar_name = (std::string)argv[1];

    LaserToPointCloud lnode(&nh);
    while (ros::ok())
    {
        ros::spin();
    }

    nh.shutdown();          
    return 0;
}