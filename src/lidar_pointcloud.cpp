#include <ros/ros.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
pcl::PointCloud<pcl::PointXYZ> cloud;

pcl::PointCloud<pcl::PointXYZ> cloudmsg2cloud(sensor_msgs::PointCloud2 cloudmsg)
{
	pcl::PointCloud<pcl::PointXYZ> cloud_dst;
	pcl::fromROSMsg(cloudmsg, cloud_dst);
	return cloud_dst;
}

sensor_msgs::PointCloud2 cloud2cloudmsg(pcl::PointCloud<pcl::PointXYZ> cloud_src)
{
	sensor_msgs::PointCloud2 cloudmsg;
	pcl::toROSMsg(cloud_src, cloudmsg);
	cloudmsg.header.frame_id = "map";
	return cloudmsg;
}

void PointCloudCallBack(const sensor_msgs::PointCloud2ConstPtr& ptrCloudmsg){
    pcl::PointCloud<pcl::PointXYZ> cloud_dst = cloudmsg2cloud(*ptrCloudmsg);
    //1. Pointcloud & Image 
    //2. Message filter -> Approximate Time policy
}

int main(int argc, char** argv)
{
    // initializing ROS node
    ros::init(argc, argv, "PointCloud_Managing_Node");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("points", 1000, PointCloudCallBack);
    //                    nh.subscribe("TOPIC", queue_size, CallBackFunction);

    // Progress CallBack untill node is shutdowned
    ros::spin(); 
}