


















#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>

#include<Eigen/Dense>

using namespace sensor_msgs;
using namespace message_filters;

Eigen::Matrix3d R;
Eigen::Vector3d t;
Eigen::Matrix3d K;

pcl::PointCloud<pcl::PointXYZ> cloud;
   

struct OusterInput {
    PCL_ADD_POINT4D;
    float intensity;
    uint32_t t;
    uint16_t reflectivity;
    uint16_t ring;
    uint16_t ambient;
    uint32_t range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(OusterInput,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (uint32_t, t, t) (uint16_t, reflectivity, reflectivity)
    (uint16_t, ring, ring) (uint16_t, ambient, ambient) (uint32_t, range, range)
)

struct OusterOutput {
    PCL_ADD_POINT4D;
    float intensity;
    uint32_t t;
    uint16_t reflectivity;
    uint16_t ring;
    uint16_t ambient;
    uint32_t range;
    uint32_t rgba;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(OusterOutput,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (uint32_t, t, t) (uint16_t, reflectivity, reflectivity)
    (uint16_t, ring, ring) (uint16_t, ambient, ambient) (uint32_t, range, range) (uint32_t, rgba, rgba)
)

// for publishing pointCloud with rgb
ros::Publisher pub;
 pcl::PointCloud<OusterInput>::Ptr tmpOusterCloudIn;

void cloudmsg2cloud(const sensor_msgs::PointCloud2ConstPtr& cloudmsg)
{
    sensor_msgs::PointCloud2 currentCloudMsg = std::move(*cloudmsg);
    for(const auto& elem : currentCloudMsg.fields){
        std::cout<<elem<<std::endl;
    }

    
    pcl::moveFromROSMsg(currentCloudMsg, *tmpOusterCloudIn);
}

sensor_msgs::PointCloud2 cloud2cloudmsg(pcl::PointCloud<pcl::PointXYZ> cloud_src)
{
	sensor_msgs::PointCloud2 cloudmsg;
	pcl::toROSMsg(cloud_src, cloudmsg);
	cloudmsg.header.frame_id = "map";
	return cloudmsg;
}
cv::Point2f projectPoint(const OusterInput& point){
    Eigen::Vector3d pointLidar(point.x, point.y, point.z);
    Eigen::Vector3d pointCam = R * pointLidar + t;
    if(pointCam(2)>0){
        double u = (K(0, 0) * pointCam(0) / pointCam(2)) + K(0, 2);
        double v = (K(1, 1) * pointCam(1) / pointCam(2)) + K(1, 2);

        return cv::Point2f(u, v);
    }
    
    return cv::Point2f(-1000, -1000);
}

void callback(const sensor_msgs::ImageConstPtr& image1, const sensor_msgs::PointCloud2ConstPtr& lidar1)
{
    // image processing
    std::cout<<"CALLBACK"<<std::endl;
    cv::Mat image;
    try
    {
        // Convert the ROS image mesage to a cv::Mat using cv_bridge
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image1, sensor_msgs::image_encodings::BGR8);

        // Get the OpenCV Mat object
        image = cv_ptr->image;

        //Display the image using OpenCV
        cv::imshow("Camera Image", image);
        cv::waitKey(1); // Update the image window
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }

    // PointCLoud processing
    //pcl::PointCloud<OusterPointXYZIRT> cloud_dst = cloudmsg2cloud(*lidar1);
    cloudmsg2cloud(lidar1);
    pcl::PointCloud<OusterInput> cloud_dst = *tmpOusterCloudIn;
    // PointCloud with RGB value
    pcl::PointCloud<OusterOutput> cloud_rgb;
    for(int i=0; i<cloud_dst.size(); i++){
        // Get x(u),y(v) of image from XYZ Lidar point
        cv::Point2f uv = projectPoint(cloud_dst[i]);


        // make XYZRGB point
        OusterOutput point_rgb; 
        point_rgb.x = cloud_dst[i].x;
        point_rgb.y = cloud_dst[i].y;
        point_rgb.z = cloud_dst[i].z;
        point_rgb.intensity = cloud_dst[i].intensity;
        point_rgb.t=cloud_dst[i].t;
        point_rgb.reflectivity=cloud_dst[i].reflectivity;
        point_rgb.ring=cloud_dst[i].ring;
        point_rgb.ambient=cloud_dst[i].ambient;



        // chech whether u and v is in range of image (width, height)
        cv::Size imageSize = image.size();
        uint8_t r, g, b, a;
        if(uv.x < 0 || imageSize.width < uv.x || uv.y < 0 || imageSize.height < uv.y)
        {
            r = 0;
            g = 0;
            b = 0;
            a = 0;
            point_rgb.intensity = 0;
            point_rgb.rgba = (a << 24) + (r << 16) + (g <<8) + b;            // not drawing
            // point_rgb.r = 0;
            // point_rgb.g = 0;
            // point_rgb.b = 0;
            // point_rgb.a = 0;
            
        }else{
            cv::Vec3b pixel_rgb = image.at<cv::Vec3b>(uv.y,uv.x);
            r = pixel_rgb(2);
            g = pixel_rgb(1);
            b = pixel_rgb(0);
            a = 255;
            point_rgb.rgba = (a << 24) + (r << 16) + (g <<8) + b;            // not drawing

        }
        
        // include made point in new pointCloud
        cloud_rgb.push_back(point_rgb);
    }
    // check whether the cloud_rgb is well-made
    std::cout<<"# of colored points:\t"<<cloud_rgb.size()<<std::endl;
    if(cloud_rgb.size() > 100)    // 100 is minimum value standard to publish
    {
        sensor_msgs::PointCloud2 cloud_rgb_msg;
        pcl::toROSMsg(cloud_rgb, cloud_rgb_msg);
	    cloud_rgb_msg.header.frame_id = "map";
	    cloud_rgb_msg.header.stamp = ros::Time::now();
        pub.publish(cloud_rgb_msg);
    }
}

int main(int argc, char** argv)
{
    // Rotation from Lidar to Camera
    R << 0, 0, 1, 
        -1, 0, 0,
         0,-1, 0;
    R << 0,-1, 0, 
         0, 0,-1,
         1, 0, 0;
    tmpOusterCloudIn.reset(new pcl::PointCloud<OusterInput>());
    // Position differences from Lidar to Camera
    t << 0.15, 0.01, -0.353;     // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< TODO
    // Instrinsic parameter
    K << 643.29638671875,                 0, 644.2869873046875,   // f_x skew_cf_x c_x 
                       0, 642.4380493164062, 361.4469299316406,   //  0    f_y     c_y
                       0,                 0,               1.0;   //  0     0       1
    /* Command record
    - Installing realsense2 camera
    >> cd ~/catkin_ws/src
    >> git clone https://github.com/IntelRealSense/realsense-ros.git
    >> cd realsense-ros/
    >> git checkout ros1-legacy 
    >> cd ~/catkin_ws
    >> catkin_make
    Error occur
    >> sudo apt-get install ros-noetic-ddynamic-reconfigure
    >> catkin_make
    Error occur
    >> sudo apt-get install ros-noetic-realsense2-*
    >> catkin_make
    >> rospack find realsense2_camera
    >> roslaunch realsense2_camera rs_camera.launch

    - Check whether camera works well
    >> rqt_image_view 
    - Get information of camera (ex K)
    >> rostopic echo /camera/color/camera_info
    */

	// Initialize the ROS node
	ros::init(argc, argv, "data_converter");
    ros::NodeHandle nh;

    pub = nh.advertise<sensor_msgs::PointCloud2>("rbg_pointcloud", 1);

    message_filters::Subscriber<Image> image1_sub(nh, "/camera/color/image_raw", 1);
    message_filters::Subscriber<PointCloud2> lidar1_sub(nh, "/ouster/points", 1);

    typedef message_filters::sync_policies::ApproximateTime<Image, PointCloud2> MySyncPolicy;
    // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image1_sub, lidar1_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2));
    ros::spin(); 

	return 0;
}
