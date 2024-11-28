#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

// Callback function to handle incoming image messages
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	try
	{
		// Convert the ROS image mesage to a cv::Mat using cv_bridge
		cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

		// Get the OpenCV Mat object
		cv::Mat image = cv_ptr->image;

		//Display the image using OpenCV
		cv::imshow("Camera Image", image);
		cv::waitKey(1); // Update the image window
	}
	catch(cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
	}
	
}

int main(int argc, char** argv)
{
	// Initialize the ROS node
	ros::init(argc, argv, "image_converter");
	ros::NodeHandle nh;

	// Subscribe to the image topic
	ros::Subscriber sub = nh.subscribe("/camera/image_raw", 1, imageCallback);

	// Create an OpenCV window to display the image
	cv::namedWindow("Camera Image", cv::WINDOW_NORMAL);

	// ROS loop
	ros::spin();

	// Close the OpenCV window when exiting
	cv::destroyAllWindows();

	return 0;
}