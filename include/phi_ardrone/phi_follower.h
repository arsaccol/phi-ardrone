#ifndef PHI_FOLLOWER_H
#define PHI_FOLLOWER_H

#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/video.hpp"
class AR_Drone_Follower
{
public:
	AR_Drone_Follower();
	~AR_Drone_Follower();

private:
	cv::VideoCapture camera;
	ros::NodeHandle node;
	ros::Subscriber subscriber;


	cv_bridge::CvImage frame;


};


#endif
