#ifndef PHI_FOLLOWER_H
#define PHI_FOLLOWER_H

#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "std_srvs/Empty.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/video.hpp"
#include "opencv2/core/core.hpp"

#include "phi_locate_circle.h"

class AR_Drone_Follower
{
public:
	AR_Drone_Follower();
	~AR_Drone_Follower();
	void Run();
private:
	static void _camera_subCb(const sensor_msgs::ImageConstPtr& msg);
private:
	bool shouldRun;

	// cv::VideoCapture camera;
	ros::NodeHandle node;
	ros::Subscriber camera_sub;

	ros::ServiceServer srvs_trackingStart;
	ros::ServiceServer srvs_trackingStop;

	//cv_bridge::CvImage frame;


};


#endif
