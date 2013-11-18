
#include "../include/phi_ardrone/phi_locate_circle.h"
#include "../include/phi_ardrone/phi_follower.h"
#include "ros/ros.h"

#ifndef NULL
    #define NULL 0
#endif

using namespace std;
/*
void camera_callback(const sensor_msgs::Image::ConstPtr& ros_img);

int main(int argc, char** argv)
{
    ros::init(argc, argv, "phi_locate_circle");
	ros::NodeHandle node;

    AR_Drone_LocateCircle locator;

	cv_bridge::CvImage frame;

	ros::Subscriber subscriber = node.subscribe("ardrone/bottom/image_raw", 200, camera_callback);





    ros::spin();
    return 0;
}


void camera_callback(const sensor_msgs::Image::ConstPtr& ros_img)
{
    Point center_detected;

    //std::cout << "Receiving image messages!" << std::endl;
    cv_framePtr = cv_bridge::toCvCopy(ros_img, sensor_msgs::image_encodings::BGR8);
    image = cv_framePtr->image;

    center_detected = Iteration();

    if (found_circle.x != 0 && found_circle.y != 0)

    cout << "centro:" << found_circle.x << " e " << found_circle.y << endl;

    cv::waitKey(3);
}


*/


int main(int argc, char** argv)
{
	ros::init(argc, argv, "phi_locate_circle");
	AR_Drone_Follower flw;


	return 0;
}
