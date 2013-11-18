#include "../include/phi_ardrone/phi_follower.h"

AR_Drone_Follower::AR_Drone_Follower()
{
	shouldRun = false;
	// camera.open(0);

	/* Toggle the camera later on
	 *
	ros::ServiceClient toggleCam_client = node.serviceClient("ardrone/togglecam");
	toggleCam_client.call();
	*/

	camera_sub = node.subscribe("ardrone/image_raw", 200, camera_subCb);

	ros::spin();


}

AR_Drone_Follower::~AR_Drone_Follower()
{

}

void AR_Drone_Follower::camera_subCb(const sensor_msgs::ImageConstPtr &msg)
{
	static AR_Drone_LocateCircle locator;
	cv::Point detected_center;

	cv::Mat frame;
	cv_bridge::CvImagePtr cv_framePtr;
	cv_framePtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	frame = cv_framePtr->image;

	// Visualization
	detected_center = locator.Iteration(frame);

	const cv::Scalar color(0);


	cv::imshow("ardrone/image_raw", frame);
	cv::circle(frame, detected_center, 6, color, 3);
		cv::waitKey(3);


}
