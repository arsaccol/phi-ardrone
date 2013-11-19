#include "../include/phi_ardrone/phi_follower.h"

AR_Drone_Follower::AR_Drone_Follower()
{
	shouldRun = false;
	// camera.open(0);

	std_srvs::Empty toggleCamMsg;




	camera_sub = node.subscribe("ardrone/image_raw", 200, _camera_subCb);
	ros::ServiceClient toggleCam_client = node.serviceClient<std_srvs::Empty>("ardrone/togglecam");
	// toggleCam_client.call(toggleCamMsg);
	ros::spin();


}

AR_Drone_Follower::~AR_Drone_Follower()
{

}

void AR_Drone_Follower::_camera_subCb(const sensor_msgs::ImageConstPtr &msg)
{
	static AR_Drone_LocateCircle locator;

	cv::Mat frame;



	cv::Point origin;
	cv::Scalar blueColor(255, 0, 0);
	cv::Point detected_center;




	cv_bridge::CvImagePtr cv_framePtr;
	cv_framePtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	frame = cv_framePtr->image;
		origin.x = frame.cols / 2;
		origin.y = frame.rows / 2;

	detected_center = locator.Iteration(frame);
	if(detected_center.x == 0 && detected_center.y == 0)
	{
		detected_center.x = frame.cols / 2;
		detected_center.y = frame.rows / 2;
	}
	cv::line(frame, origin, detected_center, blueColor, 2, 8);

	detected_center.x = frame.cols / 2;
	detected_center.y = frame.rows / 2;

	cv::imshow("ardrone/image_raw", frame);

		cv::waitKey(3);
}
