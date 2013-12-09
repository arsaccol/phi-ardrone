#ifndef PHI_PILOT_H
#define	PHI_PILOT_H

#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "geometry_msgs/Twist.h"
#include <SDL/SDL.h>

class AR_Drone
{
public:
    AR_Drone();
    ~AR_Drone();

    void Takeoff();
	void Land();

    void moveForward(float speed);
    void moveBackward(float speed);
    void turnLeft(float angle);
    void turnRight(float angle);
    void setVerticalVelocityUp(float speed);
    void setVerticalVelocityDown(float speed);


    void resetVelocityH();
    void resetVelocityV();
    void resetVelocity();

    void keyboardControlLoop();
	void trackingControlLoop();

private:


    ros::NodeHandle Node;
    int pub_rate;
    bool shouldQuit;
	bool tracking;


    ros::Publisher pub_Takeoff;
    ros::Publisher pub_Landing;
    ros::Publisher pub_Velocity;

	ros::ServiceClient srvc_trackingStart;
	ros::ServiceClient srvc_trackingStop;

    std_msgs::Empty empty_msg;
    geometry_msgs::Twist commandVelocity_msg;

    SDL_Surface* input_window;
    SDL_Event events;

    void switchOnKeyDown();
    void switchOnKeyUp();
};


#endif
