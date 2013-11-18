#include "../include/phi_ardrone/phi_pilot.h"
#include "ros/ros.h"

#ifndef NULL
    #define NULL 0
#endif
//Sample publisher
//   ros::Publisher pub = n.advertise<turtlesim::Velocity>("/turtle1/command_velocity", 10);




int main(int argc, char **argv)
{
    ros::init(argc, argv, "phi_keyboard_pilot");

    AR_Drone drone;

    drone.keyboardControlLoop();

    return 0;
}
