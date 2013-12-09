#include "../include/phi_ardrone/phi_pilot.h"

#include <SDL/SDL.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/Image.h"
#include "geometry_msgs/Twist.h"
#include "std_srvs/Empty.h"

AR_Drone::AR_Drone()
{

    //ros::init(argc, argv, "ar_pilot_main");
    shouldQuit = false;
	tracking = false;
    pub_rate = 200; // Hz
    pub_Takeoff = Node.advertise<std_msgs::Empty>("ardrone/takeoff", pub_rate);
    pub_Landing = Node.advertise<std_msgs::Empty>("ardrone/land", pub_rate);
	pub_Velocity = Node.advertise<geometry_msgs::Twist>("cmd_vel", pub_rate);

	srvc_trackingStart = Node.serviceClient<std_srvs::Empty>("phi_ardrone/tracking");



// Should be advertised only when a "tracking" key is pressed.
//	pub_Tracking = Node.advertise<std_msgs::Empty>("phi_ardrone/tracking", pub_rate);

}

AR_Drone::~AR_Drone()
{
}

void AR_Drone::Takeoff()
{
    // TODO:
    //	Check if not flying already
    pub_Takeoff.publish(empty_msg);
}

void AR_Drone::Land()
{
    pub_Landing.publish(empty_msg);
}

void AR_Drone::moveForward(float speed)
{
    //	if(speed >= 0.0f && speed <= 1.0f)
    {
        commandVelocity_msg.linear.x = speed;
        pub_Velocity.publish(commandVelocity_msg);

    }
}

void AR_Drone::moveBackward(float speed)
{
    //	if(speed >= -1.0f && speed <= 0.0f)
    {
        commandVelocity_msg.linear.x = speed;
        pub_Velocity.publish(commandVelocity_msg);
    }
}

void AR_Drone::turnLeft(float angle)
{
    //	if(angle >= -1.0f && angle <= 0.0f)
    {
        commandVelocity_msg.angular.z = angle;
        pub_Velocity.publish(commandVelocity_msg);
    }
}

void AR_Drone::turnRight(float angle)
{
    //	if(angle >= -1.0f && angle <= 1.0f)
    {
        commandVelocity_msg.angular.z = angle;
        pub_Velocity.publish(commandVelocity_msg);
    }
}

void AR_Drone::setVerticalVelocityUp(float speed)
{
    //	if(speed >= 0.0f && speed <= 1.0f)
    {
        commandVelocity_msg.linear.z = speed;
        pub_Velocity.publish(commandVelocity_msg);
    }
}

void AR_Drone::setVerticalVelocityDown(float speed)
{
    //	if(speed >= -1.0f && speed <= 0.0f)
    {
        commandVelocity_msg.linear.z = speed;
		pub_Velocity.publish(commandVelocity_msg);
    }
}

void AR_Drone::resetVelocityH()
{
    std::cout << "Resetting horizontal velocity" << std::endl;
    commandVelocity_msg.linear.x = 0;
    commandVelocity_msg.linear.y = 0;
    commandVelocity_msg.angular.z = 0;

    pub_Velocity.publish(commandVelocity_msg);
}

void AR_Drone::resetVelocityV()
{
    std::cout << "Resetting vertical velocity" << std::endl;
    commandVelocity_msg.linear.z = 0;
    pub_Velocity.publish(commandVelocity_msg);
}

void AR_Drone::resetVelocity()
{
    resetVelocityV();
    resetVelocityH();
}

void AR_Drone::switchOnKeyDown()
{
    switch(events.key.keysym.sym)
    {
    case SDLK_ESCAPE:
        Land();
        SDL_Quit();
        shouldQuit = true;
        break;

    case SDLK_SPACE:
        std::cout << "Taking off" << std::endl;
		Takeoff();
        break;

    case SDLK_l:
        std::cout << "Landing" << std::endl;
        Land();
        break;

    case SDLK_q:
        setVerticalVelocityUp(0.5);
        std::cout << "Thrusting up" << std::endl;
        break;

    case SDLK_z:
        setVerticalVelocityDown(-0.5);
        std::cout << "Thrusting down" << std::endl;
        break;

    case SDLK_w:
        moveForward(0.3);
        std::cout << "Moving forward" << std::endl;
        break;

    case SDLK_s:
        moveBackward(-0.3);
        std::cout << "Moving backward" << std::endl;
        break;

	case SDLK_a:
        turnLeft(0.7);
        std::cout << "Turning left" << std::endl;
        break;

    case SDLK_d:
        turnRight(-0.7);
        std::cout << "Turning right" << std::endl;
        break;

	case SDLK_f:
		if(!tracking)
		{
			tracking = true;
		}
		else
		{
			tracking = false;
		}
		break;

    default:
        break;
    }
}

void AR_Drone::switchOnKeyUp()
{
    switch(events.key.keysym.sym)
    {
    case SDLK_q:
    case SDLK_z:
        std::cout << "Resetting vertical velocity" << std::endl;
        resetVelocityV();
        break;

    case SDLK_w:
    case SDLK_s:
    case SDLK_a:
    case SDLK_d:
    case SDLK_UP:
    case SDLK_DOWN:
    case SDLK_LEFT:
    case SDLK_RIGHT:
        //			std::cout << "Resetting horizontal velocity" << std::endl;
        resetVelocityH();
        break;
    default:
        break;
    }
}

void AR_Drone::keyboardControlLoop()
{
    SDL_Init(SDL_INIT_EVERYTHING);

    input_window = SDL_SetVideoMode(320, 240, 32, SDL_HWSURFACE);


    while(Node.ok() && shouldQuit == false)
    {
        SDL_PollEvent(&events);
        //		SDL_WaitEvent(&events);
        //		std::cout << "key press/release " << events.type << std::endl;

        switch(events.type)
        {
        case SDL_KEYDOWN:
            switchOnKeyDown();
            break;


        case SDL_KEYUP:
            switchOnKeyUp();
            break;
        }
        events.type = SDL_NOEVENT;
        ros::spinOnce();
    }


    SDL_Quit();
}

