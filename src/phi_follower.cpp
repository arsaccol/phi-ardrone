#include "../include/phi_ardrone/phi_follower.h"

AR_Drone_Follower::AR_Drone_Follower()
{
	subscriber = node.subscribe("ardrone/image_raw");


}

AR_Drone_Follower::~AR_Drone_Follower()
{

}
