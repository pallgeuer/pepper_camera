// Pepper camera node
// Author: Philipp Allgeuer (philipp.allgeuer@uni-hamburg.de)

// Includes
#include <ros/ros.h>
#include <pepper_camera/pepper_camera.h>

// Main function
int main(int argc, char** argv)
{
	// Initialise the ROS node
	ros::init(argc, argv, "pepper_camera");
	ros::NodeHandle nh, nh_private("~");

	// Run the node
	pepper_camera::PepperCamera pepper_camera(nh, nh_private);
	pepper_camera.run();

	// Return success
	return 0;
}
// EOF
