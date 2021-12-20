// Pepper camera node
// Author: Philipp Allgeuer (philipp.allgeuer@uni-hamburg.de)

// Includes
#include <pepper_camera/pepper_camera.h>

// Main function
int main(int argc, char** argv)
{
	// Initialise the ROS node
	ros::init(argc, argv, "pepper_camera");
	ros::NodeHandle nh, nh_private("~");

	// Run the node
	pepper_camera::PepperCamera pcamera(nh, nh_private);
	pcamera.run();

	// Return success
	return 0;
}
// EOF
