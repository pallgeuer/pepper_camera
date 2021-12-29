// Pepper camera node
// Author: Philipp Allgeuer (philipp.allgeuer@uni-hamburg.de)

// Includes
#include <pepper_camera/pepper_camera.h>

// Main function
int main(int argc, char** argv)
{
	// Initialise the ROS node
	ros::init(argc, argv, "pepper_camera");
	ros::NodeHandle nh_interface, nh_param("~");

	// Start an asynchronous ROS spinner
	ros::AsyncSpinner ros_spinner(2);
	ros_spinner.start();

	// Run the Pepper camera streamer
	pepper_camera::PepperCamera pcamera(nh_interface, nh_param);
	pcamera.run();

	// Stop the asynchronous ROS spinner
	ros_spinner.stop();

	// Shut down ROS (would happen anyway when last node handle is destructed)
	ros::shutdown();

	// Return success
	return 0;
}
// EOF
