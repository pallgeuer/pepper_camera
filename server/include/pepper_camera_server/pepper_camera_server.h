// Pepper camera server class
// Author: Philipp Allgeuer (philipp.allgeuer@uni-hamburg.de)

// Ensure header is only included once
#ifndef PEPPER_CAMERA_SERVER_H
#define PEPPER_CAMERA_SERVER_H

// Includes
// TODO

// Pepper camera namespace
namespace pepper_camera
{
	// Pepper camera server class
	class PepperCameraServer
	{
	public:
		// Constants
		static const int DEFAULT_CMD_PORT = 3006;

		// Constructor
		explicit PepperCameraServer(int cmd_port = DEFAULT_CMD_PORT);

		// Run
		void run();
	};
}

#endif
// EOF
