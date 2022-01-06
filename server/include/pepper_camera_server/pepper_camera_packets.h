// Pepper camera packet structs
// Author: Philipp Allgeuer (philipp.allgeuer@uni-hamburg.de)

// Ensure header is only included once
#ifndef PEPPER_CAMERA_PACKETS_H
#define PEPPER_CAMERA_PACKETS_H

// Includes
#include <cstdint>

// Pepper camera namespace
namespace pepper_camera
{
	// Constants
	const int MAX_VIDEO_DEVICE = 255;

	// Start camera packet struct
	struct __attribute((__packed__)) StartCameraStruct
	{
		char cmd;         // Must be 'B'
		uint8_t device;   // X = /dev/videoX
		uint16_t port;    // Range 1-65535
		uint32_t ip;      // Network byte order
		uint16_t width;   // Range 1-
		uint16_t height;  // Range 1-
		uint8_t quality;  // Range 1-100
	};

	// Stop camera packet struct
	struct __attribute((__packed__)) StopCameraStruct
	{
		char cmd;        // Must be 'E'
		uint8_t device;  // X = /dev/videoX
		uint16_t port;   // Range 1-65535
		uint32_t ip;     // Network byte order
	};
}

#endif
// EOF
