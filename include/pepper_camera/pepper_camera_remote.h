// Pepper camera remote class
// Author: Philipp Allgeuer (philipp.allgeuer@uni-hamburg.de)

// Ensure header is only included once
#ifndef PEPPER_CAMERA_REMOTE_H
#define PEPPER_CAMERA_REMOTE_H

// Includes
#include <sys/socket.h>
#include <string>

// Pepper camera namespace
namespace pepper_camera
{
	// Pepper camera remote class
	class PepperCameraRemote
	{
	public:
		// Remote commands
		static bool start_camera(const std::string& remote_host, int remote_port, int udp_port, int device, int width, int height, int quality);
		static bool stop_camera(const std::string& remote_host, int remote_port, int udp_port, int device);

	private:
		// Constructor
		PepperCameraRemote() = delete;

		// Connection utilities
		static int connect(const std::string& remote_host, int remote_port);
		static uint32_t get_send_addr(int connection);
		template<class S> static bool send_data(int connection, const S& data) { return send(connection, &data, sizeof(S), 0) == sizeof(S); }
		static void disconnect(int connection);
	};

	// Start camera packet struct
	struct __attribute((__packed__)) StartCameraStruct
	{
		char cmd;         // Must be 'B'
		uint8_t device;   // 0 = /dev/video0, 1 = /dev/video1
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
		uint8_t device;  // 0 = /dev/video0, 1 = /dev/video1
		uint16_t port;   // Range 1-65535
		uint32_t ip;     // Network byte order
	};
}

#endif
// EOF
