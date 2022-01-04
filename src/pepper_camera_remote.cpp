// Pepper camera remote class
// Author: Philipp Allgeuer (philipp.allgeuer@uni-hamburg.de)

// Includes
#include <pepper_camera/pepper_camera_remote.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <sstream>
#include <netdb.h>

// Namespaces
using namespace pepper_camera;

//
// Remote commands
//

// Connect to the Pepper and trigger the required camera to start streaming
bool PepperCameraRemote::start_camera(const std::string& remote_host, int remote_port, int udp_port, int device, int width, int height, int quality)
{
	// Connect to the Pepper
	int connection = connect(remote_host, remote_port);
	if(connection < 0)
		return false;

	// Get the local IP address that the Pepper should start streaming camera images to
	uint32_t udp_ip = get_send_addr(connection);
	if(udp_ip == 0U)
	{
		disconnect(connection);
		return false;
	}

	// Send the command to start the camera stream
	StartCameraStruct S = {'B', (uint8_t) device, (uint16_t) udp_port, udp_ip, (uint16_t) width, (uint16_t) height, (uint8_t) quality};
	bool success = send_data(connection, S);

	// Disconnect from the Pepper
	disconnect(connection);

	// Return whether success
	return success;
}

// Connect to the Pepper and trigger the required camera to stop streaming
bool PepperCameraRemote::stop_camera(const std::string& remote_host, int remote_port, int udp_port, int device)
{
	// Connect to the Pepper
	int connection = connect(remote_host, remote_port);
	if(connection < 0)
		return false;

	// Get the local IP address that the Pepper should stop streaming camera images to
	uint32_t udp_ip = get_send_addr(connection);
	if(udp_ip == 0U)
	{
		disconnect(connection);
		return false;
	}

	// Send the command to stop the camera stream
	StopCameraStruct S = {'E', (uint8_t) device, (uint16_t) udp_port, udp_ip};
	bool success = send_data(connection, S);

	// Disconnect from the Pepper
	disconnect(connection);

	// Return whether success
	return success;
}

//
// Connection utilities
//

// Connect to the Pepper robot
int PepperCameraRemote::connect(const std::string& remote_host, int remote_port)
{
	// Create a new socket
	int sockfd = socket(AF_INET, SOCK_STREAM, 0);
	if(sockfd < 0)
		return -1;

	// Resolve the remote address
	bool found_address = false;
	sockaddr_storage sas;
	socklen_t sas_len;
	in_addr ina;
	if(inet_pton(AF_INET, remote_host.c_str(), &ina) == 1)
	{
		sockaddr_in* address = (sockaddr_in*) &sas;
		sas_len = sizeof(sockaddr_in);
		address->sin_family = AF_INET;
		address->sin_port = htons(remote_port);
		address->sin_addr.s_addr = ina.s_addr;
		found_address = true;
	}
	else
	{
		addrinfo hints, *addr;
		std::memset(&hints, 0, sizeof(hints));
		hints.ai_family = AF_INET;
		hints.ai_socktype = SOCK_STREAM;
		if(getaddrinfo(remote_host.c_str(), NULL, &hints, &addr) == 0)
		{
			sockaddr_in* address = (sockaddr_in*) &sas;
			sas_len = sizeof(sockaddr_in);
			std::memcpy(address, addr->ai_addr, addr->ai_addrlen);
			address->sin_family = addr->ai_family;
			address->sin_port = htons(remote_port);
			freeaddrinfo(addr);
			found_address = true;
		}
	}

	// Connect to the remote address
	if(!(found_address && ::connect(sockfd, (sockaddr*) &sas, sas_len) == 0))
	{
		disconnect(sockfd);
		return -1;
	}

	// Return success
	return sockfd;
}

// Get the sending IP address of the local machine
uint32_t PepperCameraRemote::get_send_addr(int connection)
{
	// Get the local IP address
	sockaddr_in addr;
	socklen_t addr_len = sizeof(sockaddr_in);
	if(getsockname(connection, (sockaddr*) &addr, &addr_len) == 0)
		return ntohl(addr.sin_addr.s_addr);
	else
		return 0U;
}

// Disconnect from the Pepper robot
void PepperCameraRemote::disconnect(int connection)
{
	// Close the socket
	if(connection >= 0)
	{
		shutdown(connection, SHUT_RDWR);
		close(connection);
	}
}
// EOF
