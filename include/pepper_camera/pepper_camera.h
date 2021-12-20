// Pepper camera class
// Author: Philipp Allgeuer (philipp.allgeuer@uni-hamburg.de)

// Ensure header is only included once
#ifndef PEPPER_CAMERA_H
#define PEPPER_CAMERA_H

// Includes - ROS
#include <ros/ros.h>

// Includes - GStreamer
extern "C"
{
#include <gst/gst.h>
}

// Pepper camera namespace
namespace pepper_camera
{
	// Pepper camera class
	class PepperCamera
	{
	public:
		// Constructor
		PepperCamera(const ros::NodeHandle& nh_interface, const ros::NodeHandle& nh_param);

		// Run
		void run();

		// Configuration
		bool configure();

		// Stream management
		bool init_stream();
		bool run_stream();
		void cleanup_stream();

	private:
		// GStreamer signal callbacks
		static void error_callback(GstBus* bus, GstMessage* msg, GMainLoop* main_loop);

		// ROS node handles
		const ros::NodeHandle& m_nh_interface;
		const ros::NodeHandle& m_nh_param;

		// Configuration
		int m_port;
		bool m_auto_reopen;

		// GStreamer pipeline elements class
		class GSElements {
		public:
			GSElements() = default;

			GstElement* udpsrc;
			GstElement* rtpjpegdepay;
			GstElement* jpegdec;
			GstElement* fpsdisplaysink;

			bool valid() const {
				return (
					udpsrc != NULL &&
					rtpjpegdepay != NULL &&
					jpegdec != NULL &&
					fpsdisplaysink != NULL
				);
			}
		};

		// GStreamer
		GSElements* m_elem;
		GstElement* m_pipeline;
		GMainLoop* m_main_loop;
	};
}

#endif
// EOF
