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
		void reset_config();
		bool configure();

		// Stream management
		bool init_stream();
		bool run_stream();
		void cleanup_stream();

	private:
		// ROS node handles
		const ros::NodeHandle& m_nh_interface;
		const ros::NodeHandle& m_nh_param;

		// Configuration
		int m_port;
		bool m_auto_retry;
		bool m_publish_jpeg;
		bool m_publish_yuv;
		bool m_publish_rgb;
		std::string m_record_jpegs;
		int m_record_jpegs_max;
		std::string m_record_mjpeg;
		std::string m_record_h264;
		int m_record_h264_bitrate;
		int m_record_h264_speed;
		std::string m_record_h264_profile;
		bool m_preview;

		// GStreamer pipeline elements class
		class GSElements {
		public:
			GSElements() = default;  // Initialises all members to NULL
			void clear();
			GstElement* udpsrc;
			GstElement* rtpjpegdepay;
			GstElement* tee_jpeg;
			GstElement* publish_jpeg_queue;
			GstElement* publish_jpeg;
			GstElement* record_jpegs_queue;
			GstElement* record_jpegs;
			GstElement* record_mjpeg_queue;
			GstElement* record_mjpeg_mux;
			GstElement* record_mjpeg;
			GstElement* jpegdec_queue;
			GstElement* jpegdec;
			GstElement* tee_yuv;
			GstElement* publish_yuv_queue;
			GstElement* publish_yuv;
			GstElement* publish_rgb_queue;
			GstElement* publish_rgb_convert;
			GstElement* publish_rgb;
			GstElement* record_h264_queue;
			GstElement* record_h264_enc;
			GstElement* record_h264_mux;
			GstElement* record_h264;
			GstElement* preview_queue;
			GstElement* preview;
		};

		// GStreamer variables
		GSElements* m_elem = NULL;
		GstElement* m_pipeline = NULL;
		GMainLoop* m_main_loop = NULL;

		// GStreamer signal callbacks
		static bool quit_main_loop_callback(GMainLoop* main_loop);
		static void stream_error_callback(GstBus* bus, GstMessage* msg, GMainLoop* main_loop);
		static GstFlowReturn publish_jpeg_callback(GstElement* appsink, PepperCamera* pc);
		static GstFlowReturn publish_yuv_callback(GstElement* appsink, PepperCamera* pc);
		static GstFlowReturn publish_rgb_callback(GstElement* appsink, PepperCamera* pc);

		// Utilities
		static bool ensure_extension(std::string& str, const std::string& ext);
		static gboolean gst_bin_add_ref(GstBin *bin, GstElement *element);
		static void gst_bin_add_many_ref(GstBin *bin, GstElement *element1, ...) G_GNUC_NULL_TERMINATED;
	};
}

#endif
// EOF
