// Pepper camera class
// Author: Philipp Allgeuer (philipp.allgeuer@uni-hamburg.de)

// Ensure header is only included once
#ifndef PEPPER_CAMERA_H
#define PEPPER_CAMERA_H

// Includes - ROS
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>

// Includes - GStreamer
extern "C"
{
#include <gst/gst.h>
}

// Defines
#define PC_G_SOURCE_FUNC(f) ((GSourceFunc) (void (*)(void)) (f))  // Note: G_SOURCE_FUNC is only available as of GLib 2.58

// Pepper camera namespace
namespace pepper_camera
{
	// Pepper camera class
	class PepperCamera
	{
	public:
		// Constructor
		PepperCamera(ros::NodeHandle& nh_interface, ros::NodeHandle& nh_param);

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
		std::string m_camera_name;
		std::string m_camera_frame;
		std::string m_camera_info_url;
		double m_time_offset;
		int m_queue_size_mb;
		int m_publish_queue_size;

		// ROS variables
		ros::NodeHandle& m_nh_interface;
		ros::NodeHandle& m_nh_param;
		ros::Time m_pipeline_time_offset;
		camera_info_manager::CameraInfoManager m_camera_info_manager;
		image_transport::ImageTransport m_image_transport;
		ros::Publisher m_pub_camera_info;
		ros::Time m_pub_camera_info_stamp;
		ros::Publisher m_pub_jpeg;
		ros::Publisher m_pub_yuv;
		image_transport::Publisher m_pub_rgb;

		// ROS utilities
		void publish_camera_info(const ros::Time& stamp);

		// GStreamer pipeline elements class
		class GSElements {
		public:
			GSElements() = default;  // Initialises all members to NULL
			void clear();
			GstElement* udpsrc;
			GstElement* rtpjpegdepay;
			GstElement* tee_jpeg;
			GstPad*     publish_jpeg_pad;
			GstElement* publish_jpeg_queue;
			GstElement* publish_jpeg;
			GstPad*     record_jpegs_pad;
			GstElement* record_jpegs_queue;
			GstElement* record_jpegs;
			GstPad*     record_mjpeg_pad;
			GstElement* record_mjpeg_queue;
			GstElement* record_mjpeg_mux;
			GstElement* record_mjpeg;
			GstPad*     jpegdec_pad;
			GstElement* jpegdec_queue;
			GstElement* jpegdec;
			GstElement* tee_yuv;
			GstPad*     publish_yuv_pad;
			GstElement* publish_yuv_queue;
			GstElement* publish_yuv;
			GstPad*     publish_rgb_pad;
			GstElement* publish_rgb_queue;
			GstElement* publish_rgb_convert;
			GstElement* publish_rgb;
			GstPad*     record_h264_pad;
			GstElement* record_h264_queue;
			GstElement* record_h264_enc;
			GstElement* record_h264_mux;
			GstElement* record_h264;
			GstPad*     preview_pad;
			GstElement* preview_queue;
			GstElement* preview;
		};

		// GStreamer variables
		guint m_sigint_callback_id = 0U;
		GSElements* m_elem = NULL;
		GstElement* m_pipeline = NULL;
		bool m_pipeline_stalled = false;
		GMainLoop* m_main_loop = NULL;

		// GStreamer callbacks
		static gboolean sigint_callback(PepperCamera* pc);
		static void stream_eos_callback(GstBus* bus, GstMessage* msg, PepperCamera* pc);
		static void stream_warning_callback(GstBus* bus, GstMessage* msg, PepperCamera* pc);
		static void stream_error_callback(GstBus* bus, GstMessage* msg, PepperCamera* pc);
		static void queue_overrun_callback(GstElement* queue, PepperCamera* pc);
		static GstFlowReturn publish_jpeg_callback(GstElement* appsink, PepperCamera* pc);
		static GstFlowReturn publish_yuv_callback(GstElement* appsink, PepperCamera* pc);
		static GstFlowReturn publish_rgb_callback(GstElement* appsink, PepperCamera* pc);

		// GStreamer utilities
		void configure_queue(GstElement* queue);
		static gboolean link_tee_queue(GstElement* tee, GstElement* queue, GstPad*& tee_pad);
		static gboolean gst_bin_add_ref(GstBin *bin, GstElement *element);
		static void gst_bin_add_many_ref(GstBin *bin, GstElement *element1, ...) G_GNUC_NULL_TERMINATED;
		static void gst_object_unref_safe(GstObject** object_ptr);
		void cancel_main_loop();
		void quit_main_loop();

		// Misc utilities
		static bool ensure_extension(std::string& str, const std::string& ext);
	};
}

#endif
// EOF
