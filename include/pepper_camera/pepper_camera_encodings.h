// Pepper camera encodings
// Author: Philipp Allgeuer (philipp.allgeuer@uni-hamburg.de)

// Ensure header is only included once
#ifndef PEPPER_CAMERA_ENCODINGS_H
#define PEPPER_CAMERA_ENCODINGS_H

// Includes
#include <string>

// Pepper camera namespace
namespace pepper_camera
{
	// Image encodings namespace
	namespace image_encodings
	{
		// Planar YUV encodings
		const std::string I420 = "yuv_I420";  // https://www.fourcc.org/pixel-format/yuv-i420
		const std::string YV12 = "yuv_YV12";  // https://www.fourcc.org/pixel-format/yuv-yv12
		const std::string NV12 = "yuv_NV12";  // https://www.fourcc.org/pixel-format/yuv-nv12
		const std::string NV21 = "yuv_NV21";  // https://www.fourcc.org/pixel-format/yuv-nv21

		// Packed YUV encodings
		const std::string YUY2 = "yuv_YUY2";  // https://www.fourcc.org/pixel-format/yuv-yuy2 (= YUYV)
		const std::string YVYU = "yuv_YVYU";  // https://www.fourcc.org/pixel-format/yuv-yvyu
		const std::string UYVY = "yuv_UYVY";  // https://www.fourcc.org/pixel-format/yuv-uyvy
	}
}

#endif
// EOF
