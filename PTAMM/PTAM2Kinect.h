#ifndef PTAM2KINECT
#define PTAM2KINECT

#if 0
#include <Ole2.h>
#include <Windows.h>
#include <NuiApi.h>
#include "Keyframe.h"
#include <TooN\TooN.h>
#include <opencv2\opencv.hpp>

namespace KINECT
{
#define CAPTURE_SIZE_X	640
#define CAPTURE_SIZE_Y	480

	void calcPTAMfromKinect(PTAMM::KeyFrame*, NUI_DEPTH_IMAGE_POINT*);
	void calcPTAMfromKinect(std::vector<TooN::Vector<4>>, NUI_DEPTH_IMAGE_POINT*);
	TooN::Matrix<4,4> getPTAMfromKinect();
	cv::Mat getPTAMfromKinect_mat();
	void setPTAMfromKinect(TooN::Matrix<4,4>);
}
#else 

#include "KeyFrame.h"
#include <TooN\TooN.h>
#include <opencv2\opencv.hpp>
#include "KinectManager.h"

namespace KINECT
{
	void calcPTAMfromKinect(PTAMM::KeyFrame*, DepthXY *);
	void calcPTAMfromKinect(cv::Mat& mvPTAM3DPoints, DepthXY * dDepthMap);
	TooN::Matrix<4,4> getPTAMfromKinect();
	cv::Mat getPTAMfromKinect_mat();
	void setPTAMfromKinect(TooN::Matrix<4,4>);
	void GridProjection(TooN::SE3<> mse3CfW, std::vector<TooN::Vector<4>> * gp1, std::vector<TooN::Vector<4>> * gp2, unsigned int width, unsigned int height);
}

#endif

#endif