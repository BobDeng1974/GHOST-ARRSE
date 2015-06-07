#pragma once
#include "ghostsettings.h"

#if GHOST_INPUT == INPUT_WEBCAM
#endif

#if GHOST_INPUT == INPUT_KINECT

#include <Windows.h>
#include <Ole2.h>


#include <NuiApi.h>
#include <NuiImageCamera.h>
#include <NuiSensor.h>

#define KINECT_CAPTURE_SIZE_X 640
#define KINECT_CAPTURE_SIZE_Y 480

enum Update{
	Color = 0x01,
	Depth = 0x02
};

class KinectManager{
public:
	~KinectManager();

	HRESULT InitializeDefaultSensor();

	void Update(unsigned int options);

	unsigned char * GetColorRGBX();
	unsigned short * GetDepth();
	bool IsOpened();

	INuiSensor * GetSensor();

	static KinectManager * GetKinectManager();

private:
	KinectManager();

	void UpdateColor();
	void UpdateDepth();

	INuiSensor * sensor;
	HANDLE rgb_stream;
	HANDLE depth_stream;

	unsigned char * rgbx_data;
	unsigned short * depth_data;

	bool is_opened;
};

#endif

#if GHOST_INPUT == INPUT_KINECT2

#include <opencv2\opencv.hpp>
#include "definitions.h"
#include "Skeleton.h"

#if GHOST_CAPTURE == CAPTURE_KINECT2
#define CAPTURE_SIZE_X 640
#define CAPTURE_SIZE_Y 480
#else
#define CAPTURE_SIZE_X	640
#define CAPTURE_SIZE_Y	480
#endif
#define FPS				60

namespace KINECT{

	bool doCalib();
	bool init();
	bool release();
	void updateFrames();
	cv::Mat getColorFrame();
	CroppedCvMat getPlayerColorFrame();
	cv::Mat getDepthFrame();
	Skeleton getSkeleton();
	cv::Mat getUserColorFrame();
	bool skeletonIsGood();
	float getSkeletonGoodness(Skeleton * s);

	int getCenterJoint();
	int getHeadJoint();
	void initMapping(Mapping * mapping);

#if INIT_KINECT
	//cv::Vec2f toScreen(cv::Vec3f);
#endif

	cv::Vec3f calculateFacing(Skeleton * s);
	bool checkTracked(int state);
	int initSkeletonScore(Skeleton kinectPoints);

	//save/load device
	void saveParams(std::string);
	void loadParams(std::string);

	//adds new joint to OpenNI skeletons
	void augmentSkeleton(Skeleton * s);

	//oldskool, remove later
	struct DepthXY{
		long depth;
		long x;
		long y;
	};

	void getKinectData_depth_raw(DepthXY * depthPoints);

	cv::Vec3f mapDepthToSkeletonPoint(DepthXY d);

	////takes 4xN Mat of camera points and uses Kinect function to map it to 2xN Mat of color space points
	//cv::Mat mapCameraPointsToColorPoints(cv::Mat cameraPoints);
	//
	////use for inverse mapping
	//cv::Mat makeRays(cv::Mat pts2d);

	cv::Mat calculateCameraParameters();
	cv::Mat loadCameraParameters();

	//void GridProjection(TooN::SE3<> mse3CfW, std::vector<TooN::Vector<4>> * gp1, std::vector<TooN::Vector<4>> * gp2); //moved to PTAM2Kinect

	//temp function for approximating facing. s=1: shoulders, s=2:hips
	std::pair<int, int> facingHelper(int s);
}
#endif