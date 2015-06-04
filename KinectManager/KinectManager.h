#pragma once

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