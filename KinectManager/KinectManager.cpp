#include "KinectManager.h"

KinectManager::KinectManager():
is_opened(false)
{
	rgbx_data = new unsigned char[KINECT_CAPTURE_SIZE_X * KINECT_CAPTURE_SIZE_Y * 4];
	depth_data = new unsigned short[KINECT_CAPTURE_SIZE_X * KINECT_CAPTURE_SIZE_Y];
}

KinectManager::~KinectManager(){
	if (rgbx_data){
		delete[] rgbx_data;
	}
	if (depth_data){
		delete[] depth_data;
	}
}

HRESULT KinectManager::InitializeDefaultSensor(){
	HRESULT hr;

	int numSensors;
	hr = NuiGetSensorCount(&numSensors) < 0 || numSensors < 1;
	if (FAILED(hr)) return hr;

	hr = NuiCreateSensorByIndex(0, &sensor) < 0;
	if (FAILED(hr)) return hr;

	// Initialize sensor
	hr = sensor->NuiInitialize(NUI_INITIALIZE_FLAG_USES_DEPTH | NUI_INITIALIZE_FLAG_USES_COLOR);
	hr = sensor->NuiImageStreamOpen(
		NUI_IMAGE_TYPE_COLOR,            // Depth camera or rgb camera?
		NUI_IMAGE_RESOLUTION_640x480,    // Image resolution
		0,        // Image stream flags, e.g. near mode
		2,        // Number of frames to buffer
		NULL,   // Event handle
		&rgb_stream);
	is_opened = SUCCEEDED(hr);
	if (!is_opened) return hr;

	hr = sensor->NuiImageStreamOpen(
		NUI_IMAGE_TYPE_DEPTH,
		NUI_IMAGE_RESOLUTION_640x480,
		0,
		2,
		NULL,
		&depth_stream);
	is_opened = SUCCEEDED(hr);

	return hr;
}

void KinectManager::Update(unsigned int options){
	if (options & Update::Color){
		UpdateColor();
	}

	if (options & Update::Depth){
		UpdateDepth();
	}
}

void KinectManager::UpdateColor(){
	NUI_IMAGE_FRAME image_frame;
	NUI_LOCKED_RECT locked_rect;
	if (FAILED(sensor->NuiImageStreamGetNextFrame(rgb_stream, 0, &image_frame))) return;

	INuiFrameTexture * texture = image_frame.pFrameTexture;
	texture->LockRect(0, &locked_rect, NULL, 0);
	if (locked_rect.Pitch != 0){
		const unsigned char * curr = (const unsigned char*)locked_rect.pBits;
		unsigned char * rgbx_dest = rgbx_data;

		for (int y = 0; y < KINECT_CAPTURE_SIZE_Y; ++y){
			for (int x = 0; x < KINECT_CAPTURE_SIZE_X; ++x){
				int x_ = KINECT_CAPTURE_SIZE_X - x - 1;
				rgbx_dest[y*KINECT_CAPTURE_SIZE_X * 4 + x_ * 4 + 0] = curr[y*KINECT_CAPTURE_SIZE_X * 4 + x * 4 + 0];
				rgbx_dest[y*KINECT_CAPTURE_SIZE_X * 4 + x_ * 4 + 1] = curr[y*KINECT_CAPTURE_SIZE_X * 4 + x * 4 + 1];
				rgbx_dest[y*KINECT_CAPTURE_SIZE_X * 4 + x_ * 4 + 2] = curr[y*KINECT_CAPTURE_SIZE_X * 4 + x * 4 + 2];
				rgbx_dest[y*KINECT_CAPTURE_SIZE_X * 4 + x_ * 4 + 3] = curr[y*KINECT_CAPTURE_SIZE_X * 4 + x * 4 + 3];
			}
		}

	}
	texture->UnlockRect(0);
	sensor->NuiImageStreamReleaseFrame(rgb_stream, &image_frame);
}

unsigned char * KinectManager::GetColorRGBX(){
	return rgbx_data;
}

void KinectManager::UpdateDepth(){
	NUI_IMAGE_FRAME image_frame;
	NUI_LOCKED_RECT locked_rect;
	if (FAILED(sensor->NuiImageStreamGetNextFrame(depth_stream, 0, &image_frame))) return;

	INuiFrameTexture * texture = image_frame.pFrameTexture;
	texture->LockRect(0, &locked_rect, NULL, 0);

	const unsigned short* curr = (const unsigned short*)locked_rect.pBits;
	unsigned short * depth_dest = depth_data;

	for (int y = 0; y < KINECT_CAPTURE_SIZE_Y; ++y){
		for (int x = 0; x < KINECT_CAPTURE_SIZE_X; ++x){
			int x_ = KINECT_CAPTURE_SIZE_X - x - 1;
			depth_dest[y*KINECT_CAPTURE_SIZE_X + x_] = NuiDepthPixelToDepth(curr[y*KINECT_CAPTURE_SIZE_X + x]);
		}
	}

	texture->UnlockRect(0);
	sensor->NuiImageStreamReleaseFrame(depth_stream, &image_frame);
}

unsigned short * KinectManager::GetDepth(){
	return depth_data;
}


bool KinectManager::IsOpened(){
	return is_opened;
}

INuiSensor * KinectManager::GetSensor(){
	return sensor;
}

static KinectManager * singleton_manager;

KinectManager * KinectManager::GetKinectManager(){
	if (!singleton_manager){
		singleton_manager = new KinectManager();
	}

	return singleton_manager;
}