#include "KinectManager.h"

#if GHOST_INPUT == INPUT_KINECT

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

#elif GHOST_INPUT == INPUT_KINECT2
#include "KinectManager.h"
#include <opencv2\opencv.hpp>

#ifndef isinf
bool isinf(double x) { return !_isnan(x) && _isnan(x - x); }
#endif 

//#if !INIT_KINECT
//#endif

//#include "PTAM2Kinect.h"

#define JFS(s, j) mat_to_vec3(s.points.col(j))
#define JFM(m, j) mat_to_vec3(m.col(j))


namespace KINECT{
	cv::Mat loadCameraParameters(){
		cv::FileStorage fs;
		fs.open("kinect2camera.yml", cv::FileStorage::READ);
		cv::Mat camparam;
		fs["kinect2camera"] >> camparam;
		return camparam;
	}
}


namespace KINECT{

	bool bInit = false;
	bool bSkeletonIsGood = false;
	bool bAutoUpdate = false;

	const float nRatio = (CAPTURE_SIZE_Y + 0.0) / CAPTURE_SIZE_Y_COLOR;
	const float nOffsetX = ((nRatio * CAPTURE_SIZE_X_COLOR) - CAPTURE_SIZE_X) / 2;

	bool doCalib(){
#if GHOST_INPUT == INPUT_KINECT2
		return true;
#else
		return false;
#endif
	}

	bool init(){
		if (bInit) return true;
		bInit = SUCCEEDED(InitializeDefaultSensor());
		if (bInit){
			InitKinect2Starter();
		}

		return bInit;
	}

	bool release(){
		DestroyKinect2Starter();
		return true;
	}

	void updateFrames(){
		UpdateColor();
		UpdateDepth();
		UpdateBody();
		UpdateBodyFrameIndex();
	}

	cv::Mat scaleAndCrop(cv::Mat& m){

		float tempWidth = nRatio * getColorWidth();
		cv::Mat m_resized;
		cv::resize(m, m_resized, cv::Size(tempWidth, CAPTURE_SIZE_Y));
		return m_resized(cv::Rect(nOffsetX, 0, CAPTURE_SIZE_X, CAPTURE_SIZE_Y)).clone();
	}

	template<typename T>
	cv::Mat_<T> scaleAndCropDiscrete(cv::Mat_<T>& m){
		float tempWidth = nRatio * getColorWidth();
		cv::Mat_<T> m_resized(CAPTURE_SIZE_Y, tempWidth);

		float nRatioR = 1. / nRatio;
		for (int r = 0; r<CAPTURE_SIZE_Y; ++r){
			for (int c = 0; c<tempWidth; ++c){
				m_resized(r, c) = m(r*nRatioR, c*nRatioR);
			}
		}

		return m_resized(cv::Rect(nOffsetX, 0, CAPTURE_SIZE_X, CAPTURE_SIZE_Y)).clone();
	}

	cv::Mat getColorFrame(){
		if (bAutoUpdate) UpdateColor();

		if (getColorHeight() == 0 || getColorWidth() == 0) return cv::Mat();
		cv::Mat colorFrame_ = cv::Mat(getColorHeight(), getColorWidth(), CV_8UC4, GetColorRGBX()).clone();
		cv::Mat colorFrame = scaleAndCrop(colorFrame_);

		return colorFrame;
	}


	cv::Mat getDepthFrame(){
		if (bAutoUpdate) UpdateDepth();

		if (getDepthHeight() == 0 || getDepthWidth() == 0 || getColorHeight() == 0 || getColorWidth() == 0) return cv::Mat();
		cv::Mat depthFrame_ = cv::Mat(getColorHeight(), getColorWidth(), CV_16U, GetDepthMappedToColor()).clone();
		cv::Mat depthFrame = scaleAndCrop(depthFrame_);
		return depthFrame;
	}
	

	cv::Mat getUserColorFrame(){
		return cv::Mat();
	}

	bool skeletonIsGood(){
		return getSkeletonIsGood();
	}


	bool checkTracked(int state){
		//TODO
		return true;
	}


	void saveParams(std::string filename){
		//nothing to do here
	}

	void loadParams(std::string filename){
		//nothing to do here
	}

	//delete later

	void getKinectData_depth_raw(DepthXY * depthPoints){
		if (!init()) {
			std::cerr << "Error! not initialized!\n";
			return;
		}

		if (getDepthHeight() == 0 || getDepthWidth() == 0 || getColorHeight() == 0 || getColorWidth() == 0) return;

		cv::Mat depthFrame_ = cv::Mat(getColorHeight(), getColorWidth(), CV_16U, GetDepthMappedToColor());
		cv::Mat depthFrame = scaleAndCrop(depthFrame_);

		int * pDepthXMappedToColor = KINECT::GetDepthXMappedToColor();
		int * pDepthYMappedToColor = KINECT::GetDepthYMappedToColor();

		cv::Mat_<cv::Point> mpPoints_(getColorHeight(), getColorWidth());
		for (int y = 0; y<getColorHeight(); ++y){
			for (int x = 0; x<getColorWidth(); ++x){
				int depthX, depthY;
				depthX = pDepthXMappedToColor[x + y*getColorWidth()];
				depthY = pDepthYMappedToColor[x + y*getColorWidth()];
				mpPoints_.ptr<cv::Point>(y)[x] = cv::Point(depthX, depthY);
			}
		}

		cv::Mat mpPoints = scaleAndCropDiscrete(mpPoints_);

		for (int y = 0; y<CAPTURE_SIZE_Y; ++y){
			for (int x = 0; x<CAPTURE_SIZE_X; ++x){
				depthPoints[x + CAPTURE_SIZE_X*y].depth = depthFrame.ptr<unsigned short>()[x + CAPTURE_SIZE_X*y];
				depthPoints[x + CAPTURE_SIZE_X*y].x = mpPoints.ptr<cv::Point>(y)[x].x;
				depthPoints[x + CAPTURE_SIZE_X*y].y = mpPoints.ptr<cv::Point>(y)[x].y;
			}
		}
	}

	cv::Vec3f mapDepthToSkeletonPoint(DepthXY d){
		cv::Vec3f ret;

		float dx = d.x;
		float dy = d.y;
		long dz = d.depth;

		mapDepthToSkeleton(&dx, &dy, &dz, &ret(0), &ret(1), &ret(2));

		return ret;
	}


	//takes 4xN Mat of camera points and uses Kinect function to map it to 2xN Mat of color space points
	cv::Mat mapCameraPointsToColorPoints(cv::Mat cameraPoints){
		ICoordinateMapper * coordinateMapper = getCoordinateMapper();
		int nCameraPoints = cameraPoints.cols;
		std::vector<CameraSpacePoint> vCameraPoints(nCameraPoints);
		std::vector<ColorSpacePoint> vColorPoints(nCameraPoints);

		for (int i = 0; i<nCameraPoints; ++i){
			vCameraPoints[i].X = cameraPoints.ptr<float>(0)[i];
			vCameraPoints[i].Y = cameraPoints.ptr<float>(1)[i];
			vCameraPoints[i].Z = cameraPoints.ptr<float>(2)[i];
		}

		HRESULT hr = coordinateMapper->MapCameraPointsToColorSpace(nCameraPoints, vCameraPoints.data(), nCameraPoints, vColorPoints.data());

		cv::Mat mColorPoints(2, nCameraPoints, CV_32F);
		for (int i = 0; i<nCameraPoints; ++i){
			mColorPoints.ptr<float>(0)[i] = nRatio * vColorPoints[i].X - nOffsetX;
			mColorPoints.ptr<float>(1)[i] = nRatio * vColorPoints[i].Y;
		}

		return mColorPoints;
	}

	cv::Mat makeRays(cv::Mat pts2d){
		ICoordinateMapper * coordinateMapper = getCoordinateMapper();
		int n2DPoints = pts2d.cols;

		std::vector<DepthSpacePoint> vDepthPoints(n2DPoints);
		for (int i = 0; i<n2DPoints; ++i){
			vDepthPoints[i].X = pts2d.ptr<float>(0)[i];
			vDepthPoints[i].Y = pts2d.ptr<float>(1)[i];
		}

		std::vector<UINT16> vDepthValues(n2DPoints, 1000);
		std::vector<CameraSpacePoint> vCameraPoints(n2DPoints);

		HRESULT hr = coordinateMapper->MapDepthPointsToCameraSpace(n2DPoints, vDepthPoints.data(), n2DPoints, vDepthValues.data(), n2DPoints, vCameraPoints.data());

		cv::Mat mCameraPoints(4, n2DPoints, CV_32F);
		for (int i = 0; i<n2DPoints; ++i){
			mCameraPoints.ptr<float>(0)[i] = vCameraPoints[i].X;
			mCameraPoints.ptr<float>(1)[i] = vCameraPoints[i].Y;
			mCameraPoints.ptr<float>(2)[i] = 1;
			mCameraPoints.ptr<float>(3)[i] = 1;
		}

		return mCameraPoints;
	}

	cv::Mat calculateCameraParameters(){
		ICoordinateMapper* coordinateMapper = getCoordinateMapper();

		//try to calculate the intrinsic camera parameters
		//we make a cube of camera space points:
		int cubeSide = 2;
		float spacing = 0.1;
		int stride = cubeSide / spacing;

		std::vector<CameraSpacePoint> cameraPoints;
		for (int w = -stride; w < stride; ++w){
			for (int h = -stride; h < stride; ++h){
				for (int d = stride; d < stride * 2; ++d){
					CameraSpacePoint csp;
					csp.X = w*spacing;
					csp.Y = h*spacing;
					csp.Z = d*spacing;
					cameraPoints.push_back(csp);
				}
			}
		}
		int count = cameraPoints.size();
		std::vector<ColorSpacePoint> colorPoints(count);

		HRESULT hr = coordinateMapper->MapCameraPointsToColorSpace(count, cameraPoints.data(),
			count, colorPoints.data());

		if (!SUCCEEDED(hr)){
			std::cerr << "error with coordinate mapper...\n";
			throw;
		}

		for (int i = 0; i < count; ++i){
			colorPoints[i].X = nRatio*colorPoints[i].X - nOffsetX;
			colorPoints[i].Y = nRatio*colorPoints[i].Y;
		}

		cv::Mat mCameraPoints(4, count, CV_32F);
		cv::Mat mColorPoints(4, count, CV_32F);

		for (int i = 0; i < count; ++i){
			if (!isinf(colorPoints[i].X) && !isinf(colorPoints[i].Y)){
				mCameraPoints.ptr<float>(0)[i] = cameraPoints[i].X;
				mCameraPoints.ptr<float>(1)[i] = cameraPoints[i].Y;
				mCameraPoints.ptr<float>(2)[i] = cameraPoints[i].Z;
				mCameraPoints.ptr<float>(3)[i] = 1;

				float W = cameraPoints[i].Z;

				mColorPoints.ptr<float>(0)[i] = colorPoints[i].X*W;
				mColorPoints.ptr<float>(1)[i] = colorPoints[i].Y*W;
				mColorPoints.ptr<float>(2)[i] = W;
				mColorPoints.ptr<float>(3)[i] = 1;
			}
			else{

				mCameraPoints.ptr<float>(0)[i] = 0;
				mCameraPoints.ptr<float>(1)[i] = 0;
				mCameraPoints.ptr<float>(2)[i] = 0;
				mCameraPoints.ptr<float>(3)[i] = 0;

				mColorPoints.ptr<float>(0)[i] = 0;
				mColorPoints.ptr<float>(1)[i] = 0;
				mColorPoints.ptr<float>(2)[i] = 0;
				mColorPoints.ptr<float>(3)[i] = 0;
			}
		}

		return mColorPoints*mCameraPoints.t()*((mCameraPoints*mCameraPoints.t()).inv());
	}

	std::pair<int, int> facingHelper(int s){
		if (s == 1) //shoulders
		{
			return std::pair<int, int>(JointType_ShoulderLeft, JointType_ShoulderRight);
		}
		else if (s == 2){
			return std::pair<int, int>(JointType_HipLeft, JointType_HipRight);
		}
		else{
			std::cerr << "wrong value passed into facingHelper\n";
			throw std::exception();
		}
	}
}

#endif