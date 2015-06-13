#include "ARCaptureGame.h"

#include "ghostsettings.h"
#if GHOST_INPUT == INPUT_KINECT
#include "KinectManager.h"
#endif

#if GHOST_INPUT == INPUT_KINECT2
#include "KinectManager.h"
#include "Kinect2Starter.h"
#include "Kinect.h"

#endif

#include "Map.h"
#include "MapPoint.h"
#include "KeyFrame.h"



namespace PTAMM{

	ARCaptureGame::ARCaptureGame():
		Game("AR Capture"),
		camera("Camera"),
		calibrate(false){

		static bool arc_menus_added = false;
		GVars3::GUI.RegisterCommand("ARC_Calibrate", GUICommandCallback, this);
		if (!arc_menus_added){
			//gui

			GVars3::GUI.ParseLine("GLWindow.AddMenu ARCMenu AR_Calibration");
			GVars3::GUI.ParseLine("ARCMenu.AddMenuButton Root \"Calibrate\" ARC_Calibrate Root");
			arc_menus_added = true;
		}
	}

	ARCaptureGame::~ARCaptureGame(){
		GVars3::GUI.UnRegisterCommand("ARC_Calibrate");
	}

	void ARCaptureGame::Reset(){

	}

	void ARCaptureGame::Init(){

	}

	void ARCaptureGame::Draw3D(const GLWindow2 &gl_window, Map &map, SE3<> camera_from_world){
		if (!calibrate) return;


		//dont actually draw
		int nCameraPoints = map.vpPoints.size();
		cv::Mat ptamm_camera_pts(4, nCameraPoints, CV_32F, cv::Scalar(0));
		cv::Mat kinect_camera_pts(4, nCameraPoints, CV_32F, cv::Scalar(0));

		cv::Mat ptamm_2d_pts(2, nCameraPoints, CV_32F, cv::Scalar(0));
		cv::Mat kinect_repro_pts(2, nCameraPoints, CV_32F, cv::Scalar(0));

#if GHOST_INPUT == INPUT_KINECT
		INuiSensor * sensor = KinectManager::GetKinectManager()->GetSensor();
		INuiCoordinateMapper * coordinate_mapper;
		HRESULT hr = sensor->NuiGetCoordinateMapper(&coordinate_mapper);
		if (FAILED(hr)) return;
		//KinectManager::GetKinectManager()->Update(Update::Color | Update::Depth);
		//unsigned short * depth = KinectManager::GetKinectManager()->GetDepth();

		NUI_IMAGE_FRAME depth_frame;
		sensor->NuiImageStreamGetNextFrame(KinectManager::GetKinectManager()->GetDepthStream(), 10, &depth_frame);

		BOOL near_mode;
		INuiFrameTexture * depth_frame_texture;
		sensor->NuiImageFrameGetDepthImagePixelFrameTexture(KinectManager::GetKinectManager()->GetDepthStream(), &depth_frame, &near_mode, &depth_frame_texture);

		NUI_DEPTH_IMAGE_POINT* depth_points;
		depth_points = new NUI_DEPTH_IMAGE_POINT[640 * 480];

		NUI_LOCKED_RECT locked_rect_depth;
		depth_frame_texture->LockRect(0, &locked_rect_depth, NULL, 0); 
		if (locked_rect_depth.Pitch != 0) { 
			INuiCoordinateMapper* pMapper; 
			
			sensor->NuiGetCoordinateMapper(&pMapper); 
			pMapper->MapColorFrameToDepthFrame(NUI_IMAGE_TYPE_COLOR, NUI_IMAGE_RESOLUTION_640x480, NUI_IMAGE_RESOLUTION_640x480, 640 * 480, (NUI_DEPTH_IMAGE_PIXEL*)locked_rect_depth.pBits, 640 * 480, depth_points); 

		}
		
		
#elif GHOST_INPUT == INPUT_KINECT2
		const float nRatio = (CAPTURE_SIZE_Y + 0.0) / CAPTURE_SIZE_Y_COLOR;
		const float nOffsetX = ((nRatio * CAPTURE_SIZE_X_COLOR) - CAPTURE_SIZE_X) / 2;
		ICoordinateMapper * coordinate_mapper = KINECT::getCoordinateMapper();
		USHORT * depth_raw = KINECT::GetDepth();
		DepthSpacePoint * dsp = new DepthSpacePoint[CAPTURE_SIZE_X_COLOR * CAPTURE_SIZE_Y_COLOR];

		coordinate_mapper->MapColorFrameToDepthSpace(CAPTURE_SIZE_X_DEPTH * CAPTURE_SIZE_Y_DEPTH, depth_raw,
			CAPTURE_SIZE_X_COLOR * CAPTURE_SIZE_Y_COLOR, dsp);
#endif

		
		for (int i = 0; i < map.vpPoints.size(); ++i){
			TooN::Vector<3,double> camera_pt = camera_from_world * map.vpPoints[i]->v3WorldPos;

			TooN::Vector<2, double> camplane_pt;
			camplane_pt[0] = camera_pt[0] / camera_pt[2];
			camplane_pt[1] = camera_pt[1] / camera_pt[2];

			TooN::Vector<2, double> screen_pt = camera.Project(camplane_pt);


#if GHOST_INPUT == INPUT_KINECT
			if (screen_pt[0] < 0 || screen_pt[0] >= KINECT_CAPTURE_SIZE_X || screen_pt[1] < 0 || screen_pt[1] >= KINECT_CAPTURE_SIZE_Y) continue;

			int screen_x = screen_pt[0];
			int screen_y = screen_pt[1];

			NUI_DEPTH_IMAGE_POINT depth_pt = depth_points[screen_y * KINECT_CAPTURE_SIZE_X + screen_x];

			Vector4 skeleton_pt;
			coordinate_mapper->MapDepthPointToSkeletonPoint(NUI_IMAGE_RESOLUTION_640x480, &depth_pt, &skeleton_pt);
			NUI_COLOR_IMAGE_POINT color_pt;
			coordinate_mapper->MapSkeletonPointToColorPoint(&skeleton_pt, NUI_IMAGE_TYPE_COLOR, NUI_IMAGE_RESOLUTION_640x480, &color_pt);

			if (color_pt.x < 0.1 || color_pt.y < 0.1) continue;

			kinect_camera_pts.ptr<float>(0)[i] = skeleton_pt.x;
			kinect_camera_pts.ptr<float>(1)[i] = skeleton_pt.y;
			kinect_camera_pts.ptr<float>(2)[i] = skeleton_pt.z;
			kinect_camera_pts.ptr<float>(3)[i] = skeleton_pt.w;
			
			kinect_repro_pts.ptr<float>(0)[i] = color_pt.x;
			kinect_repro_pts.ptr<float>(1)[i] = color_pt.y;

#elif GHOST_INPUT == INPUT_KINECT2
			if (screen_pt[0] < 0 || screen_pt[0] >= KINECT_CAPTURE_SIZE_X || screen_pt[1] < 0 || screen_pt[1] >= KINECT_CAPTURE_SIZE_Y) continue;

			int nu_y = screen_pt[1] / nRatio;
			int nu_x = (screen_pt[0] + nOffsetX) / nRatio;
			//int nu_x = screen_pt[0];
			//int nu_y = screen_pt[1];

			int depth_idx = (nu_y * CAPTURE_SIZE_X_COLOR + (CAPTURE_SIZE_X_COLOR - nu_x-1));
			if (depth_idx < 0 || depth_idx >= CAPTURE_SIZE_X_COLOR*CAPTURE_SIZE_Y_COLOR) continue;
			DepthSpacePoint kinect_dsp = dsp[depth_idx];

			UINT16 depth = depth_raw[(int)(kinect_dsp.Y*CAPTURE_SIZE_X_DEPTH + kinect_dsp.X)];

			CameraSpacePoint csp;
			coordinate_mapper->MapDepthPointToCameraSpace(kinect_dsp, depth, &csp);

			if (isinf(csp.X) ||isinf(csp.Y)||isinf(csp.Z)|| screen_pt[0]<0.001 || screen_pt[1] < 0.001) continue;

			kinect_camera_pts.ptr<float>(0)[i] = csp.X;
			kinect_camera_pts.ptr<float>(1)[i] = csp.Y;
			kinect_camera_pts.ptr<float>(2)[i] = csp.Z;
			kinect_camera_pts.ptr<float>(3)[i] = 1;
#endif

			ptamm_camera_pts.ptr<float>(0)[i] = camera_pt[0];
			ptamm_camera_pts.ptr<float>(1)[i] = camera_pt[1];
			ptamm_camera_pts.ptr<float>(2)[i] = camera_pt[2];
			ptamm_camera_pts.ptr<float>(3)[i] = 1;
			ptamm_2d_pts.ptr<float>(0)[i] = screen_pt[0];
			ptamm_2d_pts.ptr<float>(1)[i] = screen_pt[1];
		}

		PTAMM_to_kinect = kinect_camera_pts * ptamm_camera_pts.t() * (ptamm_camera_pts * ptamm_camera_pts.t()).inv();

		cv::FileStorage fs;
		fs.open("PTAMM_to_kinect.yml", cv::FileStorage::WRITE);
		fs << "PTAMM_to_kinect" << PTAMM_to_kinect;

#if GHOST_INPUT == INPUT_KINECT
		cv::Mat im = cv::Mat(KINECT_CAPTURE_SIZE_Y, KINECT_CAPTURE_SIZE_X, CV_8UC4, KinectManager::GetKinectManager()->GetColorRGBX()).clone();
		for (int y = 0; y < KINECT_CAPTURE_SIZE_Y; ++y){
			for (int x = 0; x < KINECT_CAPTURE_SIZE_X; ++x){
				im.ptr<cv::Vec4b>(y)[x](3) = 0xff;
			}
		}

		for(int i=0;i<nCameraPoints;++i){
			cv::circle(im, cv::Point(ptamm_2d_pts.ptr<float>(0)[i], ptamm_2d_pts.ptr<float>(1)[i]), 2, cv::Scalar(0xff,0,0,0xff), -1);
			cv::circle(im, cv::Point(kinect_repro_pts.ptr<float>(0)[i], kinect_repro_pts.ptr<float>(1)[i]), 4, cv::Scalar(0,0,0xff,0xff), 1);
		}

		cv::imwrite("calib-results.png", im);

		delete [] depth_points;

#elif GHOST_INPUT == INPUT_KINECT2

		std::vector<CameraSpacePoint> kinect_camera_pts_k2(nCameraPoints);
		std::vector<ColorSpacePoint> kinect_repro_pts_k2(nCameraPoints);

		for (int i = 0; i < nCameraPoints; ++i){
			kinect_camera_pts_k2[i].X = kinect_camera_pts.ptr<float>(0)[i];
			kinect_camera_pts_k2[i].Y = kinect_camera_pts.ptr<float>(1)[i];
			kinect_camera_pts_k2[i].Z = kinect_camera_pts.ptr<float>(2)[i];
		}

		HRESULT hr = coordinate_mapper->MapCameraPointsToColorSpace(nCameraPoints, kinect_camera_pts_k2.data(), nCameraPoints, kinect_repro_pts_k2.data());

		for (int i = 0; i<nCameraPoints; ++i){

			//skip if invalid
			if (ptamm_camera_pts.ptr<float>(2)[i] == 0) continue;

			kinect_repro_pts.ptr<float>(0)[i] = nRatio * kinect_repro_pts_k2[i].X - nOffsetX;
			kinect_repro_pts.ptr<float>(1)[i] = nRatio * kinect_repro_pts_k2[i].Y;
		}

		delete[] dsp;
#endif

		double repro_SSD = 0;
		int nValid = 0;
		for (int i = 0; i < map.vpPoints.size(); ++i){
			if (kinect_repro_pts.ptr<float>(0)[i] != 0 && kinect_repro_pts.ptr<float>(1)[i] != 0
				&& ptamm_2d_pts.ptr<float>(0)[i] != 0 && ptamm_2d_pts.ptr<float>(1)[i] != 0){
				repro_SSD += (kinect_repro_pts.ptr<float>(0)[i] - ptamm_2d_pts.ptr<float>(0)[i])*(kinect_repro_pts.ptr<float>(0)[i] - ptamm_2d_pts.ptr<float>(0)[i]);
				repro_SSD += (kinect_repro_pts.ptr<float>(1)[i] - ptamm_2d_pts.ptr<float>(1)[i])*(kinect_repro_pts.ptr<float>(1)[i] - ptamm_2d_pts.ptr<float>(1)[i]);
				++nValid;
			}
		}

		repro_SSD /= nValid;

		fs << "reprojected_SSD" << repro_SSD;

		fs << "PTAMM camera pts" << ptamm_camera_pts
			<< "Kinect camera pts" << kinect_camera_pts
			<< "PTAMM screen pts" << ptamm_2d_pts
			<< "Kinect reprojected pts" << kinect_repro_pts;

		fs.release();


		calibrate = false;
	}

	void ARCaptureGame::Draw2D(const GLWindow2 gl_window, Map &map){

	}

	std::string ARCaptureGame::Save(std::string map_path){

		cv::FileStorage fs;

		if (!PTAMM_to_kinect.empty()){
			fs.open(map_path + "PTAMM_to_kinect.yml", cv::FileStorage::WRITE);
			fs << "PTAMM_to_kinect" << PTAMM_to_kinect;
			fs.release();

		}
		return "dang";
	}
	void ARCaptureGame::Load(std::string map_path){

	}

	void ARCaptureGame::HandleClick(Vector<2> vid_coords, Vector<2> UFB, Vector<3> ray_direction,
		Vector<2> plane, int button){

	}
	void ARCaptureGame::HandleKeyPress(std::string key){

	}
	void ARCaptureGame::Advance(){

	}

	void ARCaptureGame::DoCalibrate(){
		calibrate = true;
	}

	void ARCaptureGame::GUICommandCallback(void *ptr, string command, string params){
		ARCaptureGame * g = static_cast<ARCaptureGame*>(ptr);
		if (command == "ARC_Calibrate"){
			g->DoCalibrate();
		}
	}
}