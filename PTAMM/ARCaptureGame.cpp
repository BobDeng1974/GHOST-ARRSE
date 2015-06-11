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
		cv::Mat ptamm_camera_pts(4, map.vpPoints.size(), CV_32F, cv::Scalar(0));
		cv::Mat kinect_camera_pts(4, map.vpPoints.size(), CV_32F, cv::Scalar(0));
		int nCameraPoints = ptamm_camera_pts.cols;
#if GHOST_INPUT == INPUT_KINECT
		const float nRatio = (CAPTURE_SIZE_Y + 0.0) / CAPTURE_SIZE_Y_COLOR;
		const float nOffsetX = ((nRatio * CAPTURE_SIZE_X_COLOR) - CAPTURE_SIZE_X) / 2;
		INuiSensor * sensor = KinectManager::GetKinectManager()->GetSensor();
		INuiCoordinateMapper * coordinate_mapper;
		HRESULT hr = sensor->NuiGetCoordinateMapper(&coordinate_mapper);
		if (FAILED(hr)) return;

		KinectManager::GetKinectManager()->Update(Update::Depth);
		unsigned short * depth = KinectManager::GetKinectManager()->GetDepth();

		cv::Mat ptamm_camera_pts(4, map.vpPoints.size(), CV_32F, cv::Scalar(0));
		cv::Mat kinect_camera_pts(4, map.vpPoints.size(), CV_32F, cv::Scalar(0));

		cv::Mat ptamm_2d_pts(2, map.vpPoints.size(), CV_32F, cv::Scalar(0));
		cv::Mat kinect_repro_pts(2, map.vpPoints.size(), CV_32F, cv::Scalar(0));

		
		for (int i = 0; i < map.vpPoints.size(); ++i){
			TooN::Vector<3,double> camera_pt = camera_from_world * map.vpPoints[i]->v3WorldPos;

			TooN::Vector<2, double> camplane_pt;
			camplane_pt[0] = camera_pt[0] / camera_pt[2];
			camplane_pt[1] = camera_pt[1] / camera_pt[2];

			TooN::Vector<2, double> screen_pt = camera.Project(camplane_pt);

			if (screen_pt[0] < 0 || screen_pt[0] >= KINECT_CAPTURE_SIZE_X || screen_pt[1] < 0 || screen_pt[1] >= KINECT_CAPTURE_SIZE_Y) continue;


			NUI_DEPTH_IMAGE_POINT depth_pt;
			depth_pt.x = screen_pt[0];
			depth_pt.y = screen_pt[1];
			depth_pt.depth = depth[depth_pt.y * KINECT_CAPTURE_SIZE_X + depth_pt.x];

			Vector4 skeleton_pt;
			coordinate_mapper->MapDepthPointToSkeletonPoint(NUI_IMAGE_RESOLUTION_640x480, &depth_pt, &skeleton_pt);
			NUI_COLOR_IMAGE_POINT color_pt;
			coordinate_mapper->MapSkeletonPointToColorPoint(&skeleton_pt, NUI_IMAGE_TYPE_COLOR, NUI_IMAGE_RESOLUTION_640x480, &color_pt);

			if (color_pt.x < 0.1 || color_pt.y < 0.1) continue;

			kinect_camera_pts.ptr<float>(0)[i] = skeleton_pt.x;
			kinect_camera_pts.ptr<float>(1)[i] = skeleton_pt.y;
			kinect_camera_pts.ptr<float>(2)[i] = skeleton_pt.z;
			kinect_camera_pts.ptr<float>(3)[i] = skeleton_pt.w;
			ptamm_camera_pts.ptr<float>(0)[i] = camera_pt[0];
			ptamm_camera_pts.ptr<float>(1)[i] = camera_pt[1];
			ptamm_camera_pts.ptr<float>(2)[i] = camera_pt[2];
			ptamm_camera_pts.ptr<float>(3)[i] = 1;
			ptamm_2d_pts.ptr<float>(0)[i] = screen_pt[0];
			ptamm_2d_pts.ptr<float>(1)[i] = screen_pt[1];
			kinect_repro_pts.ptr<float>(0)[i] = color_pt.x;
			kinect_repro_pts.ptr<float>(1)[i] = color_pt.y;
		}

		PTAMM_to_kinect = kinect_camera_pts * ptamm_camera_pts.t() * (ptamm_camera_pts * ptamm_camera_pts.t()).inv();

		cv::FileStorage fs;
		fs.open("PTAMM_to_kinect.yml", cv::FileStorage::WRITE);
		fs << "PTAMM_to_kinect" << PTAMM_to_kinect;


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





#endif

#if GHOST_INPUT == INPUT_KINECT2
		ICoordinateMapper * coordinate_mapper = KINECT::getCoordinateMapper();
		USHORT * depth_raw = KINECT::GetDepth();
		DepthSpacePoint * dsp = new DepthSpacePoint[CAPTURE_SIZE_X_COLOR * CAPTURE_SIZE_Y_COLOR];

		coordinate_mapper->MapColorFrameToDepthSpace(CAPTURE_SIZE_X_DEPTH * CAPTURE_SIZE_Y_DEPTH, depth_raw,
			CAPTURE_SIZE_X_COLOR * CAPTURE_SIZE_Y_COLOR, dsp);

		cv::Mat ptamm_2d(2, nCameraPoints, CV_32F, cv::Scalar(0));

		for (int i = 0; i < map.vpPoints.size(); ++i){
			TooN::Vector<3, double> camera_pt = camera_from_world * map.vpPoints[i]->v3WorldPos;

			TooN::Vector<2, double> camplane_pt;
			camplane_pt[0] = camera_pt[0] / camera_pt[2];
			camplane_pt[1] = camera_pt[1] / camera_pt[2];

			TooN::Vector<2, double> screen_pt = camera.Project(camplane_pt);

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
			ptamm_camera_pts.ptr<float>(0)[i] = camera_pt[0];
			ptamm_camera_pts.ptr<float>(1)[i] = camera_pt[1];
			ptamm_camera_pts.ptr<float>(2)[i] = camera_pt[2];
			ptamm_camera_pts.ptr<float>(3)[i] = 1;
			ptamm_2d.ptr<float>(0)[i] = screen_pt[0];
			ptamm_2d.ptr<float>(1)[i] = screen_pt[1];
		}

		cv::Mat PTAMM_to_kinect = kinect_camera_pts * ptamm_camera_pts.t() * (ptamm_camera_pts * ptamm_camera_pts.t()).inv();

		cv::FileStorage fs;
		fs.open("PTAMM_to_kinect.yml", cv::FileStorage::WRITE);
		fs << "PTAMM_to_kinect" << PTAMM_to_kinect;

		cv::Mat kinected_PTAMM = PTAMM_to_kinect * ptamm_camera_pts;
		cv::Mat diff;
		cv::subtract(kinect_camera_pts, ptamm_camera_pts, diff);

		fs << "sumdiff_kinect_ptamm" << cv::sum(diff);

		cv::subtract(kinect_camera_pts, kinected_PTAMM, diff);

		fs << "sumdiff_calculated" << cv::sum(diff);

		std::vector<CameraSpacePoint> vCameraPoints(nCameraPoints);
		std::vector<ColorSpacePoint> vColorPoints(nCameraPoints);

		for (int i = 0; i < nCameraPoints; ++i){
			vCameraPoints[i].X = kinect_camera_pts.ptr<float>(0)[i];
			vCameraPoints[i].Y = kinect_camera_pts.ptr<float>(1)[i];
			vCameraPoints[i].Z = kinect_camera_pts.ptr<float>(2)[i];
		}

		HRESULT hr = coordinate_mapper->MapCameraPointsToColorSpace(nCameraPoints, vCameraPoints.data(), nCameraPoints, vColorPoints.data());


		cv::Mat mColorPoints(2, nCameraPoints, CV_32F, cv::Scalar(0));
		for (int i = 0; i<nCameraPoints; ++i){
			mColorPoints.ptr<float>(0)[i] = nRatio * vColorPoints[i].X - nOffsetX;
			mColorPoints.ptr<float>(1)[i] = nRatio * vColorPoints[i].Y;
		}

		double repro_SSD = 0;
		int nValid = 0;
		for (int i = 0; i < nCameraPoints; ++i){
			if (mColorPoints.ptr<float>(0)[i] != 0 && mColorPoints.ptr<float>(1)[i] != 0
				&& ptamm_2d.ptr<float>(0)[i] != 0 && ptamm_2d.ptr<float>(1)[i] != 0){
				repro_SSD += (mColorPoints.ptr<float>(0)[i] - ptamm_2d.ptr<float>(0)[i])*(mColorPoints.ptr<float>(0)[i] - ptamm_2d.ptr<float>(0)[i]);
				repro_SSD += (mColorPoints.ptr<float>(1)[i] - ptamm_2d.ptr<float>(1)[i])*(mColorPoints.ptr<float>(1)[i] - ptamm_2d.ptr<float>(1)[i]);
				++nValid;
			}
		}

		repro_SSD /= nValid;

		fs << "reprojected_SSD" << repro_SSD;

		fs << "mColorPoints" << mColorPoints
			<< "ptamm2d" << ptamm_2d;

		fs.release();

		delete[] dsp;
#endif
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