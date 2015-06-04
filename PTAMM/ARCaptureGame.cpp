#include "ARCaptureGame.h"

#include "ghostsettings.h"
#if GHOST_INPUT == INPUT_KINECT
#include "KinectManager.h"
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
#if GHOST_INPUT == INPUT_KINECT
		INuiSensor * sensor = KinectManager::GetKinectManager()->GetSensor();
		INuiCoordinateMapper * coordinate_mapper;
		HRESULT hr = sensor->NuiGetCoordinateMapper(&coordinate_mapper);
		if (FAILED(hr)) return;

		KinectManager::GetKinectManager()->Update(Update::Depth);
		unsigned short * depth = KinectManager::GetKinectManager()->GetDepth();

		cv::Mat ptamm_camera_pts(4, map.vpPoints.size(), CV_32F);
		cv::Mat kinect_camera_pts(4, map.vpPoints.size(), CV_32F);

		for (int i = 0; i < map.vpPoints.size(); ++i){
			TooN::Vector<3,double> camera_pt = camera_from_world * map.vpPoints[i]->v3WorldPos;
			ptamm_camera_pts.ptr<float>(0)[i] = camera_pt[0];
			ptamm_camera_pts.ptr<float>(1)[i] = camera_pt[1];
			ptamm_camera_pts.ptr<float>(2)[i] = camera_pt[2];
			ptamm_camera_pts.ptr<float>(3)[i] = 1;

			TooN::Vector<2, double> camplane_pt;
			camplane_pt[0] = camera_pt[0] / camera_pt[2];
			camplane_pt[1] = camera_pt[1] / camera_pt[2];

			TooN::Vector<2, double> screen_pt = camera.Project(camplane_pt);

			NUI_DEPTH_IMAGE_POINT depth_pt;
			depth_pt.x = screen_pt[0];
			depth_pt.y = screen_pt[1];
			depth_pt.depth = depth[depth_pt.y * KINECT_CAPTURE_SIZE_X + depth_pt.x];

			Vector4 skeleton_pt;
			coordinate_mapper->MapDepthPointToSkeletonPoint(NUI_IMAGE_RESOLUTION_640x480, &depth_pt, &skeleton_pt);

			kinect_camera_pts.ptr<float>(0)[i] = skeleton_pt.x;
			kinect_camera_pts.ptr<float>(1)[i] = skeleton_pt.y;
			kinect_camera_pts.ptr<float>(2)[i] = skeleton_pt.z;
			kinect_camera_pts.ptr<float>(3)[i] = skeleton_pt.w;
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

		fs.release();
#endif
	}

	void ARCaptureGame::Draw2D(const GLWindow2 gl_window, Map &map){

	}

	std::string ARCaptureGame::Save(std::string map_path){
		return "";
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