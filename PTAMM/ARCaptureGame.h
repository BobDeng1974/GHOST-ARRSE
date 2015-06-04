#pragma once
#include "Game.h"

#include <Windows.h>

#include <gvars3\instances.h>

#include <opencv2\opencv.hpp>
#include <gh_common.h>
#include <cv_skeleton.h>
#include <ReconsVoxel.h>
#include <gl\glew.h>
#include "ATANCamera.h"

namespace PTAMM{
	class ARCaptureGame :public Game{
	public:
		ARCaptureGame();
		~ARCaptureGame();

		void Reset();
		void Init();

		void Draw3D(const GLWindow2 &gl_window, Map &map, SE3<> camera_from_world);
		void Draw2D(const GLWindow2 gl_window, Map &map);

		std::string Save(std::string map_path);
		void Load(std::string map_path);

		void HandleClick(Vector<2> vid_coords, Vector<2> UFB, Vector<3> ray_direction,
			Vector<2> plane, int button);
		void HandleKeyPress(std::string key);
		void Advance();


	private:
		static void GUICommandCallback(void *ptr, string command, string params);
		void DoCalibrate();

		ATANCamera camera;
		bool calibrate;
	};
}