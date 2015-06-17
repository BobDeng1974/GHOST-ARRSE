#pragma once
#include "Game.h"

#include <Windows.h>

#include <gvars3\instances.h>

#include <opencv2\opencv.hpp>
#include <gh_common.h>
#include <gh_search.h>
#include <cv_skeleton.h>
#include <ReconsVoxel.h>
#include <gl\glew.h>
#include <fbolib.h>
#include "ATANCamera.h"

namespace PTAMM{
	class ARReenactmentGame :public Game{
	public:
		ARReenactmentGame();
		~ARReenactmentGame();

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
		
		void section_start_frame();
		void reset_cfw();
		void save_cfw();

		int current_section;
		std::vector<std::vector<int>> section_frames;
		int current_frame_within_section;

		//Reenactment stuff
		BodyPartDefinitionVector bodypart_definitions;
		std::vector<FrameDataProcessed> frame_datas;

		std::vector<Cylinder> bodypart_cylinders;
		std::vector<VoxelMatrix> bodypart_voxels;
		std::vector<cv::Mat> bodypart_TSDF_array;
		std::vector<cv::Mat> bodypart_weight_array;
		
		float voxel_size;
		float tsdf_offset;

		std::vector<std::vector<float>> triangle_vertices;
		std::vector<std::vector<unsigned int>> triangle_indices;
		std::vector<std::vector<unsigned char>> triangle_colors;

		BodypartFrameCluster bodypart_frame_cluster;
		std::vector<std::vector<cv::Vec3f>> bodypart_precalculated_rotation_vectors;
		std::vector<SkeletonNodeHardMap> frame_snhmaps;

		//cv::Mat model_center, model_center_inv;

		bool debug_draw_skeleton, 
			debug_shape_cylinders,
			debug_show_volumes,
			debug_inspect_texture_map;

		FBO fbo1;

		cv::Mat opengl_projection, camera_matrix_current;
		cv::Mat PTAMM_to_kinect;

		ATANCamera temp_cam;

		cv::Vec3b bg_color;

		int anim_frame;
		SYSTEMTIME prev_time;
		unsigned int elapsed;

		GVars3::gvar3<int> pause;

		cv::Mat camera_from_world_capture; //this should be different per frame, based on the capturing stage; for now, we'll set it to the first frame
		float secret_offset; //sshhh!

		std::string PTAMM_map_path;
	};
}