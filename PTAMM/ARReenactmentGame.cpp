#include "ARReenactmentGame.h"

#include <algorithm>

#include <gvars3\instances.h>
#include <cvd\gl_helpers.h>

#include <glcv.h>
#include <cv_pointmat_common.h>
#include <gh_search.h>
#include <gh_render.h>

#include "cylinder.h"

#include <fstream>

#define MAX_SEARCH 16
#define FILL_LIMIT 16
#define FILL_NEIGHBORHOOD 5
#define FRAMERATE 15


namespace PTAMM{

#ifdef DEBUG_OUT
	std::string debug_print_dir;

	std::string generate_debug_print_dir(){
		std::time_t time = std::time(nullptr);
		std::tm ltime;
		localtime_s(&ltime, &time);
		std::stringstream ss;

		ss << "debug-PTAMM-" << ltime.tm_year << ltime.tm_mon << ltime.tm_mday << ltime.tm_hour << ltime.tm_min;
		return ss.str();
	}

#endif

	ARReenactmentGame::ARReenactmentGame()
		:Game("AR Reenactment"),
		tsdf_offset(0),
		debug_draw_skeleton(false),
		debug_shape_cylinders(false),
		debug_show_volumes(false),
		debug_inspect_texture_map(false),
		fbo1(0, 0),
		bg_color(0, 0, 0),
		temp_cam("Camera"),
		anim_frame( 0),
		elapsed(0),
		secret_offset(0),
		PTAMM_map_path("")
	{

#ifdef DEBUG_OUT
		debug_print_dir = generate_debug_print_dir();
		CreateDirectory(debug_print_dir.c_str(), nullptr);
#endif

		quadric = gluNewQuadric();
	}

	ARReenactmentGame::~ARReenactmentGame(){
		GVars3::GUI.UnRegisterCommand("ARR_NextSection");
		GVars3::GUI.UnRegisterCommand("ARR_PrevSection");
		GVars3::GUI.UnRegisterCommand("ARR_StartFrame");
		GVars3::GUI.UnRegisterCommand("ARR_Pause");

		gluDeleteQuadric(quadric);
	}

	void ARReenactmentGame::Reset(){

	}

	void ARReenactmentGame::Init(){

		//hook into the GVARS window, add extra menu options:

		GVars3::GUI.RegisterCommand("ARR_NextSection", GUICommandCallback, this);
		GVars3::GUI.RegisterCommand("ARR_PrevSection", GUICommandCallback, this);
		GVars3::GUI.RegisterCommand("ARR_StartFrame", GUICommandCallback, this);
		GVars3::GUI.RegisterCommand("ARR_ResetCFW", GUICommandCallback, this);
		GVars3::GUI.RegisterCommand("ARR_SaveCFW", GUICommandCallback, this);

		GVars3::GV2.Register(pause, "ARR_Pause", 0, GVars3::SILENT);

		static bool arr_menus_added = false;
		if (!arr_menus_added){
			//gui

			GVars3::GUI.ParseLine("GLWindow.AddMenu ARRMenu AR_Reenactment");
			GVars3::GUI.ParseLine("ARRMenu.AddMenuButton Root \"Next Section\" ARR_NextSection Root");
			GVars3::GUI.ParseLine("ARRMenu.AddMenuButton Root \"Prev Section\" ARR_PrevSection Root");
			GVars3::GUI.ParseLine("ARRMenu.AddMenuButton Root \"Start Frame\" ARR_StartFrame Root");
			GVars3::GUI.ParseLine("ARRMenu.AddMenuToggle Root \"Pause\" ARR_Pause Root");
			GVars3::GUI.ParseLine("ARRMenu.AddMenuButton Root \"Reset CFW\" ARR_ResetCFW Root");
			GVars3::GUI.ParseLine("ARRMenu.AddMenuButton Root \"Save CFW\" ARR_SaveCFW Root");
			arr_menus_added = true;
		}


		GVars3::GUI.LoadFile("reenactment.cfg"); //this file should define: ARReenactmentMotion and ARReenactmentVoxels

		static GVars3::gvar3<std::string> gv_video_directory("ARReenactmentMotion", "", GVars3::SILENT);
		static GVars3::gvar3<std::string> gv_voxel_path("ARReenactmentVoxels", "", GVars3::SILENT);
		static GVars3::gvar3<int> gv_startframe("ARReenactmentStartFrame", 0, GVars3::SILENT);
		static GVars3::gvar3<int> gv_numframes("ARReenactmentNumFrames", 1, GVars3::SILENT);
		static GVars3::gvar3<std::string> gv_package_directory("ARReenactmentPackage", "", GVars3::SILENT);
		static GVars3::gvar3<std::string> gv_sectionframes_directory("ARReenactmentSectionFrames", "", GVars3::SILENT);
		static GVars3::gvar3<int> gv_secret_offset("ARReenactmentSecretOffset", 0, GVars3::SILENT);
		static GVars3::gvar3<int> gv_draw_cylinders("ARReenactmentDrawCylinders", 0, GVars3::SILENT);
		//TODO: extra data file specifically for the AR Reenactment (e.g. sections)

		std::string package_directory = *gv_package_directory;
		secret_offset = *gv_secret_offset;

		cv::FileStorage fs;

		debug_shape_cylinders = *gv_draw_cylinders==1;

		if (package_directory == ""){
//
//			int startframe = *gv_startframe, numframes = *gv_numframes;
//			std::string video_directory = *gv_video_directory;
//			std::string voxel_path = *gv_voxel_path;
//			//load in the motion
//			std::stringstream filename_ss;
//			filename_ss << video_directory << "/bodypartdefinitions.xml.gz";
//
//			fs.open(filename_ss.str(), cv::FileStorage::READ);
//			for (auto it = fs["bodypartdefinitions"].begin();
//				it != fs["bodypartdefinitions"].end();
//				++it){
//				BodyPartDefinition bpd;
//				read(*it, bpd);
//				bodypart_definitions.push_back(bpd);
//			}
//			fs.release();
//
//			std::string extension = ".xml.gz";
//
//			//load frames
//			std::vector<std::string> filenames;
//			for (int i = startframe; i < startframe + numframes; ++i){
//				filename_ss.str("");
//				filename_ss << video_directory << "/" << i << extension;
//				filenames.push_back(filename_ss.str());
//			}
//			load_processed_frames(filenames, extension, bodypart_definitions.size(), frame_datas, true);
//
//			//build snhmaps
//			for (int i = 0; i < frame_datas.size(); ++i){
//				frame_snhmaps.push_back(SkeletonNodeHardMap());
//				cv_draw_and_build_skeleton(&frame_datas[i].mRoot, cv::Mat::eye(4, 4, CV_32F), frame_datas[i].mCameraMatrix, frame_datas[i].mCameraPose, &frame_snhmaps[i]);
//			}
//
//			std::string sectionframes_directory;
//			if (*gv_sectionframes_directory == ""){
//				sectionframes_directory = video_directory + "/section_frames.xml";
//			}
//			else{
//				sectionframes_directory = *gv_sectionframes_directory;
//			}
//
//			fs.open(sectionframes_directory, cv::FileStorage::READ);
//			if (fs.isOpened()){
//				cv::FileNode node = fs["section"];
//				for (auto it = node.begin(); it != node.end(); ++it){
//					section_frames.push_back(std::vector<int>());
//					cv::FileNode node2 = (*it)["frames"];
//					for (auto it2 = node2.begin(); it2 != node2.end(); ++it2){
//						int n;
//						(*it2) >> n;
//						section_frames.back().push_back(n);
//					}
//				}
//			}
//			fs.release();
//
//
//			//calculate model center in frame 0
//			//cv::Vec4f center_pt(0, 0, 0, 0);
//			//
//			//for (int i = 0; i < bodypart_definitions.size(); ++i){
//			//	cv::Mat bp_pt_m = get_bodypart_transform(bodypart_definitions[i], frame_snhmaps[0], frame_datas[0].mCameraPose)(cv::Range(0, 4), cv::Range(3, 4));
//			//	cv::Vec4f bp_pt = bp_pt_m;
//			//	center_pt += bp_pt;
//			//}
//			//
//			//center_pt /= center_pt(3);
//			//
//			//model_center = cv::Mat::eye(4, 4, CV_32F);
//			//cv::Mat(center_pt).copyTo(model_center(cv::Range(0, 4), cv::Range(3, 4)));
//			//model_center_inv = model_center.inv();
//
//			bodypart_frame_cluster = cluster_frames(64, bodypart_definitions, frame_snhmaps, frame_datas, 1000);
//
//			//load voxels & do marching cubes
//			load_voxels(voxel_path, bodypart_cylinders, bodypart_voxels, bodypart_TSDF_array, bodypart_weight_array, voxel_size);
//
//			triangle_vertices.resize(bodypart_definitions.size());
//			triangle_indices.resize(bodypart_definitions.size());
//			triangle_colors.resize(bodypart_definitions.size());
//
//			double num_vertices = 0;
//			for (int i = 0; i < bodypart_definitions.size(); ++i){
//				std::vector<TRIANGLE> tri_add;
//
//				cv::add(tsdf_offset * cv::Mat::ones(bodypart_TSDF_array[i].rows, bodypart_TSDF_array[i].cols, CV_32F), bodypart_TSDF_array[i], bodypart_TSDF_array[i]);
//
//				if (bodypart_TSDF_array[i].empty()){
//					tri_add = marchingcubes_bodypart(bodypart_voxels[i], voxel_size);
//				}
//				else{
//					tri_add = marchingcubes_bodypart(bodypart_voxels[i], bodypart_TSDF_array[i], voxel_size);
//				}
//				std::vector<cv::Vec4f> vertices;
//				std::vector<unsigned int> vertex_indices;
//				for (int j = 0; j < tri_add.size(); ++j){
//					for (int k = 0; k < 3; ++k){
//						cv::Vec4f candidate_vertex = tri_add[j].p[k];
//
//						bool vertices_contains_vertex = false;
//						int vertices_index;
//						for (int l = 0; l < vertices.size(); ++l){
//							if (vertices[l] == candidate_vertex){
//								vertices_contains_vertex = true;
//								vertices_index = l;
//								break;
//							}
//						}
//						if (!vertices_contains_vertex){
//							vertices.push_back(candidate_vertex);
//							vertices_index = vertices.size() - 1;
//						}
//						vertex_indices.push_back(vertices_index);
//					}
//				}
//				triangle_vertices[i].reserve(vertices.size() * 3);
//				triangle_colors[i].reserve(vertices.size() * 3);
//				triangle_indices[i].reserve(vertex_indices.size());
//				for (int j = 0; j < vertices.size(); ++j){
//					triangle_vertices[i].push_back(vertices[j](0));
//					triangle_vertices[i].push_back(vertices[j](1));
//					triangle_vertices[i].push_back(vertices[j](2));
//					triangle_colors[i].push_back(bodypart_definitions[i].mColor[0] * 255);
//					triangle_colors[i].push_back(bodypart_definitions[i].mColor[1] * 255);
//					triangle_colors[i].push_back(bodypart_definitions[i].mColor[2] * 255);
//				}
//				num_vertices += vertices.size();
//				for (int j = 0; j < vertex_indices.size(); ++j){
//					triangle_indices[i].push_back(vertex_indices[j]);
//				}
//			}
		}
		else{

			load_packaged_file(package_directory, bodypart_definitions, frame_datas, bodypart_frame_cluster, triangle_vertices, triangle_indices, bodypart_voxels, voxel_size, bodypart_cylinders);
			triangle_colors.resize(bodypart_definitions.size());
			for (int i = 0; i < bodypart_definitions.size(); ++i){
				for (int j = 0; j < triangle_indices[i].size(); ++j){
					triangle_colors[i].push_back(bodypart_definitions[i].mColor[0] * 0xff);
				}
			}
			for (int i = 0; i < frame_datas.size(); ++i){
				frame_snhmaps.push_back(SkeletonNodeHardMap());
				cv_draw_and_build_skeleton(&frame_datas[i].mRoot, cv::Mat::eye(4, 4, CV_32F), frame_datas[i].mCameraMatrix, frame_datas[i].mCameraPose, &frame_snhmaps[i]);
			}


			std::string sectionframes_directory;
			if (*gv_sectionframes_directory == ""){
			}
			else{
				sectionframes_directory = *gv_sectionframes_directory;

				fs.open(sectionframes_directory, cv::FileStorage::READ);
				if (fs.isOpened()){
					cv::FileNode node = fs["section"];
					for (auto it = node.begin(); it != node.end(); ++it){
						section_frames.push_back(std::vector<int>());
						cv::FileNode node2 = (*it)["frames"];
						for (auto it2 = node2.begin(); it2 != node2.end(); ++it2){
							int n;
							(*it2) >> n;
							section_frames.back().push_back(n);
						}
					}
				}
				fs.release();
			}



		}

		if (section_frames.empty()){
			section_frames.push_back(std::vector<int>());
			section_frames[0].resize(frame_datas.size());
			for (int i = 0; i < frame_datas.size(); ++i){
				section_frames[0][i] = i;
			}
		}

		camera_matrix_current = cv::Mat::eye(4, 4, CV_32F);

		if (PTAMM_to_kinect.empty()){
			fs.open("PTAMM_to_kinect.yml", cv::FileStorage::READ);
			fs["PTAMM_to_kinect"] >> PTAMM_to_kinect;
			fs.release();

			if (PTAMM_to_kinect.empty()){
				PTAMM_to_kinect = cv::Mat::eye(4, 4, CV_32F);
			}
		}


		bodypart_precalculated_rotation_vectors.resize(bodypart_definitions.size());
		for (int i = 0; i < bodypart_definitions.size(); ++i){
			bodypart_precalculated_rotation_vectors[i] = precalculate_vecs(bodypart_definitions[i], frame_snhmaps, frame_datas);
		}

	}

	void ARReenactmentGame::Draw3D(const GLWindow2 &gl_window, Map &map, SE3<> camera_from_world){

		//debug
		//unsigned int timestamp = std::time(nullptr);
		//std::stringstream debug_ss;
		//debug_ss << debug_print_dir << "/" << "debug" << timestamp << ".txt";
		//std::ofstream debug_os;
		//debug_os.open(debug_ss.str());


		int ptamm_fbo;
		glGetIntegerv(GL_FRAMEBUFFER_BINDING, &ptamm_fbo);

		int viewport[4];
		glGetIntegerv(GL_VIEWPORT, viewport);

		int viewport_width = viewport[2], viewport_height = viewport[3];

		if (fbo1.width == 0 || fbo1.height == 0){
			fbo1.width = viewport_width;
			fbo1.height = viewport_height;
			genFBO(fbo1);
		}

		camera_matrix_current.ptr<float>(0)[0] = temp_cam.GetParams()[0] * viewport_width;
		camera_matrix_current.ptr<float>(1)[1] = temp_cam.GetParams()[1] * viewport_height;
		camera_matrix_current.ptr<float>(0)[2] = temp_cam.GetParams()[2] * viewport_width - 0.5;
		camera_matrix_current.ptr<float>(1)[2] = temp_cam.GetParams()[3] * viewport_height - 0.5;
		//camera_matrix_current.ptr<float>(0)[0] = -3.6402221679687500e+002;
		//camera_matrix_current.ptr<float>(1)[1] = 3.6513830566406250e+002;
		//camera_matrix_current.ptr<float>(0)[1] = 1.1480305343866348e-002;
		//camera_matrix_current.ptr<float>(0)[2] = 2.5479405212402344e+002;
		//camera_matrix_current.ptr<float>(1)[2] = 2.0673567199707031e+002;

		cv::Mat flip_all = cv::Mat::eye(4, 4, CV_32F);
		flip_all.ptr<float>(0)[0] = 1;
		flip_all.ptr<float>(1)[1] = 1;
		flip_all.ptr<float>(2)[2] = 1;

		cv::Mat flip_z = cv::Mat::eye(4, 4, CV_32F);
		flip_all.ptr<float>(2)[2] = -1;

		cv::Mat flip_2 = cv::Mat::eye(4, 4, CV_32F);
		flip_2.ptr<float>(1)[1] = -1;


		cv::Mat flip_x = cv::Mat::eye(4, 4, CV_32F);
		flip_x.ptr<float>(0)[0] = -1;

		//convert camera_from_world into a cv::Mat
		cv::Mat camera_from_world_mat = cv::Mat::eye(4, 4, CV_32F);
		{
			Matrix<3, 3, double> rotation_matrix = camera_from_world.get_rotation().get_matrix();
			Vector<3, double> translation_vector = camera_from_world.get_translation();

			camera_from_world_mat.ptr<float>(0)[0] = rotation_matrix(0, 0);
			camera_from_world_mat.ptr<float>(1)[0] = rotation_matrix(1, 0);
			camera_from_world_mat.ptr<float>(2)[0] = rotation_matrix(2, 0);
			camera_from_world_mat.ptr<float>(0)[1] = rotation_matrix(0, 1);
			camera_from_world_mat.ptr<float>(1)[1] = rotation_matrix(1, 1);
			camera_from_world_mat.ptr<float>(2)[1] = rotation_matrix(2, 1);
			camera_from_world_mat.ptr<float>(0)[2] = rotation_matrix(0, 2);
			camera_from_world_mat.ptr<float>(1)[2] = rotation_matrix(1, 2);
			camera_from_world_mat.ptr<float>(2)[2] = rotation_matrix(2, 2);
			camera_from_world_mat.ptr<float>(0)[3] = translation_vector[0];
			camera_from_world_mat.ptr<float>(1)[3] = translation_vector[1];
			camera_from_world_mat.ptr<float>(2)[3] = translation_vector[2];

			
			//camera_from_world_mat = flip_all * camera_from_world_mat;
		}

		if (camera_from_world_capture.empty()){
			camera_from_world_capture = camera_from_world_mat.clone();
		}

		//cv::Mat current_transform = camera_from_world_mat * model_center_inv; 
		//cv::Mat current_transform = flip_all * PTAMM_to_kinect * flip_z * camera_from_world_mat * camera_from_world_capture.inv() * flip_z * PTAMM_to_kinect.inv() * flip_all; // *model_center_inv; //multiply PTAMM to Kinect inverse (B^-1); camera from world := A
		cv::Mat current_transform = flip_all *PTAMM_to_kinect *  camera_from_world_mat * camera_from_world_capture.inv() * PTAMM_to_kinect.inv() * flip_all*flip_x; // *model_center_inv; //multiply PTAMM to Kinect inverse (B^-1); camera from world := A

		//current_transform = cv::Mat::eye(4, 4, CV_32F);
		cv::Mat current_transform_t = current_transform.t();

		//debug
		//debug_os << "transformation\n" << current_transform << std::endl;
		//debug_os << "camera_from_world\n" << camera_from_world_mat << std::endl;
		//debug_os << "camera_from_world_inv\n" << camera_from_world_capture.inv() << std::endl;
		//debug_os << "camera_from_world_times_inv\n" << camera_from_world_mat * camera_from_world_capture.inv() << std::endl;
		//debug_os << "ptamm_to_kinect_inv\n" << PTAMM_to_kinect.inv() << std::endl;
		//debug_os << "camera_from_world_inv_times_p2k_inv\n" << camera_from_world_capture.inv() * PTAMM_to_kinect.inv() << std::endl;

		glMatrixMode(GL_PROJECTION);
		glPushMatrix();
		CVD::glMultMatrix(camera_from_world.inverse());

		//debug
		//cv::Mat old_projection_t(4, 4, CV_32F);
		//glGetFloatv(GL_PROJECTION_MATRIX, old_projection_t.ptr<float>());
		//debug_os << "ptamm_old_projection\n" << old_projection_t.t() << std::endl;

		if (opengl_projection.empty()){
			//opengl_projection.create(4, 4, CV_32F);
			//glGetFloatv(GL_PROJECTION_MATRIX, (GLfloat*)opengl_projection.data);
			//opengl_projection = opengl_projection.t();
			opengl_projection = build_opengl_projection_for_intrinsics(viewport,
				-camera_matrix_current.ptr<float>(0)[0],
				camera_matrix_current.ptr<float>(1)[1],
				camera_matrix_current.ptr<float>(0)[1],
				camera_matrix_current.ptr<float>(0)[2],
				camera_matrix_current.ptr<float>(1)[2] + secret_offset,
				viewport_width,
				viewport_height,
				0.001, 10, -1);


		}

		//debug
		//debug_os << "opengl_projection\n" << opengl_projection << std::endl;
		//debug_os << "camera_matrix_current\n" << camera_matrix_current << std::endl;

		cv::Mat opengl_projection_t = opengl_projection.t();
		glLoadIdentity();
		glMultMatrixf(opengl_projection_t.ptr<float>());

		
		glBindFramebuffer(GL_FRAMEBUFFER, fbo1.fboId);
		glClearColor(bg_color(0)/256.f, bg_color(1)/256.f, bg_color(2)/256.f, 1);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		glMatrixMode(GL_MODELVIEW);
		glEnable(GL_DEPTH_TEST);
		glLoadIdentity();
		//CVD::glMultMatrix(camera_from_world);
		//cv::Mat model_center_inv_t = model_center_inv.t();
		//glMultMatrixf(model_center_inv_t.ptr<float>());
		glMultMatrixf(current_transform_t.ptr<float>());

		//debug
		//debug_os << "bodypart_transforms\n" << "[\n";

		//render
		std::vector<cv::Vec3b> bp_colors(bodypart_definitions.size());
		for (int i = 0; i < bodypart_definitions.size(); ++i){
			bp_colors[i] = cv::Vec3b(bodypart_definitions[i].mColor[0] * 0xff, bodypart_definitions[i].mColor[1] * 0xff, bodypart_definitions[i].mColor[2] * 0xff);
		}

		glEnableClientState(GL_VERTEX_ARRAY);


		int start_bp = 0;
		int bp_inc = debug_shape_cylinders ? 1 : bodypart_definitions.size();
		cv::Mat output_img(viewport_height, viewport_width, CV_8UC4, cv::Scalar(0, 0, 0, 0));

		for (start_bp; start_bp < bodypart_definitions.size(); start_bp += bp_inc){
			int end_bp = start_bp + bp_inc;

			for (int i = start_bp; i < end_bp; ++i){
				glPushMatrix();

				if (debug_shape_cylinders){

					cv::Mat transform_t = (get_bodypart_transform(bodypart_definitions[i], frame_snhmaps[anim_frame], frame_datas[anim_frame].mCameraPose)).t();
					glMultMatrixf(transform_t.ptr<float>());

					glVertexPointer(3, GL_FLOAT, 0, triangle_vertices[i].data());
					glColorPointer(3, GL_UNSIGNED_BYTE, 0, triangle_colors[i].data());
					glColor3ubv(&(bp_colors[i][0]));

					renderCylinder(0, 0, 0, 0, bodypart_voxels[i].height * voxel_size, 0, bodypart_cylinders[i].width, 16, quadric);

				}
				else{
					cv::Mat transform_t = (get_bodypart_transform(bodypart_definitions[i], frame_snhmaps[anim_frame], frame_datas[anim_frame].mCameraPose) * get_voxel_transform(bodypart_voxels[i].width, bodypart_voxels[i].height, bodypart_voxels[i].depth, voxel_size)).t();
					glMultMatrixf(transform_t.ptr<float>());

					//debug
					//debug_os << "{\n" << "transform\n" << transform_t.t()
					//	<< "\ncombined_transform\n" << current_transform * transform_t.t()
					//	<< "\n}\n";

					glVertexPointer(3, GL_FLOAT, 0, triangle_vertices[i].data());
					glColorPointer(3, GL_UNSIGNED_BYTE, 0, triangle_colors[i].data());
					glColor3ubv(&(bp_colors[i][0]));

					glDrawElements(GL_TRIANGLES, triangle_indices[i].size(), GL_UNSIGNED_INT, triangle_indices[i].data());
				}

				glPopMatrix();
			}

			//debug
			//debug_os << "]\n";

			glDisableClientState(GL_VERTEX_ARRAY);


			//now take the different body part colors and map em to the proper textures

			if (!debug_show_volumes){

				glPixelStorei(GL_PACK_ALIGNMENT, 1);


				cv::Mat render_pretexture = gl_read_color(viewport[2], viewport[3]);

				cv::imwrite("renpre.png", render_pretexture);

				cv::Mat render_depth = gl_read_depth(viewport_width, viewport_height, opengl_projection);

				std::vector<std::vector<cv::Vec4f>> bodypart_pts_2d_withdepth_v(bodypart_definitions.size());
				std::vector<std::vector<cv::Point2i>> bodypart_pts_2d_v(bodypart_definitions.size());


				for (int y = 0; y < viewport_height; ++y){
					for (int x = 0; x < viewport_width; ++x){
						cv::Vec3b& orig_color = render_pretexture.ptr<cv::Vec3b>(y)[x];
						if (orig_color == bg_color) continue;
						for (int i = 0; i < bodypart_definitions.size(); ++i){
							if (orig_color(0) == (unsigned char)(bp_colors[i][0]) &&
								orig_color(1) == (unsigned char)(bp_colors[i][1]) &&
								orig_color(2) == (unsigned char)(bp_colors[i][2])
								){
								float depth = render_depth.ptr<float>(y)[x];
								bodypart_pts_2d_withdepth_v[i].push_back(cv::Vec4f(depth*x, depth*y,
									depth, 1));
								bodypart_pts_2d_v[i].push_back(cv::Point2i(x, y));
								break;
							}
						}
					}
				}

				cv::Mat flip_z_2 = cv::Mat::eye(4, 4, CV_32F);
				flip_z_2.ptr<float>(0)[0] = -1;
				flip_z_2.ptr<float>(2)[2] = -1;

				for (int i = 0; i < bodypart_definitions.size(); ++i){

					if (bodypart_pts_2d_withdepth_v[i].size() == 0) continue;

					//convert the vector into a matrix
					cv::Mat bodypart_pts = pointvec_to_pointmat(bodypart_pts_2d_withdepth_v[i]);

					//now multiply the inverse bodypart transform + the bodypart transform for the best frame
					//oh yeah, look for the best frame
					//this should probably be in a different function, but how do i access it in display...?
					//,maybe just global vars

					cv::Mat source_transform = current_transform * get_bodypart_transform(bodypart_definitions[i], frame_snhmaps[anim_frame], frame_datas[anim_frame].mCameraPose);

					if (debug_shape_cylinders){
						//remove the y-rotation
						cv::Vec3f axis_endpoint = source_transform(cv::Range(0, 3), cv::Range(3, 4));
						cv::Vec3f axis_direction = source_transform(cv::Range(0, 3), cv::Range(1, 2));
						cv::Vec3f front_vector = axis_direction.cross((-axis_endpoint).cross(axis_direction));
						cv::Vec3f side_vector = axis_direction.cross(front_vector);

						source_transform.ptr<float>(0)[2] = front_vector(0);
						source_transform.ptr<float>(1)[2] = front_vector(1);
						source_transform.ptr<float>(2)[2] = front_vector(2);

						source_transform.ptr<float>(0)[0] = side_vector(0);
						source_transform.ptr<float>(1)[0] = side_vector(1);
						source_transform.ptr<float>(2)[0] = side_vector(2);
					}

					cv::Mat source_transform_texsearch = flip_x * current_transform * get_bodypart_transform(bodypart_definitions[i], frame_snhmaps[anim_frame], frame_datas[anim_frame].mCameraPose);

					//std::vector<unsigned int> best_frames = sort_best_frames(bodypart_definitions[i], source_transform, frame_snhmaps, frame_datas, bodypart_frame_cluster[i]);
					std::vector<unsigned int> best_frames = sort_best_frames(bodypart_definitions[i], source_transform_texsearch, frame_snhmaps, frame_datas, bodypart_precalculated_rotation_vectors[i], bodypart_frame_cluster[i]);

					cv::Mat neutral_pts = (camera_matrix_current * source_transform).inv() * bodypart_pts;

					int search_limit = std::min((int)best_frames.size(), MAX_SEARCH);

					for (int best_frames_it = 0; best_frames_it < best_frames.size() && !neutral_pts.empty(); ++best_frames_it){

						unsigned int best_frame = best_frames[best_frames_it];

						//if (bpdv[i].mBodyPartName == "HEAD"){
						//	std::cout << "head best frame: " << best_frame << "; actual frame: " << anim_frame << std::endl;
						//}
						cv::Mat target_transform = get_bodypart_transform(bodypart_definitions[i], frame_snhmaps[best_frame], frame_datas[best_frame].mCameraPose);
						//cv::Mat bodypart_img_uncropped = uncrop_mat(frame_datas[best_frame].mBodyPartImages[i], cv::Vec3b(0xff, 0xff, 0xff));


						if (debug_shape_cylinders){

							//remove the y-rotation
							cv::Vec3f axis_endpoint = target_transform(cv::Range(0, 3), cv::Range(3, 4));
							cv::Vec3f axis_direction = target_transform(cv::Range(0, 3), cv::Range(1, 2));
							cv::Vec3f front_vector = axis_direction.cross((-axis_endpoint).cross(axis_direction));
							cv::Vec3f side_vector = axis_direction.cross(front_vector);

							target_transform.ptr<float>(0)[2] = front_vector(0);
							target_transform.ptr<float>(1)[2] = front_vector(1);
							target_transform.ptr<float>(2)[2] = front_vector(2);

							target_transform.ptr<float>(0)[0] = side_vector(0);
							target_transform.ptr<float>(1)[0] = side_vector(1);
							target_transform.ptr<float>(2)[0] = side_vector(2);
						}

						cv::Mat neutral_pts_occluded;
						std::vector<cv::Point2i> _2d_pts_occluded;

						if (debug_shape_cylinders){
							inverse_point_mapping(neutral_pts, bodypart_pts_2d_v[i], frame_datas[best_frame].mCameraMatrix, target_transform,
								frame_datas[best_frame].mBodyImage.mMat, frame_datas[best_frame].mBodyImage.mOffset, output_img, neutral_pts_occluded, _2d_pts_occluded, !debug_shape_cylinders, debug_inspect_texture_map);

						}
						else{
							inverse_point_mapping(neutral_pts, bodypart_pts_2d_v[i], frame_datas[best_frame].mCameraMatrix, target_transform,
								frame_datas[best_frame].mBodyPartImages[i].mMat, frame_datas[best_frame].mBodyPartImages[i].mOffset, output_img, neutral_pts_occluded, _2d_pts_occluded, !debug_shape_cylinders, debug_inspect_texture_map);
						}

						bodypart_pts_2d_v[i] = _2d_pts_occluded;
						if (!_2d_pts_occluded.empty()){
							neutral_pts = neutral_pts_occluded(cv::Range(0, 4), cv::Range(0, _2d_pts_occluded.size()));
						}
						else{
							break;
						}
					}

					//fill holes


					if (!bodypart_pts_2d_v[i].empty()){
						bool up = true;

						for (int iter = 0; iter < FILL_LIMIT && !bodypart_pts_2d_v[i].empty(); ++iter){

							for (int _n = 0; _n < bodypart_pts_2d_v[i].size(); ++_n){

								int n = up ? _n : bodypart_pts_2d_v[i].size() - _n - 1;

								int npix = 0;
								int px = bodypart_pts_2d_v[i][n].x;
								int py = bodypart_pts_2d_v[i][n].y;

								int av_b = 0, av_g = 0, av_r = 0;

								for (int fx = -FILL_NEIGHBORHOOD; fx < FILL_NEIGHBORHOOD; ++fx){
									for (int fy = -FILL_NEIGHBORHOOD; fy < FILL_NEIGHBORHOOD; ++fy){
										int wx = px + fx;
										int wy = py + fy;
										if (CLAMP(wx, wy, output_img.cols, output_img.rows)){
											const cv::Vec4b& color = output_img.ptr<cv::Vec4b>(wy)[wx];
											if (color(3) > 0){
												av_b += color(0);
												av_g += color(1);
												av_r += color(2);
												++npix;
											}
										}
									}
								}

								if (npix > 0){
									av_b /= npix;
									av_g /= npix;
									av_r /= npix;

									output_img.ptr<cv::Vec4b>(py)[px] = cv::Vec4b(av_b, av_g, av_r, 0xff);

									bodypart_pts_2d_v[i].erase(bodypart_pts_2d_v[i].begin() + n);
									--n;
								}
							}
							up = !up;
						}
					}


				}

			}
		}

		cv::Mat output_img_flip;
		cv::flip(output_img, output_img_flip, 0);

		glBindFramebuffer(GL_FRAMEBUFFER, ptamm_fbo);
		glEnable(GL_BLEND);

		//now display the rendered pts
		display_mat(output_img_flip, true);

		//cv::imwrite("output.png", output_img_flip);

		glBindFramebuffer(GL_FRAMEBUFFER, ptamm_fbo);

		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		//CVD::glMultMatrix(camera_from_world);
		glMultMatrixf(current_transform_t.ptr<float>());

		if (debug_draw_skeleton){

			glDisable(GL_DEPTH_TEST);
			glColor3f(1.f, 0., 0.);
			glBegin(GL_LINES);
			for (int i = 0; i < bodypart_definitions.size(); ++i){
				cv::Mat endpts(4, 2, CV_32F, cv::Scalar(1));
				endpts.ptr<float>(0)[0] = 0;
				endpts.ptr<float>(1)[0] = 0;
				endpts.ptr<float>(2)[0] = 0;
				endpts.ptr<float>(0)[1] = 0;
				endpts.ptr<float>(1)[1] = bodypart_voxels[i].height * voxel_size;
				endpts.ptr<float>(2)[1] = 0;

				endpts = get_bodypart_transform(bodypart_definitions[i], frame_snhmaps[anim_frame], frame_datas[anim_frame].mCameraPose) * endpts;
				glVertex3f(endpts.ptr<float>(0)[0], endpts.ptr<float>(1)[0], endpts.ptr<float>(2)[0]);
				glVertex3f(endpts.ptr<float>(0)[1], endpts.ptr<float>(1)[1], endpts.ptr<float>(2)[1]);
			}
			glEnd();
			glEnable(GL_DEPTH_TEST);

			glMatrixMode(GL_MODELVIEW);
			glPushMatrix();
			glMultMatrixf(flip_2.ptr<float>());
			glDisable(GL_DEPTH_TEST);
			glColor3f(0.f, 0., 1.f);
			glBegin(GL_LINES);
			for (int i = 0; i < bodypart_definitions.size(); ++i){
				cv::Mat endpts(4, 2, CV_32F, cv::Scalar(1));
				endpts.ptr<float>(0)[0] = 0;
				endpts.ptr<float>(1)[0] = 0;
				endpts.ptr<float>(2)[0] = 0;
				endpts.ptr<float>(0)[1] = 0;
				endpts.ptr<float>(1)[1] = bodypart_voxels[i].height * voxel_size;
				endpts.ptr<float>(2)[1] = 0;

				endpts = get_bodypart_transform(bodypart_definitions[i], frame_snhmaps[anim_frame], frame_datas[anim_frame].mCameraPose) * endpts;
				glVertex3f(endpts.ptr<float>(0)[0], endpts.ptr<float>(1)[0], endpts.ptr<float>(2)[0]);
				glVertex3f(endpts.ptr<float>(0)[1], endpts.ptr<float>(1)[1], endpts.ptr<float>(2)[1]);
			}
			glEnd();
			glEnable(GL_DEPTH_TEST);
			glPopMatrix();
		}

		glMatrixMode(GL_PROJECTION);
		glPopMatrix();

	}
	void ARReenactmentGame::Draw2D(const GLWindow2 gl_window, Map &map){

	}

	void ARReenactmentGame::HandleClick(Vector<2> vid_coords, Vector<2> UFB, Vector<3> ray_direction,
		Vector<2> plane, int button){

	}
	void ARReenactmentGame::HandleKeyPress(std::string key){
		switch (tolower(key[0])){
		case 'p':
			secret_offset+=1;
			opengl_projection = cv::Mat();
			break;
		case 'o':
			secret_offset -= 1;
			opengl_projection = cv::Mat();
			break;
		case 'c':
			debug_shape_cylinders = !debug_shape_cylinders;
			break;
		}
	}
	void ARReenactmentGame::Advance(){

		SYSTEMTIME time;
		GetLocalTime(&time);

		int temp = time.wMilliseconds - prev_time.wMilliseconds + 1000 * (time.wSecond - prev_time.wSecond);

		if (temp < 0) temp = 0;

		elapsed += temp;
		prev_time = time;


		while (elapsed >= (1000 / FRAMERATE))
		{

			elapsed -= 1000 / FRAMERATE;
			if (!*pause)
			{
				++current_frame_within_section;
			}

			if (current_frame_within_section >= section_frames[current_section].size()){
				current_frame_within_section = 0;
			}
		}

		anim_frame = section_frames[current_section][current_frame_within_section];

		if (anim_frame >= frame_snhmaps.size()){
			anim_frame = 0;
		}
	}

	std::string ARReenactmentGame::Save(std::string map_path){

		cv::FileStorage fs;

		fs.open(map_path + "/PTAMM_to_kinect.yml", cv::FileStorage::WRITE);
		fs << "PTAMM_to_kinect" << PTAMM_to_kinect;
		fs.release();

		fs.open(map_path + "/Camera_from_world_initial.yml", cv::FileStorage::WRITE);
		fs << "Camera_from_world_initial" << camera_from_world_capture;
		fs.release();

		PTAMM_map_path = map_path;

		return "/";
	}
	void ARReenactmentGame::Load(std::string map_path){
		cv::FileStorage fs;

		fs.open(map_path + "/PTAMM_to_kinect.yml", cv::FileStorage::READ);
		fs["PTAMM_to_kinect"] >> PTAMM_to_kinect;
		fs.release();

		fs.open(map_path + "/Camera_from_world_initial.yml", cv::FileStorage::READ);
		fs["Camera_from_world_initial"] >> camera_from_world_capture;
		fs.release();


		fs.open(map_path + "/Section_frames.yml", cv::FileStorage::READ);
		//TO DO
		fs.release();

		PTAMM_map_path = map_path;

		Init();
	}

	void ARReenactmentGame::section_start_frame(){
		current_frame_within_section = 0;
	}

	void ARReenactmentGame::reset_cfw(){
		camera_from_world_capture = cv::Mat();
	}

	void ARReenactmentGame::save_cfw(){
		cv::FileStorage fs;
		fs.open(PTAMM_map_path + "/Camera_from_world_initial.yml", cv::FileStorage::WRITE);
		fs << "Camera_from_world_initial" << camera_from_world_capture;
		fs.release();

		fs.open(PTAMM_map_path + "PTAMM_to_kinect.yml", cv::FileStorage::WRITE);
		fs << "PTAMM_to_kinect" << PTAMM_to_kinect;
		fs.release();
	}

	void ARReenactmentGame::GUICommandCallback(void *ptr, string command, string params){
		ARReenactmentGame * g = static_cast<ARReenactmentGame*>(ptr);
		if (command == "ARR_NextSection"){
			g->current_section += 1;
			if (g->current_section >= g->section_frames.size()){
				g->current_section = g->section_frames.size() - 1;
			}

			g->section_start_frame();

		}
		else if (command == "ARR_PrevSection"){
			g->current_section -= 1;
			if (g->current_section < 0){
				g->current_section = 0;
			}

			g->section_start_frame();
		}
		else if (command == "ARR_StartFrame"){
			g->section_start_frame();
		}
		else if (command == "ARR_ResetCFW"){
			g->reset_cfw();
		}
		else if (command == "ARR_SaveCFW"){
			g->save_cfw();
		}
	};
}

