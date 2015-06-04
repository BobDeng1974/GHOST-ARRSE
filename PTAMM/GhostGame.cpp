#if 0

#include "GLWindow2.h"
#include "TooN\TooN.h"
#include "OpenGL.h"
#include "cvd\gl_helpers.h"

#include <array>
#include <algorithm>
#include <stdlib.h>
#include <fstream>
#include <ctime>

#include "tinyxml.h"
#include <gvars3/instances.h>
#include "Map.h"
#include "MapPoint.h"
#include "ATANCamera.h"

#include "GhostGame.h"
#include "PTAM2Kinect.h"
#include "cylinder.h"
#include "ghostutil.h"
#include "loader.h"
#include "cylindermapping.h"
#include "definitions.h"
#include "KinectManager.h"
#include "cvutil.h"
#include "process.h"
#include "ghostcam.h"
#include "ghostdraw.h"
#include "GLCV.h"

#define __USING_3D 0
#define PI 3.14159265359
#define MAXLENGTHDIFF 1.3
#define MINLENGTHDIFF 0.7
#define TIMERATE 1

using namespace GVars3;
using std::cout;
using std::pair;

namespace GHOST{


Matrix<4,4> camMatrix;

GhostGame::GhostGame():Game("Ghost"),version(1.6),
	camera("Camera"),
	camParams(camera.GetParams())
{
	Reset();
	quadric=gluNewQuadric();
	gluQuadricNormals(quadric, GLU_SMOOTH);
	
	camMatrix = TooN::Zeros;

	initDefinitions();

	setCameraMatrixScene(camParams[0], camParams[1], camParams[2], camParams[3], CAPTURE_SIZE_X, CAPTURE_SIZE_Y);
	//cylinderBody.calculateCameraMatrix();

	//gvars
	GV2.Register(gridproj,				"GH_GridProjection", 0, SILENT);
	GV2.Register(draw2d,				"GH_Draw2D", 0, SILENT);
	GV2.Register(drawDots,				"GH_DrawDots", 0, SILENT);
	GV2.Register(pause,					"GH_Pause", 0, SILENT);
	GV2.Register(cylinders,				"GH_Cylinders", 0, SILENT);
	GV2.Register(usePose,				"GH_UsePose", 0, SILENT);
	GV2.Register(drawOnPic,				"GH_DrawOnPic", 0, SILENT);
	GV2.Register(cyl_drawCylinders,		"GH_DrawCylinders", 0, SILENT);
	GV2.Register(cyl_indivLimbs,		"GH_IndividualLimbs", 0, SILENT);
	GV2.Register(cyl_individualRender,	"GH_IndividualRender", 0, SILENT);
	GV2.Register(cyl_colorPixels,		"GH_ColorPixels", 0, SILENT);
	GV2.Register(cyl_norm,				"GH_Norm", 0, SILENT);
	GV2.Register(cyl_norot,				"GH_NoRotation", 0, SILENT);
	GV2.Register(cyl_shittyOption,		"GH_ShittyOption", 0, SILENT);
	GV2.Register(writecmp,				"GH_WriteCmp", 0, SILENT);

	//gui
	GUI.RegisterCommand("GH_NextSection", GUICommandCallBack, this);
	GUI.RegisterCommand("GH_PrevSection", GUICommandCallBack, this);
	GUI.RegisterCommand("GH_StartFrame", GUICommandCallBack, this);

	static bool menusAdded = false;

	if (!menusAdded){
		GUI.ParseLine("GLWindow.AddMenu GhostMenu AR_Reenactment");
		GUI.ParseLine("GhostMenu.AddMenuButton Root \"Next Section\" GH_NextSection Root");
		GUI.ParseLine("GhostMenu.AddMenuButton Root \"Prev Section\" GH_PrevSection Root");
		GUI.ParseLine("GhostMenu.AddMenuButton Root \"Start Frame\" GH_StartFrame Root");
		GUI.ParseLine("GhostMenu.AddMenuToggle Root \"Pause\" GH_Pause Root");
		menusAdded = true;
	}
};

GhostGame::~GhostGame(){
	Reset();
	gluDeleteQuadric(quadric);
	glDestroy();


	GUI.UnRegisterCommand("GH_NextSection");
	GUI.UnRegisterCommand("GH_PrevSection");
	GUI.UnRegisterCommand("GH_StartFrame");

};


SYSTEMTIME lastTime;
int numFPS = 0;
float currAveFPS = 0;

void GhostGame::Draw2D(const GLWindow2 &glWindow, Map &map){
	glEnable(GL_BLEND);

	glColor3f(1.0,1.0,1.0);

#if !TABLET_GUI

	std::string pout;
	if(recording){
		pout += "Enter: stop ";

	}else{
		pout += "Enter: record ";
	}

	pout += "A: reset record ";


	if(playing){
		pout += "Z: stop ";
	}else{
		pout += "Z: playback ";
	}
		
	pout += "X: recalib ";

	if(!*gridproj)
		pout += "C: grid ";
	else
		pout += "C: stop ";

	/*if(!draw2d)
		pout += "V: 2D ";
	else
		pout += "V: 2D off ";*/

	//pout += "B: vidsave N: vidload ";
	pout += "S: refresh skeleton ";
	pout += "P: pause";

	glWindow.PrintString( CVD::ImageRef(10, glWindow.size().y - 20), pout);

#endif

	//FPS
	std::stringstream FPSss;

	//check FPS

	SYSTEMTIME thisTime;
	GetSystemTime(&thisTime);
	double seconds = thisTime.wSecond - lastTime.wSecond + (thisTime.wMilliseconds - lastTime.wMilliseconds)/1000.;
	lastTime = thisTime;
	double FPSnow = 1./seconds;

	FPSss << "FPS: " << FPSnow;
	
	double sumFPS = 0;
	sumFPS = currAveFPS*numFPS;

	sumFPS += FPSnow;
	++numFPS;

	currAveFPS = sumFPS/numFPS;
	
	FPSss << " average: " << currAveFPS;

	glWindow.PrintString( CVD::ImageRef(10, glWindow.size().y - 40), FPSss.str());

	glWindow.PrintString( CVD::ImageRef(10, glWindow.size().y - 60), dispstring);

	glDisable(GL_BLEND);
};

void GhostGame::camWorld_mat(){
	TooN::Matrix<3,3,double> sorot = mse3CfW.get_rotation().get_matrix();
	TooN::Vector<3, double> sotrans = mse3CfW.get_translation();

	float mparts[] = {sorot(0,0), sorot(0,1), sorot(0,2), sotrans[0],
						sorot(1,0), sorot(1,1), sorot(1,2), sotrans[1],
						sorot(2,0), sorot(2,1), sorot(2,2), sotrans[2],
						0, 0, 0, 1};

	matCfW = cv::Mat(4,4,cv::DataType<float>::type, mparts).clone();
}

void GhostGame::Draw3D(const GLWindow2 &glWindow, Map &map, TooN::SE3<> se3CfromW){
	
	if(!invprojcalc){
		glMatrixMode(GL_PROJECTION);
		glPushMatrix();
		
		CVD::glMultMatrix(se3CfromW.inverse());

		calcInvProjMat(CAPTURE_SIZE_X+_capx, CAPTURE_SIZE_Y+((CAPTURE_SIZE_Y+0.0)/CAPTURE_SIZE_X)*_capx);

		std::cout << "capx: " << _capx << std::endl;

		glPopMatrix();

		invprojcalc = true;

	}
	
	//try
	//{

	mpMap = &map;
	mse3CfW = se3CfromW;
	camWorld_mat();

	mapLoaded = true;
	
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	CVD::glMultMatrix(TooN::SE3<>());

	
      glEnable(GL_BLEND);
	  glDisable(GL_DEPTH_TEST);
	  glDisable(GL_CULL_FACE);
	  glDisable(GL_ALPHA_TEST);
	  glDisable(GL_TEXTURE_2D);

	
	glPointSize(20);
	
	skeleton.calibPTAM(map.vpPoints, mse3CfW);
#if 0
	if(playing)
	{

		glMatrixMode (GL_PROJECTION);

		glPushMatrix();

		CVD::glMultMatrix(mse3CfW.inverse());

		glMatrixMode(GL_MODELVIEW);

		glPushMatrix();

		glLoadIdentity();

		//renderCylinder_convenient(0,-1,-5,0,1,-5,1,15);

		//probably the only matrix we need is the camera from world

		//check how to convert from TooN::se3 to cv::Mat

		DrawOptions d;
		d.drawCylinders = *cyl_drawCylinders;
		d.colorPixels = *cyl_colorPixels;
		d.drawOnPic = *drawOnPic;
		d.noRot = *cyl_norot;
		d.writecmp = *writecmp;
		d.usePose = *usePose;
		d.indivLimbs = *cyl_indivLimbs;
		d.shittyOption = *cyl_shittyOption;
		d.individualRender = *cyl_individualRender;

		if(*cylinders) drawSkeletonCylinder(matCfW, currAnim, &vidRecord, &wcSkeletons, &cylinderBody,  &chosenBest, &dispstring);
		GUI.ParseLine("GH_WriteCmp=0");
		GUI.ParseLine("GH_DrawOnPic=0");
		

		glMatrixMode(GL_PROJECTION);
		glPopMatrix();

		glMatrixMode(GL_MODELVIEW);
	
		glPopMatrix();
	
	}
#elif 1
	if(playing && vidRecord[currAnim].valid){
		cv::Mat draw_(CAPTURE_SIZE_Y, CAPTURE_SIZE_X, CV_8UC4, cv::Scalar(0,0,0,0));
		cv::Mat zBuf;

		cv::FileStorage fs("ptammat.yml", cv::FileStorage::WRITE);
		fs << "matCfW" << matCfW;
		fs.release();

		ghostdraw_parallel(currAnim, /*getScaleMatrix(1,-1,1)*/matCfW, vidRecord, wcSkeletons, cylinderBody, limbrary, draw_, zBuf, GD_DRAW);


		cv::Mat draw;
		cv::flip(draw_, draw, 0);
		

		GLuint tex;
		load_texture(draw, &tex);

		glMatrixMode(GL_PROJECTION);
		glPushMatrix();
		glLoadIdentity();

		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
		glLoadIdentity();

		glEnable( GL_TEXTURE_2D );
		glBindTexture(GL_TEXTURE_2D, tex);
		glColor3f(1,1,1);

		glBegin( GL_QUADS );
		glTexCoord2d(0.0,0.0); glVertex2d(-1.0,-1.0);
		glTexCoord2d(1.0,0.0); glVertex2d(1.0,-1.0);
		glTexCoord2d(1.0,1.0); glVertex2d(1.0,1.0);
		glTexCoord2d(0.0,1.0); glVertex2d(-1.0,1.0);
		glEnd();

		glBindTexture(GL_TEXTURE_2D, NULL);
		glDeleteTextures(1, &tex);
		glDisable(GL_TEXTURE_2D);

		glMatrixMode(GL_PROJECTION);
		glPopMatrix();

		glMatrixMode(GL_MODELVIEW);
		glPopMatrix();
	}


#endif

	//grid

	if(*gridproj){
		glBegin(GL_POINTS);

		float g = 0;

		for(auto it=skeleton.gridpts.begin();it!=skeleton.gridpts.end();++it){
			glColor3f(1.0, g, 1.0);
			CVD::glVertex(*it);
			g+=1.0/skeleton.gridpts.size();
		}
		glPointSize(15);
		
		g = 0;
		for(auto it=skeleton.gridpts2.begin();it!=skeleton.gridpts2.end();++it){
			glColor3f(1.0, g, 0);
			CVD::glVertex(*it);
			g += 1.0/skeleton.gridpts2.size();
		}

		glEnd();

		glBegin(GL_LINES);

		glColor3f(1.0,1.0,1.0);
		int minsz = std::min(skeleton.gridpts.size(), skeleton.gridpts2.size());
		for(int i=0;i<minsz;++i){
			CVD::glVertex(skeleton.gridpts[i]);
			CVD::glVertex(skeleton.gridpts2[i]);
		}

		glEnd();
	}
	

	if(!good) return;
	
	if(playing){
		drawSkeleton = &(wcSkeletons[currAnim]);
	}else{
		if(!hasCurrentSkeleton) return;
		drawSkeleton = &(currSkeleton[chosenSkeleton]);
		
		glMatrixMode (GL_PROJECTION);
		glPushMatrix();
		CVD::glMultMatrix(mse3CfW.inverse());
	}

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	CVD::glMultMatrix(TooN::SE3<>());
	
	
	glColor3f(0.2, 0.1, 0.9);
	
	if(drawSkeleton->points.empty()) return;

	if(true || *drawDots)
	{
		glPointSize(10);
		glBegin(GL_POINTS);
		

		glColor3f(1.0, 1.0, 1.0);


		for(int i=0;i<NUMJOINTS;++i){
			//CVD::glVertex(drawSkeleton->points[i]);
			glVertex(drawSkeleton->points.col(i));
		}

		glEnd();
		
		glColor3f(1.0, 0.0, 0.4);
		glPointSize(5);

		glBegin(GL_POINTS);

		for(int i=0;i<NUMJOINTS;++i){
			//CVD::glVertex(drawSkeleton->points[i]);
			glVertex(drawSkeleton->points.col(i));
		}

		glEnd();
		/*
		if(!draw2d){
			glBegin(GL_LINES);

			for(int i=0;i<NUMLIMBS;++i){
				//CVD::glVertex(drawSkeleton->points[getLimbmap()[i].first]);
				//CVD::glVertex(drawSkeleton->points[getLimbmap()[i].second]);
				glVertex(drawSkeleton->points.col(getLimbmap()[i].first));
				glVertex(drawSkeleton->points.col(getLimbmap()[i].second));
			}

			glEnd();
		}*/

	}

	if(!playing){

		glMatrixMode(GL_PROJECTION);
		glPopMatrix();
	}
	
	//}catch(exception& e){
	//	std::cout << "!!!!EXCEPTION!!!! " << e.what() << std::endl;
	//}

};

TooN::Vector<2> GhostGame::project(TooN::Vector<4> v, float width, float height){
	camMatrix(0,0) = camParams[0] * width;
	camMatrix(1,1) = camParams[1] * height;
	camMatrix(0,2) = camParams[2] * width - 0.5;
	camMatrix(1,2) = camParams[3] * height - 0.5;
	camMatrix(2,2) = 1;
	camMatrix(3,3) = 1;

	TooN::Vector<4> projected = camMatrix * v;
	TooN::Vector<2> _2D;
	_2D[0] = projected[0]/projected[2];
	_2D[1] = projected[1]/projected[2];

	return _2D;
};

TooN::Vector<2> GhostGame::project0(TooN::Vector<4> v){

	TooN::Vector<2> _2D;
	_2D[0] = v[0]/v[2];
	_2D[1] = v[1]/v[2];

	return _2D;
};

void GhostGame::HandleClick(Vector<2> v2VidCoords, Vector<2> v2UFB, Vector<3> v3RayDirnW, 
                             Vector<2> v2Plane, int nButton)
{
	if(!mapLoaded) return;

	MapPoint closest;
	double threshold = 25;
	double minDist = threshold;

	Matrix<4,4> camMatrix = TooN::Zeros;
	camMatrix(0,0) = camParams[0] * mnWindowWidth;
	camMatrix(1,1) = camParams[1] * mnWindowHeight;
	camMatrix(0,2) = camParams[2] * mnWindowWidth - 0.5;
	camMatrix(1,2) = camParams[3] * mnWindowHeight - 0.5;
	camMatrix(2,2) = 1;
	camMatrix(3,3) = 1;

	for(auto it = mpMap->vpPoints.begin(); it != mpMap->vpPoints.end(); ++it){
		Vector<3> worldPos = (*it)->v3WorldPos;
		Vector<4> worldPosHom = TooN::makeVector(worldPos[0], worldPos[1], worldPos[2], 1);
		Vector<4> camPos = camMatrix * mse3CfW * worldPosHom;


		camPos[0] = camPos[0]/camPos[2];
		camPos[1] = camPos[1]/camPos[2];

		double distance = (camPos[0]-v2VidCoords[0])*(camPos[0]-v2VidCoords[0]) +
			(camPos[1]-v2VidCoords[1])*(camPos[1]-v2VidCoords[1]);

		if(distance < threshold){
			if(distance < minDist){
				closest = **it;
				minDist = distance;
			}
		}
	}
	
	Vector<3> camPos = mse3CfW * closest.v3WorldPos;
	
	KINECT::DepthXY * _depthdata = new KINECT::DepthXY[CAPTURE_SIZE_X * CAPTURE_SIZE_Y];
	KINECT::getKinectData_depth_raw(_depthdata);

	std::cout << "depth: " << _depthdata[CAPTURE_SIZE_X-1-(int)v2VidCoords[0]+(int)v2VidCoords[1]*CAPTURE_SIZE_X].depth << std::endl;
	//std::cout << "x : "<< _depthdata[CAPTURE_SIZE_X-1-(int)v2VidCoords[0]+(int)v2VidCoords[1]*WIDTH].x << " vs " << CAPTURE_SIZE_X-1-(int)v2VidCoords[0] << std::endl;

	cv::Vec3f skpt = KINECT::mapDepthToSkeletonPoint(_depthdata[CAPTURE_SIZE_X-1-(int)v2VidCoords[0]+(int)v2VidCoords[1]*CAPTURE_SIZE_X]);

	std::cout << "skel space: " << skpt[0] << " " << skpt[1] << " " << skpt[2] << std::endl << std::endl;

	TooN::Vector<4> skelPos = camMatrix * TooN::makeVector(skpt[0], skpt[1], skpt[2], 1);
	cout << "unproject skel space: " << skelPos[0]/skelPos[2] << " " << skelPos[1]/skelPos[2] << " " << endl;

	if(minDist < threshold){
		

		cout << "CCS point: " << camPos[0] << " " << camPos[1] << " " << camPos[2] << std::endl;
		
		cout << "ratio: " << camPos[0]/skpt[0] << " " << camPos[1]/skpt[1] << " " << camPos[2]/skpt[2] << std::endl;

		camPos = camMatrix.slice(0,0,3,3) * camPos;

		cout << "unproject CCS point: " << camPos[0]/camPos[2] << " " << camPos[1]/camPos[2] << " " << endl << endl;


	}
	else cout << "no map point at that location...\n";

	delete [] _depthdata;
};

void GhostGame::ProjectGrid(){
	if(!mapLoaded) return;
	//skeleton.ProjectGrid(mse3CfW);

	//KINECT::GridProjection(mse3CfW, &skeleton.gridpts, &skeleton.gridpts2, mnWindowWidth, mnWindowHeight);


	skeleton.gridpts.clear();
	skeleton.gridpts2.clear();

	KINECT::DepthXY * _depthdata = new KINECT::DepthXY[CAPTURE_SIZE_X * CAPTURE_SIZE_Y];

	getKinectData_depth_raw(_depthdata);

	TooN::SE3<> cam2world = mse3CfW.inverse();
	TooN::Matrix<4, 4> K2P = KINECT::getPTAMfromKinect();

	const int width = CAPTURE_SIZE_X;
	const int height = CAPTURE_SIZE_Y;

	for (int x = 64; x<width; x += 64){
		for (int y = 64; y<height; y += 64){
			float dX = x;
			float dY = height - 1 - y;
			float depth = _depthdata[(int)dX + (int)dY*width].depth;
			if (depth == 0) continue;

			cv::Vec3f skeletonPt_cv = mapDepthToSkeletonPoint(_depthdata[(int)dX + (int)dY*width]);
			TooN::Vector<4> skeletonPt = TooN::makeVector(skeletonPt_cv(0), skeletonPt_cv(1), skeletonPt_cv(2), 1);

			skeleton.gridpts.push_back(cam2world * K2P * (skeletonPt));

			/*skeletonPt /= skeletonPt[2];
			skeletonPt[3] = 1;
			gridpts2->push_back(cam2world * (skeletonPt));*/

			TooN::Vector<4, float> temp = K2P * (skeletonPt);
			temp /= temp[2];
			temp[3] = 1;
			skeleton.gridpts2.push_back(cam2world * temp);


			//std::cout << _depthdata[x+y*WIDTH] << ", (" << x << ", " << y << ") -> " << gridpts2->back() << std::endl;

		}
	}
	std::cout << "Grid size: " << skeleton.gridpts.size() << std::endl;

	delete[] _depthdata;
};

void GhostGame::setStartFrame(){

	//check valid
	if(currentDivision >= vidDivisions.size()){
		currentDivision = vidDivisions.size()-1;
	}
	if(currentDivision < 0){
		currentDivision = 0;
	}
	
	currAnim = vidDivisions[currentDivision].first;
	currAnimF = currAnim;
}

void GhostGame::advanceFrame(){

	currAnimF += TIMERATE;
	currAnim = floor(currAnimF);
}

void GhostGame::HandleKeyPress(std::string sKey){
	char buff[10];
	if(sKey == "Enter"){
		if(recording){
			//processVideo();
			//we do it separately now

			int divisionStart=currentDivision==0?0:vidDivisions[currentDivision-1].second;

			vidDivisions.push_back(pair<int,int>(divisionStart, vidRecord.size()));
			++currentDivision;
			dumpVideo();
		}

		recording = !recording;
		playing = false;
	}
	else if(sKey == "A" || sKey == "a"){
		//reset everything


		vidRecord.clear();
		vidDivisions.clear();

		refreshVidDirectory();

		playing=false;
		recording=false;
		currentDivision=0;

	}
	else if(sKey == "S" || sKey == "s"){
		KINECT::cycleTrackedSkeleton();
	}
	else if(sKey == "Z" || sKey == "z"){
		if(recording) processVideo();
		playing = !playing;
		recording = false;
	}
	else if(sKey == "X" || sKey == "x"){
		//recalib
		skeleton.recalibPTAM();
	}
	else if(sKey == "C" || sKey == "c"){
		//grid projection
		*gridproj = !*gridproj;
		if(*gridproj){
			ProjectGrid();
		}
	}
	else if(sKey == "V" || sKey == "v"){
		//2d draw
		//*draw2d = !*draw2d;
	}
	else if(sKey == "B" || sKey == "b"){
		//save vid
		//skeleton.processVideo(&flesh);
		//skeleton.SaveVideo();
		SaveVideo(&vidRecord, getCameraMatrixScene());
	}
	else if(sKey == "N" || sKey == "n"){
		//load from vid
		/*skeleton.Reset();
		vidRecord.clear();
		currentDivision = 0;*/

		LoadVideo(matCfW, KINECT::getPTAMfromKinect_mat(), &vidRecord, &wcSkeletons);
		//SpecialLoad(matCfW, KINECT::getPTAMfromKinect_mat(), &vidRecord);
		cylinderBody.setVidRecord(&vidRecord);

		calculateWorldCoordinateSkeletons(KINECT::getPTAMfromKinect_mat(),  &vidRecord, &wcSkeletons);

		//build
		buildCylinderBody(&vidRecord, &cylinderBody);

		calculateSkeletonOffsetPoints(vidRecord, wcSkeletons, cylinderBody);

		good = true;
	}
	else if(sKey == "M" || sKey == "m"){
		*drawDots = !*drawDots;
	}
	else if(sKey == "P" || sKey == "p"){
		if(*pause == 0) GUI.ParseLine("GH_Pause=1");
		else GUI.ParseLine("GH_Pause=0");
	}
	else if(sKey == "Y" || sKey == "y"){
		*cylinders = !*cylinders;
	}
	else if(sKey == "1"){
		//*cyl_drawCylinders = !*cyl_drawCylinders;
	}
	else if(sKey == "2"){
		std::stringstream ss;
		ss << "GH_ColorPixels=" << !*cyl_colorPixels;
		//GUI.ParseLine(std::string("GH_ColorPixels=")+itoa(!*cyl_colorPixels, buff, 10));
		GUI.ParseLine(ss.str());
	}
	else if(sKey == "i"){
		invprojcalc = false;
	}
	else if(sKey == "3"){
		//*cyl_norot = !*cyl_norot;
	}
	else if(sKey == "w"){
		_capx-=10;
		invprojcalc = false;
	}
	else if(sKey == "e"){
		_capx+=10;
		invprojcalc = false;
	}
	else if(sKey == "t"){
		GUI.ParseLine("GH_WriteCmp=1");
	}
	else if(sKey == "4"){
		GUI.ParseLine("GH_DrawOnPic=1");
	}
	else if(sKey == "d" || sKey == "D"){
		++currentDivision;
		if(currentDivision >= vidDivisions.size()){
			currentDivision = vidDivisions.size()-1;
		}

		setStartFrame();
	}
	else if(sKey == "f" || sKey == "F"){
		--currentDivision;
		if(currentDivision < 0){
			currentDivision = 0;
		}

		setStartFrame();
	}
	else if(sKey == "u"){
		//*usePose = !*usePose;
	}
	else if(sKey == "l"){
		//*cyl_indivLimbs = !*cyl_indivLimbs;
	}
	else if(sKey == "6"){
		//*cyl_shittyOption = !*cyl_shittyOption;
	}
}
void GhostGame::Reset(){
	GUI.ParseLine("GH_ShittyOption=0");
	GUI.ParseLine("GH_NoRotation=0");
	GUI.ParseLine("GH_GridProjection=0");
	GUI.ParseLine("GH_Draw2D=1");
	GUI.ParseLine("GH_DrawDots=0");
	GUI.ParseLine("GH_Pause=0");
	GUI.ParseLine("GH_Cylinders=1");
	GUI.ParseLine("GH_UsePose=1");
	GUI.ParseLine("GH_DrawOnPic=0");
	GUI.ParseLine("GH_IndividualRender=0");
	GUI.ParseLine("GH_IndividualLimbs=0");
	GUI.ParseLine("GH_WriteCmp=0");
	GUI.ParseLine("GH_DrawCylinders=1");
	GUI.ParseLine("GH_ColorPixels=1");
	GUI.ParseLine("GH_Norm=0");

	vidRecord.clear();
	refreshVidDirectory();

	mapLoaded = false;
	good = false;
	recording = false;
	playing = false;
	currAnim = 0;
	//animRecord.clear();
	GetLocalTime(&prevtime);
	elapsed = 0;
	camParams = camera.GetParams();
	skeleton.Reset();
	vidDivisions.clear();
	currentDivision = 0;
	
	invprojcalc = false;

	_capx = 560;
};

void GhostGame::processVideo(){
	//skeleton.processVideo(&flesh);
	//flesh.load_textures();
	buildCylinderBody(&vidRecord, &cylinderBody);

};


void GhostGame::Init(){
	//skeleton.Init();
	initLoader();
	//calcInvProjMat(CAPTURE_SIZE_X, CAPTURE_SIZE_Y);
	glInit();

	refreshVidDirectory();
};

void GhostGame::Advance(){
	
	
	
	if(!mapLoaded) return;

	SYSTEMTIME time;
	GetLocalTime(&time);

	int temp = time.wMilliseconds - prevtime.wMilliseconds + 1000*(time.wSecond - prevtime.wSecond);

	if(temp < 0) temp = 0;

	elapsed += temp;
	prevtime = time;


	while(elapsed >= (1000/FRAMERATE))
	{

		if(!playing){ 
			
			elapsed = 0;
			Skeleton newSkeleton;
			cv::Mat origPoints;

			if(KINECT::skeletonIsGood())
			{
				KINECT::setRefresh(false);
				good = true;

				newSkeleton = KINECT::getSkeleton();

				KINECT::augmentSkeleton( &newSkeleton );
				origPoints = newSkeleton.points.clone();

				newSkeleton.points = KINECT::getPTAMfromKinect_mat() * newSkeleton.points;
				currSkeleton[0] = newSkeleton;
				chosenSkeleton = 0;
				hasCurrentSkeleton = true;
			}

			if(KINECT::skeletonIsGood() || FORCE_VIDEO_CAPTURE){
				
				if(recording){
					//vid record . push things
				
					SkeleVideoFrame svf;
					
					//reworked. PROCESS

					if(KINECT::skeletonIsGood()){
						svf.kinectPoints.points = origPoints;
						svf.kinectPoints.states = newSkeleton.states;
						//extract person color
						svf.videoFrame = KINECT::getPlayerColorFrame();
						svf.valid = true;
					}
					else{
						svf.valid = false;
					}
					svf.cam2World = matCfW.inv();

#if FULL_VIDEO_CAPTURE
					svf.fullVideoFrame = KINECT::getColorFrame();
#if VIDEO_DUMP_IMMEDIATELY
					dumpVideo();
#endif
#endif

					svf.depthFrame = KINECT::getDepthFrame();

					vidRecord.push_back(svf);
					removeUnnecessaryDepthFrames();
				}
			}else{
				hasCurrentSkeleton = false;
			}

			KINECT::setRefresh(true);

		}
		else {

			elapsed -= 1000/FRAMERATE;
			if(*pause == 0)
			{
				advanceFrame();

				if(vidDivisions.size() == 0)
				{
					if(currAnim >= vidRecord.size()){
						currAnim = 0;
						currAnimF = 0;
					}
				}else{
					if( currAnim >= vidRecord.size() || 
						(currentDivision < vidDivisions.size() && currAnim >= vidDivisions[currentDivision].second)){
						
						setStartFrame();

					}
				}
			}
		}
	}
	

};


std::string GhostGame::Save(std::string mapPath){

	std::string fileName = "GhostGame.xml";
	std::string filePath = mapPath + "/" + fileName;

	TiXmlDocument xmlDoc;

	TiXmlDeclaration * decl = new TiXmlDeclaration("1.0", "", "");
	xmlDoc.LinkEndChild(decl);

	TiXmlElement * rootNode = new TiXmlElement( Name());
	xmlDoc.LinkEndChild(rootNode);
	rootNode->SetDoubleAttribute("version", version);

	TiXmlElement * K2PNode = new TiXmlElement("Kinect_to_PTAM");
	rootNode->LinkEndChild(K2PNode);
	TooN::Matrix<4,4> K2P = KINECT::getPTAMfromKinect();
	for(int i=0;i<3;++i){
		for(int j=0;j<4;++j){
			TiXmlElement * matcell = new TiXmlElement("Mat_entry");
			K2PNode->LinkEndChild(matcell);
			matcell->SetDoubleAttribute("value", K2P(i,j));
		}
	}

	if(!getCameraMatrixScene().empty()){
		TiXmlElement * capCamMatNode = new TiXmlElement("CaptureCameraMatrix");
		rootNode->LinkEndChild(capCamMatNode);
		for(int r=0;r<3;++r){
			for(int c=0;c<4;++c){
				TiXmlElement * matcell = new TiXmlElement("MatEntry");
				capCamMatNode->LinkEndChild(matcell);
				matcell->SetDoubleAttribute("value", getCameraMatrixScene().at<float>(r,c));
			}
		}
	}
	

	TiXmlElement * vidNode = new TiXmlElement( "Video" );
	rootNode->LinkEndChild(vidNode);
	vidNode->SetAttribute("path", "/video/");

	SaveVideo(&vidRecord, getCameraMatrixScene(), mapPath + "/video/");

	TiXmlElement * vidSegmentsNode = new TiXmlElement("VideoSegments");
	rootNode->LinkEndChild(vidSegmentsNode);
	vidSegmentsNode->SetAttribute("numsegments", vidDivisions.size());

	for(int i=0;i<vidDivisions.size();++i){
		TiXmlElement * vidSegmentStartFrameNode = new TiXmlElement("StartFrame");
		vidSegmentsNode->LinkEndChild(vidSegmentStartFrameNode);

		vidSegmentStartFrameNode->SetAttribute("first", vidDivisions[i].first);
		vidSegmentStartFrameNode->SetAttribute("last", vidDivisions[i].second);
	}

	TiXmlElement * oniNode = new TiXmlElement("ONI");
	rootNode->LinkEndChild(oniNode);
	std::string oniPath = mapPath + "/params.oni";
	oniNode->SetAttribute("path", oniPath);

	KINECT::saveParams(oniPath);

	xmlDoc.SaveFile(filePath);

	return fileName;
};

void GhostGame::Load(std::string dataFileName){
	
	Reset();
	Init();

	TiXmlDocument xmlDoc;

	if(!xmlDoc.LoadFile(dataFileName)){
		std::cerr << "Failed to load " << dataFileName << " game file. Aborting.\n";
		return;
	}

	TiXmlHandle doc(&xmlDoc);
	TiXmlElement * elem;
	TiXmlHandle root(0);

	elem = doc.FirstChildElement().Element();
	if(!elem){
		std::cerr << "No root handle in XML file " << dataFileName << "\n";
		return;
	}

	string id(Name());
	double fileVersion = 0.0;
	elem->QueryDoubleAttribute("version", &fileVersion);

	/*if((id.compare(elem->Value())!=0) && (fileVersion != version)){
		std::cerr << "Invalid XML file. Need a version " << version << " " << id <<
			" XML file. Not a version " << fileVersion << " " << elem->Value() << " file.\n";
		return;
	}*/

	root = TiXmlHandle(elem);

	TiXmlHandle K2PNode = root.FirstChild("Kinect_to_PTAM");
	TooN::Matrix<4,4> K2P = TooN::Zeros;
	int i=0;
	for(TiXmlElement * elem = K2PNode.FirstChild().Element(); elem != NULL; elem = elem->NextSiblingElement()){
		elem->QueryDoubleAttribute("value", &K2P(i/4,i%4));
		++i;
	}
	K2P(3,3) = 1;
	KINECT::setPTAMfromKinect(K2P);
	skeleton.recalibPTAM(true);

	TiXmlHandle capCamMatNode = root.FirstChild("CaptureCameraMatrix");
	if(capCamMatNode.ToElement()){
		cv::Mat camMat(3,4,cv::DataType<float>::type);
		
		int i=0;
		for(TiXmlElement * elem = capCamMatNode.FirstChild().Element(); elem != NULL; elem = elem->NextSiblingElement()){
			elem->QueryFloatAttribute("value", &camMat.at<float>(i/4,i%4));

			++i;
		}

		setCameraMatrixTexture(camMat);
	}


	TiXmlHandle vidSegmentsNode = root.FirstChild("VideoSegments");

	if(vidSegmentsNode.ToElement()){
		vidDivisions.clear();

		if(fileVersion <= 1.5){

			int i=0, lastSegIn = 0;
			for(TiXmlElement * elem = vidSegmentsNode.FirstChild().Element(); elem != NULL; elem = elem->NextSiblingElement()){
				int segIn;
				elem->QueryIntAttribute("frame", &segIn);

				vidDivisions.push_back(pair<int,int>(lastSegIn, segIn));

				lastSegIn = segIn;

				++i;
			}
		}else{
			int i=0;
			for(TiXmlElement * elem = vidSegmentsNode.FirstChild().Element(); elem != NULL; elem = elem->NextSiblingElement()){
				int first, last;
				elem->QueryIntAttribute("first", &first);
				last = first + 1;
				elem->QueryIntAttribute("last", &last);

				vidDivisions.push_back(pair<int,int>(first, last));

				++i;
			}
		}
	}

	TiXmlHandle oniNode = root.FirstChild("ONI");
	std::string oniPath = oniNode.ToElement()->Attribute("path");

	KINECT::loadParams(oniPath);
	
	TiXmlHandle vidNode = root.FirstChild("Video");
	std::vector<bool> loadStatus;
	if (fileVersion < 1.5){

		loadStatus = LoadVideo(cv::Mat::eye(4, 4, cv::DataType<float>::type), KINECT::getPTAMfromKinect_mat(), &vidRecord, &wcSkeletons, vidNode.ToElement()->Attribute("path"), false);
	}
	else{
		int basePathLastIndex = dataFileName.find_last_of('/');
		std::string basePath = dataFileName.substr(0, basePathLastIndex);
		std::string videoPath = basePath + '/' + vidNode.ToElement()->Attribute("path");

		loadStatus = LoadVideo(cv::Mat::eye(4, 4, cv::DataType<float>::type), KINECT::getPTAMfromKinect_mat(), &vidRecord, &wcSkeletons, videoPath);

	}

	/*
	TiXmlHandle mapperNode = root.FirstChild("CoordinateMapper");
	std::string mapperPath = mapperNode.ToElement()->Attribute("path");
	ULONG mapNumBytes;
	int mnbtemp;
	mapperNode.ToElement()->QueryIntAttribute("numBytes", &mnbtemp);

	mapNumBytes = mnbtemp;

	std::cout << "loading in kinect parameters: " << mapNumBytes << " bytes\n";

	std::ifstream cmfile(mapperPath.c_str(), ios::binary|ios::in);
	
	char *dptr = (char*)malloc(mapNumBytes * sizeof(char));

	cmfile.read((char*)dptr, mapNumBytes);

	cmfile.close();

	HRESULT ok = KINECT::initMapper(&mapNumBytes, &dptr);

	std::cout << "kinect mapper status: ";
	if(FAILED(ok)) std::cout << "failed!\n";
	else std::cout << "ok!\n";
	
	free(dptr);*/

	//reworked. PROCESS

	if(fileVersion < 1.4)
	{

		int animNo=0, an2=-1;

		char buff[10];
		
	
		TiXmlHandle animationNode = root.FirstChild("Animation");

		int size;
		animationNode.ToElement()->QueryIntAttribute("size", &size);

		for(TiXmlElement * elem = animationNode.FirstChild().Element(); elem != NULL;
			elem = elem->NextSiblingElement()){
				Skeleton skel;

				++an2;

				if(!loadStatus[an2]) continue;

				int jtNo=0;
				for(TiXmlElement * elem2 = elem->FirstChildElement(); elem2 != NULL;
					elem2 = elem2->NextSiblingElement()){

					
						for(int k=0;k<4;++k){
							std::stringstream ss;
							ss << "point" << k;
							elem2->QueryFloatAttribute(ss.str().c_str(), &skel.points.at<float>(k, jtNo));
							//elem2->QueryFloatAttribute((std::string("point")+std::string(itoa(k, buff, 10))).c_str(), & skel.points.at<float>(k,jtNo));
						}

						float temp;
						elem2->QueryFloatAttribute("state", &temp);

						skel.states[jtNo] = (temp);

						++jtNo;
				}
				wcSkeletons[animNo] = skel;
				++animNo;

				good = true;
		}

		if(!good) return;
	}else{
		calculateWorldCoordinateSkeletons(KINECT::getPTAMfromKinect_mat(),  &vidRecord, &wcSkeletons);
	}



	if(vidRecord.size() <= 0) return;

	playing = true;

	//TiXmlHandle fleshNode = root.FirstChild("Flesh");


	//flesh.Load(fleshNode.ToElement()->Attribute("path"));


	//set vidrecord skeletons to animrecord

	//cylinder body

	cylinderBody.setVidRecord(&vidRecord);

	//build
	//buildCylinderBody(&vidRecord, &cylinderBody);

	cylinderBody.Load("map000000-custCB/");

	cylinderBody.calcLimbTransforms();

	calculateSkeletonOffsetPoints(vidRecord, wcSkeletons, cylinderBody);

	//Shit
	cylinderBody.radiusModifier = -(KINECT::getPTAMfromKinect_mat()).ptr<float>()[0];

	//SHIT
	std::string limbraryDirectory;

	{
		int lastSlash = dataFileName.find_last_of("/\\");
		limbraryDirectory = dataFileName.substr(0, lastSlash+1);
	}

	limbrary.Load(limbraryDirectory);

	//SHIT!!
	setCameraMatrixTexture(KINECT::loadCameraParameters());

#if ON_TABLET
	//clear depth images, we don't need them anymore, we shouldn't save on tablet

	for(auto it = vidRecord.begin(); it != vidRecord.end(); ++it){
		it->depthFrame = cv::Mat();
	}

#endif
	currentDivision = 0;

	setStartFrame();
};

SkeleVideoFrame * GhostGame::getBestSkeletonScore(bool removeElse){
	SkeleVideoFrame * bestF = NULL;
	int best = 0;
	for(auto it=vidRecord.begin(); it!=vidRecord.end(); ++it){
		if(it->depthFrame.data != NULL || it->videoFrame.mat.data != NULL) continue;
		if(it->kinectPoints.initSkeleton == 0){
			int score = KINECT::initSkeletonScore(it->kinectPoints);
			it->kinectPoints.initSkeleton =  score ;
		}
		if(best < it->kinectPoints.initSkeleton){

			if(removeElse && bestF != NULL && bestF->depthFrame.data != NULL){
				bestF->depthFrame = cv::Mat();
			}

			best = it->kinectPoints.initSkeleton;
			bestF = &*it;
		}else if(removeElse){
			it->depthFrame = cv::Mat();
		}
	}
	return bestF;
};

void GhostGame::removeUnnecessaryDepthFrames(){
	getBestSkeletonScore(true);
}

void GhostGame::refreshVidDirectory(){
	time_t t = time(0);
	tm now;
	localtime_s(&now, &t);
	char buf[100];

	sprintf(buf, "videodump-%d-%d-%d-%02d%02d", now.tm_year+1900, now.tm_mon+1, now.tm_mday, now.tm_hour, now.tm_sec);

	vidpath = std::string(buf);
};

void GhostGame::dumpVideo(){

	CreateDirectory(vidpath.c_str(), NULL);

	int i=0;
	char buff[10];
	for(auto it=vidRecord.begin(); it != vidRecord.end(); ++it){
		++i;
		if(it->fullVideoFrame.data != NULL)
		{
			std::stringstream ss;
			ss << vidpath << "/" << i << ".png";
			cv::imwrite(ss.str(), it->fullVideoFrame);

			it->fullVideoFrame = cv::Mat();
		}
#if DUMP_DEPTH
		if(it->depthFrame.data != NULL)
		{
			cv::FileStorage fs(vidpath+"/"+itoa(i, buff, 10) + ".yml", cv::FileStorage::WRITE);
			fs << "depth" << it->depthFrame;
			fs.release();
		}
#endif

	}
};


void GhostGame::GUICommandCallBack(void *ptr, string sCommand, string sParams){
	GhostGame * gg = static_cast<GhostGame*>(ptr);
	if(sCommand=="GH_NextSection"){
		gg->currentDivision+=1;
		if(gg->currentDivision >= gg->vidDivisions.size()){
			gg->currentDivision = gg->vidDivisions.size()-1;
		}

		gg->setStartFrame();

	}
	else if(sCommand=="GH_PrevSection"){
		gg->currentDivision-=1;
		if(gg->currentDivision < 0){
			gg->currentDivision = 0;
		}

		gg->setStartFrame();
	}
	else if(sCommand=="GH_StartFrame"){
		gg->setStartFrame();
	}
};

}

#endif