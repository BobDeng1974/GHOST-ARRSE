#if 0

#ifndef _GHOST_GAME
#define _GHOST_GAME

#include "Game.h"
#include "GLWindow2.h"
#include "OpenNIStarter.h"
#include "NiTE.h"
#include "ATANCamera.h"
#include "SkeletonTaker.h"

#include <string>
#include <array>

#include "OpenGL.h"
#include "CylinderBody.h"
#include "Limbrary.h"

namespace GHOST{
	
#define FRAMERATE 10.0
using namespace PTAMM;
using std::pair;
using std::vector;

class GhostGame:public Game{
public:
	GhostGame();
	~GhostGame();

	void Draw3D( const GLWindow2 &glWindow, Map &map, TooN::SE3<> se3CfromW);
    void Draw2D( const GLWindow2 &glWindow, Map &map);
    
    void Reset();
    void Init();
    void HandleKeyPress( std::string sKey );
	void HandleClick(Vector<2> v2VidCoords, Vector<2> v2UFB, Vector<3> v3RayDirnW, 
                             Vector<2> v2Plane, int nButton);
    void Advance();
    
    std::string Save( std::string sMapPath );
    void Load( std::string sDataFileName );

	std::string vidpath;

private:
	static const int mnWindowWidth = 640;
	static const int mnWindowHeight = 480;

    Map * mpMap;                                  // The associated map
    SE3<> mse3CfW;                                // The camera postion
    cv::Mat matCfW;                                // The camera (openCV)
	
	CylinderBody cylinderBody;
	Limbrary limbrary;

	std::array<Skeleton, MAXUSERS> currSkeleton;
	vector<SkeleVideoFrame> vidRecord;
	vector<Skeleton> wcSkeletons;
	vector<pair<int,int>> vidDivisions; //the last frames of the current section
	int currentDivision; //to get the first frame: vidDivisions[currentDivision-1]+1 (unless currentDivision=0 in which case 0)
	//std::vector<Skeleton> animRecord;
	Skeleton* drawSkeleton;
	int currAnim;
	float currAnimF;
	int chosenSkeleton;
	
	bool good;
	bool mapLoaded;
	bool recording;
	bool playing;
	bool hasCurrentSkeleton;
	bool invprojcalc;

	GVars3::gvar3<int> gridproj;
	GVars3::gvar3<int> draw2d;
	GVars3::gvar3<int> drawDots;
	GVars3::gvar3<int> pause;
	GVars3::gvar3<int> cylinders;
	GVars3::gvar3<int> usePose;
	GVars3::gvar3<int> drawOnPic;
	GVars3::gvar3<int> cyl_individualRender;
	GVars3::gvar3<int> cyl_colorPixels;
	GVars3::gvar3<int> cyl_drawCylinders;
	GVars3::gvar3<int> cyl_norm;
	GVars3::gvar3<int> cyl_norot;
	GVars3::gvar3<int> cyl_indivLimbs;
	GVars3::gvar3<int> cyl_shittyOption;
	GVars3::gvar3<int> writecmp;

	float _capx;

	int chosenBest;
	std::string dispstring;


	SYSTEMTIME prevtime;
	int elapsed;

	void ProjectGrid();
	void camWorld_mat();

	const double version;
	
	ATANCamera camera;
	Vector<NUMTRACKERCAMPARAMETERS> camParams;

	GLUquadricObj * quadric;

	KINECT::SkeletonTaker skeleton;

	//note: seems inefficient; what uses this?
	TooN::Vector<2> project(TooN::Vector<4>, float width, float height);
	TooN::Vector<2> project0(TooN::Vector<4>);
	void processVideo();

	
	//openni::DepthPixel* _depthdata;
	
	void setStartFrame();
	SkeleVideoFrame * getBestSkeletonScore(bool removeElse = false);
	void removeUnnecessaryDepthFrames();
	void dumpVideo();
	void advanceFrame();
	void refreshVidDirectory();

	static void GUICommandCallBack(void *ptr, string sCommand, string sParams);
};
}

#endif

#endif