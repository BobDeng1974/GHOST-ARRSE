#include "ghostsettings.h"

#if GHOST_INPUT == INPUT_OPENNI || GHOST_INPUT == INPUT_KINECT2

// Copyright 2008 Isis Innovation Limited
// This VideoSource for Win32 uses CMU's 1394 driver
// available at 
// http://www.cs.cmu.edu/~iwan/1394/

#define CT_KINECT 5
#define CT_FILE 6

#define WIN32_LEAN_AND_MEAN
#include "VideoSource.h"
#include <Windows.h>
#include <cvd\utility.h>

#include "KinectManager.h"

#include <opencv2\opencv.hpp>

#include "gvars3\instances.h"

using namespace CVD;
using namespace std;
using namespace GVars3;

//videosource should also be able to load from file
//can we control this thru gvars?

void GUICommandCallback(void *ptr, string sCommand, string sParams){
	VideoSource * vs = static_cast<VideoSource*>(ptr);
	if(sCommand == "GH_CaptureReset"){
		vs->setFrame(1);
	}
}

VideoSource::VideoSource()//: cam(1)
{
	KINECT::init();
	mirSize.y = CAPTURE_SIZE_Y;
	mirSize.x = CAPTURE_SIZE_X;

	GV2.Register(captureType, "GH_CaptureType", CT_KINECT, SILENT);
	GV2.Register(vidPath, "GH_VidPath", "videoin/", SILENT);
	GV2.Register(dumpFeed, "GH_DumpFeed", 0, SILENT);

	GUI.RegisterCommand("GH_CaptureReset", GUICommandCallback, this);

	m_buffer = new unsigned char[CAPTURE_SIZE_X*CAPTURE_SIZE_Y * 3];
	
	currFrame = 1;
	currOut = 1;
	
	GetLocalTime(&prevtime);
	elapsed = 0;

	refreshVidDirectory();
};

VideoSource::~VideoSource()
{
	delete[] m_buffer;
	KINECT::release();
};

void VideoSource::setFrame(int f){
	currFrame = f;
};

//copied from GhostGame.cpp
void VideoSource::refreshVidDirectory(){
	time_t t = time(0);
    struct tm now;
	localtime_s(&now, &t);
	char buf[100];

	sprintf_s(buf, "kinectdump-%d-%d-%d-%02d%02d", now.tm_year+1900, now.tm_mon+1, now.tm_mday, now.tm_hour, now.tm_sec);

	dumpPath = std::string(buf);
};

void VideoSource::GetAndFillFrameBWandRGB(Image<CVD::byte> &imBW, Image<CVD::Rgb<CVD::byte> > &imRGB)
{
	
	SYSTEMTIME time;
	GetLocalTime(&time);

	int temp = time.wMilliseconds - prevtime.wMilliseconds + 1000*(time.wSecond - prevtime.wSecond);

	if(temp < 0) temp = 0;

	elapsed += temp;
	prevtime = time;

	KINECT::updateFrames();

	while(elapsed >= (1000/FRAMERATE))
	{
		++currFrame;
		elapsed -= (1000/FRAMERATE);
	}

	imRGB.resize(mirSize);
	imBW.resize(mirSize);

	
	if(*captureType == CT_KINECT){

		//max stuff
		//cv::Mat input;
		//cam >> input;

		//mirSize.x = input.cols;
		//mirSize.y = input.rows;
		

		//imRGB.resize(mirSize);
		//imBW.resize(mirSize);

		//m_buffer = input.ptr();

		cv::Mat video = KINECT::getColorFrame();

		if (video.empty()) return;

		//getDepthData(m_buffer);
		//getRandomData(m_buffer);

		unsigned char* pImage = video.ptr<unsigned char>();

		for (int y = 0; y<mirSize.y; y++) {
			for (int x = 0; x<mirSize.x; x++) {
				int x_ = mirSize.x - x - 1;

				imRGB[y][x_].blue = *pImage;
				pImage++;

				imRGB[y][x_].green = *pImage;
				imBW[y][x_] = *pImage;
				pImage++;

				imRGB[y][x_].red = *pImage;
				pImage++;

				if (video.channels() == 4)
					++pImage;

			}
		}

		if(*dumpFeed){
			
			CreateDirectoryA(dumpPath.c_str(), NULL);

			std::stringstream ssPath;
			ssPath << dumpPath << "/" << currOut << ".png";

			++currOut;

			cv::imwrite(ssPath.str(), video);
		}

	}else if(*captureType == CT_FILE){

		std::stringstream ssPath;
		ssPath << *vidPath << currFrame << ".png";

		cv::Mat im_ = cv::imread(ssPath.str(), CV_LOAD_IMAGE_COLOR);

		if(im_.empty()){
			currFrame=1;
			return;
		}

		cv::Mat im(mirSize.y, mirSize.x, CV_8UC3);

		cv::resize(im_, im, im.size());

		unsigned char* pImage = im.ptr();

		for (int y = 0; y<mirSize.y; y++) {
			for (int x = 0; x<mirSize.x; x++) {
				int x_ = mirSize.x - x - 1;

				imRGB[y][x_].blue = *pImage;
				pImage++;

				imRGB[y][x_].green = *pImage;
				imBW[y][x_] = *pImage;
				pImage++;

				imRGB[y][x_].red = *pImage;
				pImage++;

			}
		}
	}
}

ImageRef VideoSource::Size()
{
	return mirSize;
}
#endif
#if GHOST_INPUT == INPUT_VI


// This VideoSource for Win32 uses EWCLIB
//
// EWCLIB ver.1.2
// http://www.geocities.jp/in_subaru/ewclib/index.html

#define WIN32_LEAN_AND_MEAN
#include <Ole2.h>
#include <Windows.h>


#include "VideoSource.h"
#include <cvd/utility.h>

#include "videoInput.h"

using namespace CVD;
using namespace std;

#define CAPTURE_SIZE_X	640
#define CAPTURE_SIZE_Y	480
#define FPS				30


bool useKinect = false;
bool VIinit = false;
videoInput *VI;
int deviceID;
/*
bool initKinect() {
	/
	bool a = KINECT::initKinect();

	rgbStream = KINECT::getRGB();
	sensor = KINECT::getSensor();
	


	return a;
}*/

void StopEvent(int deviceID, void *userData)
{
	videoInput *VI = &videoInput::getInstance();

	VI->closeDevice(deviceID);
}

bool initVideoInput(){
	if (!VIinit){
		VI = &videoInput::getInstance();
		int i = VI->listDevices();

		if (i < 1) return false;
		deviceID = i - 1;
		if (!VI->setupDevice(deviceID, CAPTURE_SIZE_X, CAPTURE_SIZE_Y, 60)) return false;
		VI->setEmergencyStopEvent(deviceID, NULL, StopEvent);

		VIinit = true;
			
	}

	return VIinit;
}



VideoSource::VideoSource()
{
	m_buffer = new unsigned char[CAPTURE_SIZE_X*CAPTURE_SIZE_Y*4];
	m_buffer2 = new unsigned char[CAPTURE_SIZE_X*CAPTURE_SIZE_Y*4];

	mirSize.x = CAPTURE_SIZE_X;
	mirSize.y = CAPTURE_SIZE_Y;

	refreshVidDirectory();

	CreateDirectory(dumpPath.c_str(), NULL);
};

VideoSource::~VideoSource()
{
    delete[] m_buffer;
    delete[] m_buffer2;
}


void VideoSource::refreshVidDirectory(){
	time_t t = time(0);
	struct tm now;
	localtime_s(&now, &t);
	char buf[100];

	sprintf_s(buf, "kinectdump-%d-%d-%d-%02d%02d", now.tm_year + 1900, now.tm_mon + 1, now.tm_mday, now.tm_hour, now.tm_sec);

	dumpPath = std::string(buf);
};


void VideoSource::GetAndFillFrameBWandRGB(Image<CVD::byte> &imBW, Image<CVD::Rgb<CVD::byte> > &imRGB)
{
	if (false && useKinect){
		/*
		KINECT::getKinectData(m_buffer);

		unsigned char* pImage = m_buffer;
		unsigned char* pImage2;

		BasicImage<CVD::byte> imCaptured(pImage, mirSize);
		imRGB.resize(mirSize);
		imBW.resize(mirSize);

		if (depth){
			//KINECT::getKinectData_color_player(m_buffer2, 1);
			KINECT::getKinectData_depth(m_buffer2);
			pImage2 = m_buffer2;
		}

		for (int y = 0; y < mirSize.y; y++) {
			for (int x = 0; x < mirSize.x; x++) {
				int x_ = mirSize.x - x - 1;

				if (depth)
				{
					imRGB[y][x_].blue = *pImage2++;
					pImage++;

					imRGB[y][x_].green = *pImage2++;
					imBW[y][x_] = *pImage;
					pImage++;

					imRGB[y][x_].red = *pImage2++;
					pImage++;

					pImage++;
					++pImage2;
				}
				else{
					imRGB[y][x_].blue = *pImage;
					pImage++;

					imRGB[y][x_].green = *pImage;
					imBW[y][x_] = *pImage;
					pImage++;

					imRGB[y][x_].red = *pImage;
					pImage++;

					pImage++;
				}
			}
		}*/
	}
	else{
		if (!initVideoInput()){
			std::cerr << "VideoInput not initialized!\n";
			return;
		}


		imRGB.resize(mirSize);
		imBW.resize(mirSize);

		if (!VI->isFrameNew(deviceID)) return;

		VI->getPixels(deviceID, m_buffer);

		unsigned char* pImage = m_buffer;
		for (int y = 0; y < mirSize.y; y++) {
			for (int x = 0; x < mirSize.x; x++) {
				int x_ = x;//mirSize.x - x - 1;

				
				imRGB[y][x_].blue = *pImage;
				pImage++;

				imRGB[y][x_].green = *pImage;
				imBW[y][x_] = *pImage;
				pImage++;

				imRGB[y][x_].red = *pImage;
				pImage++;

			}
		}


		static GVars3::gvar3<int> gv_record_input("CamRecord", 0, GVars3::SILENT);

		if (*gv_record_input){
			static int save_frame_counter = 0;
			cv::Mat rgb(mirSize.y, mirSize.x, CV_8UC3, m_buffer);
			cv::imwrite(dumpPath + "/" + to_string(save_frame_counter) + ".png", rgb);
			++save_frame_counter;
		}
	}

}

ImageRef VideoSource::Size()
{
	return mirSize;
}


#endif
#if GHOST_INPUT == INPUT_WEBCAM

#define WIN32_LEAN_AND_MEAN
#include "VideoSource.h"
#include <Windows.h>
#include <cvd\utility.h>
#include <opencv2\opencv.hpp>

#include "gvars3\instances.h"

using namespace CVD;
using namespace std;
using namespace GVars3;

//videosource should also be able to load from file
//can we control this thru gvars?

cv::VideoCapture cam;

void GUICommandCallback(void *ptr, string sCommand, string sParams){
	VideoSource * vs = static_cast<VideoSource*>(ptr);
	if (sCommand == "GH_CaptureReset"){
		vs->setFrame(1);
	}
}

VideoSource::VideoSource()//: cam(1)
{
	cam.open(0);

	cv::Mat frame;
	cam >> frame;

	mirSize.x = frame.cols;
	mirSize.y = frame.rows;

	currFrame = 1;
	currOut = 1;

	GetLocalTime(&prevtime);
	elapsed = 0;

	refreshVidDirectory();
};

VideoSource::~VideoSource()
{
};

void VideoSource::setFrame(int f){
	currFrame = f;
};

//copied from GhostGame.cpp
void VideoSource::refreshVidDirectory(){
	time_t t = time(0);
	struct tm now;
	localtime_s(&now, &t);
	char buf[100];

	sprintf_s(buf, "kinectdump-%d-%d-%d-%02d%02d", now.tm_year + 1900, now.tm_mon + 1, now.tm_mday, now.tm_hour, now.tm_sec);

	dumpPath = std::string(buf);
};

void VideoSource::GetAndFillFrameBWandRGB(Image<CVD::byte> &imBW, Image<CVD::Rgb<CVD::byte> > &imRGB)
{

	if (!cam.isOpened()){
		cam.open(0);
	}

	SYSTEMTIME time;
	GetLocalTime(&time);

	int temp = time.wMilliseconds - prevtime.wMilliseconds + 1000 * (time.wSecond - prevtime.wSecond);

	if (temp < 0) temp = 0;

	elapsed += temp;
	prevtime = time;

	while (elapsed >= (1000 / FRAMERATE))
	{
		++currFrame;
		elapsed -= (1000 / FRAMERATE);
	}

	cv::Mat rgb;

	cam >> rgb;

	mirSize.x = rgb.cols;
	mirSize.y = rgb.rows;

	imRGB.resize(mirSize);
	imBW.resize(mirSize);

	for (int y = 0; y<mirSize.y; y++) {
		for (int x = 0; x<mirSize.x; x++) {
			int x_ = mirSize.x - x - 1;

			imRGB[y][x_].blue = rgb.ptr<cv::Vec3b>(y)[x_](0);
			imRGB[y][x_].green = rgb.ptr<cv::Vec3b>(y)[x_](1);
			imBW[y][x_] = rgb.ptr<cv::Vec3b>(y)[x_](1);
			imRGB[y][x_].red = rgb.ptr<cv::Vec3b>(y)[x_](2);

		}
	}

}

ImageRef VideoSource::Size()
{
	return mirSize;
}

#endif
#if GHOST_INPUT == INPUT_KINECT

#define WIN32_LEAN_AND_MEAN
#include <Windows.h>
#include <Ole2.h>

#include "VideoSource.h"
#include <Windows.h>
#include <cvd\utility.h>
#include <opencv2\opencv.hpp>

#include "gvars3\instances.h"


#include "KinectManager.h"

using namespace CVD;
using namespace std;
using namespace GVars3;

//videosource should also be able to load from file
//can we control this thru gvars?

KinectManager * kinect_manager;

void GUICommandCallback(void *ptr, string sCommand, string sParams){
	VideoSource * vs = static_cast<VideoSource*>(ptr);
	if (sCommand == "GH_CaptureReset"){
		vs->setFrame(1);
	}
}

VideoSource::VideoSource()//: cam(1)
{
	kinect_manager = KinectManager::GetKinectManager();

	if (!kinect_manager->IsOpened()){
		kinect_manager->InitializeDefaultSensor();
	}
	
	mirSize.x = KINECT_CAPTURE_SIZE_X;
	mirSize.y = KINECT_CAPTURE_SIZE_Y;

	currFrame = 1;
	currOut = 1;

	GetLocalTime(&prevtime);
	elapsed = 0;

	refreshVidDirectory();

	CreateDirectory(dumpPath.c_str(), NULL);
};

VideoSource::~VideoSource()
{
};

void VideoSource::setFrame(int f){
	currFrame = f;
};

//copied from GhostGame.cpp
void VideoSource::refreshVidDirectory(){
	time_t t = time(0);
	struct tm now;
	localtime_s(&now, &t);
	char buf[100];
	
	sprintf_s(buf, "kinectdump-%d-%d-%d-%02d%02d", now.tm_year + 1900, now.tm_mon + 1, now.tm_mday, now.tm_hour, now.tm_sec);

	dumpPath = std::string(buf);
};

void VideoSource::GetAndFillFrameBWandRGB(Image<CVD::byte> &imBW, Image<CVD::Rgb<CVD::byte> > &imRGB)
{
	if (!kinect_manager->IsOpened()){
		kinect_manager->InitializeDefaultSensor();
	}
	kinect_manager->Update(Update::Color);

	SYSTEMTIME time;
	GetLocalTime(&time);

	int temp = time.wMilliseconds - prevtime.wMilliseconds + 1000 * (time.wSecond - prevtime.wSecond);

	if (temp < 0) temp = 0;

	elapsed += temp;
	prevtime = time;

	while (elapsed >= (1000 / FRAMERATE))
	{
		++currFrame;
		elapsed -= (1000 / FRAMERATE);
	}

	unsigned char * rgba_data = kinect_manager->GetColorRGBX();

	cv::Mat rgb(KINECT_CAPTURE_SIZE_Y, KINECT_CAPTURE_SIZE_X, CV_8UC4, rgba_data);

	mirSize.x = rgb.cols;
	mirSize.y = rgb.rows;

	imRGB.resize(mirSize);
	imBW.resize(mirSize);

	for (int y = 0; y<mirSize.y; y++) {
		for (int x = 0; x<mirSize.x; x++) {
			//int x_ = mirSize.x - x - 1;

			imRGB[y][x].blue = rgb.ptr<cv::Vec4b>(y)[x](0);
			imRGB[y][x].green = rgb.ptr<cv::Vec4b>(y)[x](1);
			imBW[y][x] = rgb.ptr<cv::Vec4b>(y)[x](1);
			imRGB[y][x].red = rgb.ptr<cv::Vec4b>(y)[x](2);

			rgb.ptr<cv::Vec4b>(y)[x](3) = 0xff;
		}
	}

	static GVars3::gvar3<int> gv_record_input("CamRecord", 0, GVars3::SILENT);

	if (*gv_record_input){
		static int save_frame_counter = 0;
		cv::imwrite(dumpPath + "/" + to_string(save_frame_counter) + ".png", rgb);
		++save_frame_counter;
	}
}

ImageRef VideoSource::Size()
{
	return mirSize;
}

#endif
#if GHOST_INPUT == INPUT_FILE


#define WIN32_LEAN_AND_MEAN
#include "VideoSource.h"
#include <Windows.h>
#include <Shlwapi.h>
#include <tchar.h>
#include <cvd\utility.h>
#include <opencv2\opencv.hpp>

#include "gvars3\instances.h"

using namespace CVD;
using namespace std;
using namespace GVars3;

std::string files_path = "";
std::vector<std::string> filenames;

void GUICommandCallback(void *ptr, string sCommand, string sParams){
	VideoSource * vs = static_cast<VideoSource*>(ptr);
	if (sCommand == "GH_CaptureReset"){
		vs->setFrame(1);
	}
	else if (sCommand == "GH_CapturePause"){
		vs->togglePause();
	}
}

int num_length(std::string a, int num_start){
	int i = num_start;
	while (true){
		if ('0' <= a[i] && '9' >= a[i]){
			i++;
		}
		else{
			break;
		}
	}
	return i - num_start;
}

bool strcmp_logical(std::string a, std::string b){
	int i = 0, j = 0;
	for (; i < a.size() && j < b.size(); ++i, ++j){
		//check if both a[i] and b[j] are numbers
		int a_num_ln = num_length(a, i);
		int b_num_ln = num_length(b, j);

		if (a_num_ln > 0 && b_num_ln > 0){
			int a_num = atoi(a.substr(i, a_num_ln).c_str());
			int b_num = atoi(b.substr(j, b_num_ln).c_str());

			if (a_num > b_num){
				return false;
			}
			else if (a_num < b_num){
				return true;
			}
		}
		else{
			if (a[i] > b[i]){
				return false;
			}
			else if (a[i] < b[i]){
				return true;
			}
		}

		i += std::max(0, a_num_ln - 1);
		j += std::max(0, b_num_ln - 1);
	}

	if (i < a.size()){
		return true;
	}
	else if (j < b.size()){
		return false;
	}
	
	return i < j;
}

VideoSource::VideoSource()//: cam(1)
{

	GVars3::GUI.LoadFile("filesource.cfg");
	static GVars3::gvar3<std::string> gv_filesource("FileSource", "", GVars3::SILENT);

	{
		WIN32_FIND_DATA ffd;
		HANDLE find;

		find = FindFirstFile((*gv_filesource + "\\*").c_str(), &ffd);

		if (INVALID_HANDLE_VALUE == find)
		{
			std::cout << "File video source: no files found\n";
		}


		do{
			if (!(ffd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY)){
				//file found; analyze it and file it correctly

				std::string filename(ffd.cFileName);
				filenames.push_back(filename);
			}
		} while (FindNextFile(find, &ffd) != 0);

		std::sort(filenames.begin(), filenames.end(), strcmp_logical);
	}
	
	if (!filenames.empty()){
		files_path = *gv_filesource + "\\";
		cv::Mat f1 = cv::imread(files_path + filenames[0]);

		mirSize.x = f1.cols;
		mirSize.y = f1.rows;
	}

	currFrame = 1;
	currOut = 1;

	GetLocalTime(&prevtime);
	elapsed = 0;

	refreshVidDirectory();

	GUI.RegisterCommand("GH_CaptureReset", GUICommandCallback, this);
	GUI.RegisterCommand("GH_CapturePause", GUICommandCallback, this);
	play = true;
};

VideoSource::~VideoSource()
{
};

void VideoSource::setFrame(int f){
	currFrame = f;
};

void VideoSource::togglePause(){
	play = !play;
}

//copied from GhostGame.cpp
void VideoSource::refreshVidDirectory(){
	time_t t = time(0);
	struct tm now;
	localtime_s(&now, &t);
	char buf[100];

	sprintf_s(buf, "kinectdump-%d-%d-%d-%02d%02d", now.tm_year + 1900, now.tm_mon + 1, now.tm_mday, now.tm_hour, now.tm_sec);

	dumpPath = std::string(buf);
};

void VideoSource::GetAndFillFrameBWandRGB(Image<CVD::byte> &imBW, Image<CVD::Rgb<CVD::byte> > &imRGB)
{

	SYSTEMTIME time;
	GetLocalTime(&time);

	int temp = time.wMilliseconds - prevtime.wMilliseconds + 1000 * (time.wSecond - prevtime.wSecond);

	if (temp < 0) temp = 0;

	if (play)
		elapsed += temp;

	prevtime = time;

	while (elapsed >= (1000 / FRAMERATE))
	{
		++currFrame;
		elapsed -= (1000 / FRAMERATE);
	}

	cv::Mat rgb = cv::imread(files_path + filenames[currFrame%filenames.size()]);

	mirSize.x = rgb.cols;
	mirSize.y = rgb.rows;

	imRGB.resize(mirSize);
	imBW.resize(mirSize);

	for (int y = 0; y<mirSize.y; y++) {
		for (int x = 0; x<mirSize.x; x++) {
			int x_ = mirSize.x - x - 1;

			imRGB[y][x_].blue = rgb.ptr<cv::Vec3b>(y)[x_](0);
			imRGB[y][x_].green = rgb.ptr<cv::Vec3b>(y)[x_](1);
			imBW[y][x_] = rgb.ptr<cv::Vec3b>(y)[x_](1);
			imRGB[y][x_].red = rgb.ptr<cv::Vec3b>(y)[x_](2);

		}
	}

}


ImageRef VideoSource::Size()
{
	return mirSize;
}

#endif