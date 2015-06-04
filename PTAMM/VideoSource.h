#if 0

// This VideoSource for Win32 uses EWCLIB
//
// EWCLIB ver.1.2
// http://www.geocities.jp/in_subaru/ewclib/index.html

#include <cvd/image.h>
#include <cvd/byte.h>
#include <cvd/rgb.h>

struct VideoSourceData;

class VideoSource
{
public:
	VideoSource();
	~VideoSource();
	void GetAndFillFrameBWandRGB(CVD::Image<CVD::byte> &imBW, CVD::Image<CVD::Rgb<CVD::byte> > &imRGB);
	CVD::ImageRef Size();

private:
	unsigned char *m_buffer;
	CVD::ImageRef mirSize;
};

#else

#include <Windows.h>

#include <cvd/image.h>
#include <cvd/byte.h>
#include <cvd/rgb.h>

#include <gvars3\instances.h>

#include <opencv2\opencv.hpp>

//framerate for file input
#define FRAMERATE 10.0

struct VideoSourceData;

class VideoSource
{
public:
	VideoSource();
	~VideoSource();
	void GetAndFillFrameBWandRGB(CVD::Image<CVD::byte> &imBW, CVD::Image<CVD::Rgb<CVD::byte> > &imRGB);
	CVD::ImageRef Size();

	void setFrame(int);

private:
	void refreshVidDirectory();

	unsigned char *m_buffer;
	unsigned char * m_buffer2;
	CVD::ImageRef mirSize;
	
	//where to dump the video/depth
	std::string dumpPath;
	int currOut;

	GVars3::gvar3<int> captureType;
	GVars3::gvar3<std::string> vidPath;
	GVars3::gvar3<int> dumpFeed;

	//current frame for file loading
	int currFrame; 

	
	int elapsed;
	SYSTEMTIME prevtime;

	//temp
	//cv::VideoCapture cam;
};
#endif