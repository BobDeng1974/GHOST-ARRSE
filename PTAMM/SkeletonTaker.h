#ifndef __SKELETONTAKER
#define __SKELETONTAKER

#include <vector>
#include <array>

#include "TooN\TooN.h"
#include "TooN\se3.h"
#include "opencv2\opencv.hpp"

#include "MapPoint.h"

#include "definitions.h"

#define NUMCLUSTERS 10
#define VALID_THRESH 10

namespace KINECT{
	class SkeletonTaker{

	private:
		bool ptamcalib;
		
	public:
		SkeletonTaker();
		void Reset();

		void calibPTAM(std::vector<PTAMM::MapPoint*>, TooN::SE3<>);
		void recalibPTAM(bool calib = false);

		void ProjectGrid(TooN::SE3<>);
		std::vector<TooN::Vector<4>> gridpts2;
		std::vector<TooN::Vector<4>> gridpts;

		void SaveVideo();
		void LoadVideo(TooN::SE3<>);
		
		//void setAnimRecord(std::vector<Skeleton> *);
	};

}

#endif