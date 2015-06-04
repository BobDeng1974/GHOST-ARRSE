#if 0
#include "PTAM2Kinect.h"
#include "KinectManager.h"
#include "ghostutil.h"
#include "ghostcam.h"
#include "cvutil.h"

#include <TooN\TooN.h>

//openNI
#include "util.h"

#ifndef isinf
#include <math.h>
#define isinf(x) (!_finite(x))
#endif

namespace KINECT{

	template<int T>
	cv::Vec<double, 3> TooN2openCV3(TooN::Vector<T> v){
		cv::Vec<double, 3> ret;
		for (int i = 0; i < 3; ++i){
			ret[i] = v[i];
		}
		return ret;
	}

	template<int R, int C>
	cv::Mat TooN2openCV(TooN::Matrix<R, C> m){
		cv::Mat ret(R, C, cv::DataType<float>::type);
		for (int r = 0; r < R; ++r){
			for (int c = 0; c < C; ++c){
				ret.at<float>(r, c) = m[r][c];
			}
		}
		return ret;
	}


	TooN::Matrix<4, 4> PTAMfromKinect;

	cv::Mat getPTAMfromKinect_mat(){
		return TooN2openCV(PTAMfromKinect);
	}

	void calcPTAMfromKinect(cv::Mat& mvPTAM3DPoints, DepthXY * dDepthMap){

		std::vector<cv::Vec3f> vPTAM3DPoints;
		std::vector<cv::Vec3f> vKinect3DPoints;

		float minDist = 10000;

		cv::Vec3f pPTAMMean(0, 0, 0);
		cv::Vec3f pKinectMean(0, 0, 0);

		cv::Mat mvPTAM2DPoints;

		{
			PTAMM::ATANCamera cam("Camera");

			TooN::Vector<5> camParams = cam.GetParams();

			cv::Mat mCameraMatrix = cv::Mat::zeros(4, 4, CV_32F);

			mCameraMatrix.at<float>(0, 0) = camParams[0] * CAPTURE_SIZE_X;
			mCameraMatrix.at<float>(1, 1) = camParams[1] * CAPTURE_SIZE_Y;
			mCameraMatrix.at<float>(0, 2) = camParams[2] * CAPTURE_SIZE_X - 0.5;
			mCameraMatrix.at<float>(1, 2) = camParams[3] * CAPTURE_SIZE_Y - 0.5;
			mCameraMatrix.at<float>(2, 2) = 1;
			mCameraMatrix.at<float>(3, 3) = 1;

			mvPTAM2DPoints = mCameraMatrix * mvPTAM3DPoints;
			cv::divide(mvPTAM2DPoints.row(0), mvPTAM2DPoints.row(2), mvPTAM2DPoints.row(0));
			cv::divide(mvPTAM2DPoints.row(1), mvPTAM2DPoints.row(2), mvPTAM2DPoints.row(1));
		}

		cv::Mat debugImg = KINECT::getColorFrame();
		cv::Mat debugDepth = KINECT::getDepthFrame();

		cv::Mat debugDepthGrey(debugDepth.rows, debugDepth.cols, CV_8UC1);
		for (int i = 0; i < debugDepth.rows*debugDepth.cols; ++i){
			debugDepthGrey.ptr<unsigned char>()[i] = debugDepth.ptr<unsigned short>()[i]%256;
		}

		for (int i = 0; i < mvPTAM2DPoints.cols; ++i){
			int x = CAPTURE_SIZE_X - mvPTAM2DPoints.ptr<float>(0)[i] -1;
			int y = mvPTAM2DPoints.ptr<float>(1)[i];

			cv::circle(debugImg, cv::Point(x, y), 1, cv::Scalar(255, 0, 0, 255));
			cv::circle(debugDepthGrey, cv::Point(x, y), 2, cv::Scalar(255));

			if (CLAMP_SIZE(x, y, CAPTURE_SIZE_X, CAPTURE_SIZE_Y)){
				DepthXY dDepthPoint = dDepthMap[x + y*CAPTURE_SIZE_X];
				cv::Vec3f pKinectPoint = KINECT::mapDepthToSkeletonPoint(dDepthPoint);


				if (isinf(pKinectPoint(0))){
					continue;
				}

				cv::Vec3f pPTAMPoint = mat_to_vec3(mvPTAM3DPoints.col(i));

				vPTAM3DPoints.push_back(pPTAMPoint);
				vKinect3DPoints.push_back(pKinectPoint);

				pPTAMMean += pPTAMPoint;
				pKinectMean += pKinectPoint;
			}
		}


		int M = vPTAM3DPoints.size();

		//average
		pPTAMMean /= M;
		pKinectMean /= M;

		//std::cout << "p-mean " <<  pmean << std::endl << "q-mean " << qmean << std::endl;

		//covariance
		cv::Mat sigma = cv::Mat::zeros(3, 3, CV_32F);

		for (int i = 0; i < M; ++i){
			cv::Vec3f a_ = vPTAM3DPoints[i] - pPTAMMean;
			cv::Vec3f b_ = vKinect3DPoints[i] - pKinectMean;

			cv::Mat a(a_);
			cv::Mat b(b_);

			cv::Mat c = (a)*((b).t());

			sigma += (c);
		}

		sigma /= M;

		PTAMfromKinect = TooN::Zeros;
		PTAMfromKinect(3, 3) = 1;

		cv::SVD solver(sigma);

		cv::Mat detmat = cv::Mat::eye(3, 3, CV_32F);
		detmat.at<float>(2, 2) = cv::determinant(solver.u * solver.vt);


		cv::Mat R = solver.u * detmat * solver.vt;

		float s = 0;

		for (int i = 0; i < M; ++i){
			cv::Vec3f a_ = vPTAM3DPoints[i] - pPTAMMean;
			cv::Vec3f b_ = vKinect3DPoints[i] - pKinectMean;
			cv::Mat a(a_);
			cv::Mat b(b_);
			cv::Mat c = (a.t()*R*b);
			s += c.at<float>(0, 0);
		}

		float denom = 0;
		for (int i = 0; i < M; ++i){
			cv::Vec3f b_ = vKinect3DPoints[i] - pKinectMean;
			float val2 = cv::norm(b_);
			denom += val2*val2;
		}

		s /= denom;

		R *= s;

		cv::Mat a(pPTAMMean);
		cv::Mat b(pKinectMean);

		cv::Mat t = (-1)*R*b + a;

		for (int i = 0; i < 3; ++i){
			for (int j = 0; j < 3; ++j){
				PTAMfromKinect(i, j) = R.at<float>(i, j);
			}
			PTAMfromKinect(i, 3) = t.at<float>(i);
		}
		PTAMfromKinect(3, 3) = 1;

		//std::cout << "Kinect2PTAM:\n" << TooN2openCV(PTAMfromKinect);
	};


	TooN::Matrix<4, 4> getPTAMfromKinect(){ return PTAMfromKinect; };

	void setPTAMfromKinect(TooN::Matrix<4, 4> K2P){
		PTAMfromKinect = K2P;
	};

	void GridProjection(TooN::SE3<> mse3CfW, std::vector<TooN::Vector<4>> * gridpts, std::vector<TooN::Vector<4>> * gridpts2, unsigned int width, unsigned int height){

		gridpts->clear();
		gridpts2->clear();

		DepthXY _depthdata[CAPTURE_SIZE_X * CAPTURE_SIZE_Y];

		getKinectData_depth_raw(_depthdata);

		TooN::SE3<> cam2world = mse3CfW.inverse();
		TooN::Matrix<4, 4> K2P = KINECT::getPTAMfromKinect();

		for (int x = 64; x < width; x += 64){
			for (int y = 64; y < height; y += 64){
				float dX = x;
				float dY = height - 1 - y;
				float depth = _depthdata[(int)dX + (int)dY*width].depth;
				if (depth == 0) continue;

				cv::Vec3f skeletonPt_cv = mapDepthToSkeletonPoint(_depthdata[(int)dX + (int)dY*width]);
				TooN::Vector<4> skeletonPt = TooN::makeVector(skeletonPt_cv(0), skeletonPt_cv(1), skeletonPt_cv(2), 1);

				gridpts->push_back(cam2world * K2P * (skeletonPt));

				/*skeletonPt /= skeletonPt[2];
				skeletonPt[3] = 1;
				gridpts2->push_back(cam2world * (skeletonPt));*/

				TooN::Vector<4, float> temp = K2P * (skeletonPt);
				temp /= temp[2];
				temp[3] = 1;
				gridpts2->push_back(cam2world * temp);


				//std::cout << _depthdata[x+y*WIDTH] << ", (" << x << ", " << y << ") -> " << gridpts2->back() << std::endl;

			}
		}
		std::cout << "Grid size: " << gridpts->size() << std::endl;

	}
}

#if 0

#include <Ole2.h>
#include <Windows.h>
#include "PTAM2Kinect.h"
#include "KeyFrame.h"
#include "MapPoint.h"
#include <TooN\TooN.h>
#include "NuiApi.h"
#include <fstream>
#include "dgels.h"
#include "dgesvd.h"
#include <vector>
#include <opencv2\opencv.hpp>
#include "cvd/opencv.h"
#include "ATANCamera.h"
#include "KinectStarter.h"

#define METHOD 3

using namespace TooN;

namespace KINECT
{

	//double PTAM2Kinect = 1;
	TooN::Matrix<4,4> PTAMfromKinect;

	TooN::Vector<4> Vector2TooN(Vector4 v){
		TooN::Vector<4> ret;
		ret[0] = v.x/v.w;
		ret[1] = v.y/v.w;
		ret[2] = v.z/v.w;
		ret[3] = 1;
		return ret;

	}

	template<int T>
	cv::Vec<double, T> TooN2openCV(Vector<T> v){
		cv::Vec<double, T> ret;
		for(int i=0;i<T;++i){
			ret[i] = v[i];
		}
		return ret;
	}

	template<int T>
	cv::Vec<double, 3> TooN2openCV3(Vector<T> v){
		cv::Vec<double, 3> ret;
		for(int i=0;i<3;++i){
			ret[i] = v[i];
		}
		return ret;
	}

	template<int R,int C>
	cv::Mat TooN2openCV(Matrix<R,C> m){
		cv::Mat ret(R,C,cv::DataType<float>::type);
		for(int r=0;r<R;++r){
			for(int c=0;c<C;++c){
				ret.at<float>(r,c) = m[r][c];
			}
		}
		return ret;
	}

	cv::Mat getPTAMfromKinect_mat(){
		return TooN2openCV(PTAMfromKinect);
	}

#if 1
	void calcPTAMfromKinect(PTAMM::KeyFrame* kf, NUI_DEPTH_IMAGE_POINT* dmap){

		std::vector<TooN::Vector<4>> ptamvec;
		for(auto it = kf->mMeasurements.begin(); it!=kf->mMeasurements.end(); ++it){


			TooN::Vector<3> mpt = it->first->v3WorldPos;
			TooN::Vector<4> PTAMpoint = kf->se3CfromW*TooN::makeVector(mpt[0], mpt[1], mpt[2], 1);//3d point from camera POV

			ptamvec.push_back(PTAMpoint);

		}

		calcPTAMfromKinect(ptamvec, dmap);
	}


	void calcPTAMfromKinect(std::vector<TooN::Vector<4>> camPointVector, NUI_DEPTH_IMAGE_POINT* dmap){


		std::vector<cv::Vec3d> ptamvec_;
		std::vector<cv::Vec3d> kinectvec_;
		std::vector<double> sumvec;

		TooN::Vector<4> summer = TooN::makeVector(1,1,1,1);

		double minDist = 10000;

		//ptam point mean
		cv::Vec3d pmean(0,0,0,0);

		//kinect point mean
		cv::Vec3d qmean(0,0,0,0);

		PTAMM::ATANCamera cam("Camera");

		TooN::Vector<5> camParams = cam.GetParams();

		Matrix<4,4> camMatrix = TooN::Zeros;
		camMatrix(0,0) = camParams[0] * CAPTURE_SIZE_X;
		camMatrix(1,1) = camParams[1] * CAPTURE_SIZE_Y;
		camMatrix(0,2) = camParams[2] * CAPTURE_SIZE_X - 0.5;
		camMatrix(1,2) = camParams[3] * CAPTURE_SIZE_Y - 0.5;
		camMatrix(2,2) = 1;
		camMatrix(3,3) = 1;

		INuiCoordinateMapper* mapper;
		getSensor()->NuiGetCoordinateMapper(&mapper);

		for(auto it = camPointVector.begin(); it!=camPointVector.end(); ++it){
			TooN::Vector<4> PTAMpoint = *it;//3d point from camera POV

			//unproject this point
			TooN::Vector<4> xy = camMatrix*PTAMpoint;
			xy[0] /= xy[2];
			xy[1] /= xy[2];

			if(xy[0] >= CAPTURE_SIZE_X || xy[0] < 0 ||
				xy[1] >= CAPTURE_SIZE_Y || xy[1] < 0)
				continue;

			LONG depth = 0;
			int f_x, f_y;

			//need to find which corner it is so that the depth can be accurate
			//loop thru 8 corners
			for(int x_=-1;x_<=1;++x_){
				for(int y_=-1;y_<=1;++y_){
					if((xy[0]+x_<0) ||
						(xy[0]+x_>=CAPTURE_SIZE_X)||
						(xy[1]+y_<0)||
						(xy[1]+y_>=CAPTURE_SIZE_Y))
						continue;
					if(dmap[CAPTURE_SIZE_X-1-(int)xy[0]+x_+((int)xy[1]+y_)*CAPTURE_SIZE_Y].depth==0) continue;
					if(depth < dmap[CAPTURE_SIZE_X-1-(int)xy[0]+x_+((int)xy[1]+y_)*CAPTURE_SIZE_Y].depth){
						depth = dmap[CAPTURE_SIZE_X-1-(int)xy[0]+x_+((int)xy[1]+y_)*CAPTURE_SIZE_Y].depth;
						f_x = x_;
						f_y = y_;
					}
				}
			}

			f_x = 0;
			f_y = 0;

			Vector4 skelPt;
			HRESULT r = mapper->MapDepthPointToSkeletonPoint(NUI_IMAGE_RESOLUTION_640x480, &dmap[CAPTURE_SIZE_X-1-(int)xy[0]+f_x+((int)xy[1]+f_y)*CAPTURE_SIZE_X], &skelPt);

			if(!FAILED(r) && skelPt.z > 0.01)
			{


				TooN::Vector<4> skelspace = Vector2TooN(skelPt);


				//"kinect flip"
				/*skelspace[0] *= -1;
				skelspace[1] *= -1;*/

				ptamvec_.push_back(TooN2openCV3(PTAMpoint));
				kinectvec_.push_back(TooN2openCV3(skelspace));

				TooN::Vector<4> sqsum = (PTAMpoint - skelspace);
				for(int dim=0;dim<4;++dim){
					sqsum[dim] *= sqsum[dim];
				}

				double sum = sqsum*summer;
				sumvec.push_back(sum);


				cv::Vec3d cvPTAMpoint = TooN2openCV3(PTAMpoint);
				cv::Vec3d cvSkelpoint = TooN2openCV3(skelspace);

				pmean += cvPTAMpoint;
				qmean += cvSkelpoint;

#if 0

				NUI_COLOR_IMAGE_POINT colorPt;
				mapper->MapDepthPointToColorPoint(NUI_IMAGE_RESOLUTION_640x480, &dmap[CAPTURE_SIZE_X-1-(int)xy[0]+f_x+((int)xy[1]+f_y)*CAPTURE_SIZE_X],
					NUI_IMAGE_TYPE_COLOR, NUI_IMAGE_RESOLUTION_640x480, &colorPt);

				/*
				cv::Vec2d PTAMproj(cvPTAMpoint(0)/cvPTAMpoint(2), cvPTAMpoint(1)/cvPTAMpoint(2));
				cv::Vec2d Skelproj(cvSkelpoint(0)/cvSkelpoint(2), cvSkelpoint(1)/cvSkelpoint(2));


				std::cout << "PTAM: " << PTAMproj << " skel: " << Skelproj << " color: " << colorPt.x << " " << colorPt.y << std::endl;*/

				TooN::Vector<4> projPTAM = camMatrix * *it;
				TooN::Vector<4> projSkel = camMatrix * skelspace;

				projPTAM/=projPTAM[2];
				projSkel/=projSkel[2];

				std::cout << "PTAM: " << projPTAM.slice<0,2>() << " skel: " << projSkel.slice<0,2>() << " color: " << colorPt.x << " " << colorPt.y << std::endl;
#endif
			}
		}


		int M = ptamvec_.size();

		//average
		pmean /= M;
		qmean /= M;

		//std::cout << "p-mean " <<  pmean << std::endl << "q-mean " << qmean << std::endl;

		//covariance
		cv::Mat sigma = cv::Mat::zeros(3,3,CV_64FC1);

		for(int i=0;i<M;++i){
			cv::Vec3d a_ = ptamvec_[i]-pmean;
			cv::Vec3d b_ = kinectvec_[i]-qmean;



			cv::Mat a (a_);
			cv::Mat b (b_);



			cv::Mat c = (a)*((b).t());



			sigma += (c);
		}

		sigma /= M;

		PTAMfromKinect = TooN::Zeros;
		PTAMfromKinect(3,3) = 1;

		cv::SVD solver(sigma);

		cv::Mat detmat = cv::Mat::eye(3,3,CV_64FC1);
		detmat.at<double>(2,2) = cv::determinant(solver.u * solver.vt);


		cv::Mat R = solver.u * detmat * solver.vt;


		std::cout << R<< std::endl;

		double s = 0;

		for(int i=0;i<M;++i){
			cv::Vec3d a_ = ptamvec_[i]-pmean;
			cv::Vec3d b_ = kinectvec_[i]-qmean;
			cv::Mat a (a_);
			cv::Mat  b(b_);
			cv::Mat c = (a.t()*R*b);
			s += c.at<double>(0,0);
		}

		double denom = 0;
		for(int i=0;i<M;++i){
			cv::Vec3d b_ = kinectvec_[i]-qmean;
			double val2 = cv::norm(b_);
			denom += val2*val2;
		}

		s/= denom;

		R *= s;

		cv::Mat a(pmean);
		cv::Mat b (qmean);

		cv::Mat t = (-1)*R*b + a;

		for(int i=0;i<3;++i){
			for(int j=0;j<3;++j){
				PTAMfromKinect(i,j) = R.at<double>(i,j);
			}
			PTAMfromKinect(i,3) = t.at<double>(i);
		}
		PTAMfromKinect(3,3) = 1;

		std::cout << PTAMfromKinect << std::endl;

	};
#endif
#if 0
	void calcPTAMfromKinect(PTAMM::KeyFrame* kf, USHORT dmap[CAPTURE_SIZE_X][CAPTURE_SIZE_Y]){
		std::ofstream file;
		file.open("P2K.txt");


		std::vector<cv::Vec3d> ptamvec_;
		std::vector<cv::Vec3d> kinectvec_;
		std::vector<double> sumvec;

		TooN::Vector<4> summer = TooN::makeVector(1,1,1,1);

		double minDist = 10000;

		//ptam point mean
		cv::Vec3d pmean(0,0,0,0);

		//kinect point mean
		cv::Vec3d qmean(0,0,0,0);

		for(auto it = kf->mMeasurements.begin(); it!=kf->mMeasurements.end(); ++it){
			TooN::Vector<2> xy = it->second.v2RootPos; //whats the difference between pyramid level 0 and Z=1 plane?


			TooN::Vector<3> mpt = it->first->v3WorldPos;
			TooN::Vector<4> PTAMpoint = kf->se3CfromW*TooN::makeVector(mpt[0], mpt[1], mpt[2], 1);//3d point from camera POV

			USHORT depth = dmap[(int)xy[0]][(int)xy[1]];

			//need to find which corner it is so that the depth can be accurate
			//loop thru 8 corners
			for(int x_=-1;x_<=1;++x_){
				for(int y_=-1;y_<=1;++y_){
					if((xy[0]+x_<0) ||
						(xy[0]+x_>=CAPTURE_SIZE_X)||
						(xy[1]+y_<0)||
						(xy[1]+y_>=CAPTURE_SIZE_Y))
						continue;
					if(dmap[(int)xy[0]+x_][(int)xy[1]+y_]==0) continue;
					if(depth < dmap[(int)xy[0]+x_][(int)xy[1]+y_]){
						depth = dmap[(int)xy[0]+x_][(int)xy[1]+y_];
					}
				}
			}

			if(depth > 0)
			{
				TooN::Vector<4> skelspace = Vector2TooN(NuiTransformDepthImageToSkeleton((LONG)xy[0], (LONG)xy[1], depth, NUI_IMAGE_RESOLUTION_640x480));


				file << "Kinect: " << skelspace[0] << " " << skelspace[1] << " " << skelspace[2]  << " : " << depth<< std::endl;
				file << "PTAM: "  << PTAMpoint[0] << " " << PTAMpoint[1] << " " << PTAMpoint [2] << std::endl;
				file << "2D: " << xy[0] << " " << xy[1] << std::endl;

				skelspace[0] *= -1;

				ptamvec_.push_back(TooN2openCV3(PTAMpoint));
				kinectvec_.push_back(TooN2openCV3(skelspace));

				TooN::Vector<4> sqsum = (PTAMpoint - skelspace);
				for(int dim=0;dim<4;++dim){
					sqsum[dim] *= sqsum[dim];
				}

				double sum = sqsum*summer;
				sumvec.push_back(sum);

				pmean += TooN2openCV3(PTAMpoint);
				qmean += TooN2openCV3(skelspace);

			}
		}

		file.close();

		int M = ptamvec_.size();

		//average
		pmean /= M;
		qmean /= M;

		//covariance
		cv::Mat sigma = cv::Mat::zeros(3,3,CV_64FC1);

		for(int i=0;i<M;++i){
			cv::Vec3d a_ = ptamvec_[i]-pmean;
			cv::Vec3d b_ = kinectvec_[i]-qmean;



			cv::Mat a (a_);
			cv::Mat b (b_);



			cv::Mat c = (a)*((b).t());



			sigma += (c);
		}

		sigma /= M;
		std::cout << sigma << std::endl;

		PTAMfromKinect = TooN::Zeros;
		PTAMfromKinect(3,3) = 1;

		cv::SVD solver(sigma);

		cv::Mat detmat = cv::Mat::eye(3,3,CV_64FC1);
		detmat.at<double>(2,2) = cv::determinant(solver.u * solver.vt);

		std::cout << "U :" << std::endl <<  solver.u << std::endl;
		std::cout << "Vt :" << std::endl << solver.vt << std::endl;
		std::cout << detmat << std::endl;

		cv::Mat R = solver.u * detmat * solver.vt;


		std::cout << R<< std::endl;

		double s = 0;

		for(int i=0;i<M;++i){
			cv::Vec3d a_ = ptamvec_[i]-pmean;
			cv::Vec3d b_ = kinectvec_[i]-qmean;
			cv::Mat a (a_);
			cv::Mat  b(b_);
			cv::Mat c = (a.t()*R*b);
			s += c.at<double>(0,0);
		}

		double denom = 0;
		for(int i=0;i<M;++i){
			cv::Vec3d b_ = kinectvec_[i]-qmean;
			double val2 = cv::norm(b_);
			denom += val2;
		}

		s/= denom;
		std::cout << s << std::endl;
		R *= s;

		cv::Mat a(pmean);
		cv::Mat b (qmean);

		cv::Mat t = (-1)*R*a + b;

		for(int i=0;i<3;++i){
			for(int j=0;j<3;++j){
				PTAMfromKinect(i,j) = R.at<double>(i,j);
			}
			PTAMfromKinect(i,3) = t.at<double>(i);
		}


	};
#endif

#if 0
	void calcPTAMfromKinect(std::vector<TooN::Vector<4>> camPointVector, USHORT dmap[CAPTURE_SIZE_X][CAPTURE_SIZE_Y]){


		std::vector<double> ptamvec_;
		std::vector<double> kinectvec_;
		std::vector<double> sumvec;

		TooN::Vector<4> summer = TooN::makeVector(1,1,1,1);

		double minDist = 10000;



		PTAMM::ATANCamera cam("Camera");

		TooN::Vector<5> camParams = cam.GetParams();

		Matrix<4,4> camMatrix = TooN::Zeros;
		camMatrix(0,0) = camParams[0] * CAPTURE_SIZE_X;
		camMatrix(1,1) = camParams[1] * CAPTURE_SIZE_Y;
		camMatrix(0,2) = camParams[2] * CAPTURE_SIZE_X - 0.5;
		camMatrix(1,2) = camParams[3] * CAPTURE_SIZE_Y - 0.5;
		camMatrix(2,2) = 1;
		camMatrix(3,3) = 1;

		for(auto it = camPointVector.begin(); it!=camPointVector.end(); ++it){
			TooN::Vector<4> PTAMpoint = *it;//3d point from camera POV

			//unproject this point
			TooN::Vector<4> xy = camMatrix*PTAMpoint;
			xy[0] /= xy[2];
			xy[1] /= xy[2];

			if(xy[0] >= CAPTURE_SIZE_X || xy[0] < 0 ||
				xy[1] >= CAPTURE_SIZE_Y || xy[1] < 0)
				continue;

			USHORT depth = dmap[(int)xy[0]][(int)xy[1]];

			//need to find which corner it is so that the depth can be accurate
			//loop thru 8 corners
			for(int x_=-1;x_<=1;++x_){
				for(int y_=-1;y_<=1;++y_){
					if((xy[0]+x_<0) ||
						(xy[0]+x_>=CAPTURE_SIZE_X)||
						(xy[1]+y_<0)||
						(xy[1]+y_>=CAPTURE_SIZE_Y))
						continue;
					if(dmap[(int)xy[0]+x_][(int)xy[1]+y_]==0) continue;
					if(depth < dmap[(int)xy[0]+x_][(int)xy[1]+y_]){
						depth = dmap[(int)xy[0]+x_][(int)xy[1]+y_];
					}
				}
			}

			if(depth > 0)
			{
				TooN::Vector<4> skelspace = Vector2TooN(NuiTransformDepthImageToSkeleton((LONG)xy[0], (LONG)xy[1], depth, NUI_IMAGE_RESOLUTION_640x480));

				skelspace[0] *= -1;

				ptamvec_.push_back(TooN2openCV3(PTAMpoint)(2));
				kinectvec_.push_back(TooN2openCV3(skelspace)(2));

				TooN::Vector<4> sqsum = (PTAMpoint - skelspace);
				for(int dim=0;dim<4;++dim){
					sqsum[dim] *= sqsum[dim];
				}

				double sum = sqsum*summer;
				sumvec.push_back(sum);

			}
		}

		cv::Mat A, b;
		A.create(kinectvec_.size(),1,CV_64FC1);
		b.create(kinectvec_.size(),1,CV_64FC1);

		for(int i=0;i<kinectvec_.size();++i){
			A.at<double>(i) = kinectvec_[i];
			b.at<double>(i) = ptamvec_[i];
		}

		cv::Mat out;
		out.create(1,1,CV_64FC1);
		try
		{

			cv::solve(A, b, out, cv::DECOMP_SVD);

			PTAMfromKinect(0,0) = out.at<double>(0);
			PTAMfromKinect(1,1) = out.at<double>(0);
			PTAMfromKinect(2,2) = out.at<double>(0);
			PTAMfromKinect(3,3)=1;

			std::cout << PTAMfromKinect << std::endl;
		}catch(cv::Exception& e){
			std::cout << e.what() << std::endl;
		}

	};

	void calcPTAMfromKinect(PTAMM::KeyFrame* kf, USHORT dmap[CAPTURE_SIZE_X][CAPTURE_SIZE_Y]){

		std::vector<TooN::Vector<4>> ptamvec;
		for(auto it = kf->mMeasurements.begin(); it!=kf->mMeasurements.end(); ++it){


			TooN::Vector<3> mpt = it->first->v3WorldPos;
			TooN::Vector<4> PTAMpoint = kf->se3CfromW*TooN::makeVector(mpt[0], mpt[1], mpt[2], 1);//3d point from camera POV

			ptamvec.push_back(PTAMpoint);

		}

		calcPTAMfromKinect(ptamvec, dmap);
	}

#endif
	TooN::Matrix<4,4> getPTAMfromKinect(){return PTAMfromKinect;};

	void setPTAMfromKinect(TooN::Matrix<4,4> K2P){
		PTAMfromKinect = K2P;
	}

#if 0

	void calcPTAMfromKinect(PTAMM::KeyFrame* kf, USHORT dmap[CAPTURE_SIZE_X][CAPTURE_SIZE_Y]){
		std::ofstream file;
		file.open("P2K.txt");


		std::vector<TooN::Vector<4>> ptamvec_;
		std::vector<TooN::Vector<4>> kinectvec_;
		std::vector<double> sumvec;

		TooN::Vector<4> summer = TooN::makeVector(1,1,1,1);

		double minDist = 10000;

		for(auto it = kf->mMeasurements.begin(); it!=kf->mMeasurements.end(); ++it){
			TooN::Vector<2> xy = it->second.v2RootPos; //whats the difference between pyramid level 0 and Z=1 plane?


			TooN::Vector<3> mpt = it->first->v3WorldPos;
			TooN::Vector<4> PTAMpoint = kf->se3CfromW*TooN::makeVector(mpt[0], mpt[1], mpt[2], 1);//3d point from camera POV

			USHORT depth = dmap[(int)xy[0]][(int)xy[1]];

			//need to find which corner it is so that the depth can be accurate
			//loop thru 8 corners
			for(int x_=-1;x_<=1;++x_){
				for(int y_=-1;y_<=1;++y_){
					if((xy[0]+x_<0) ||
						(xy[0]+x_>=CAPTURE_SIZE_X)||
						(xy[1]+y_<0)||
						(xy[1]+y_>=CAPTURE_SIZE_Y))
						continue;
					if(dmap[(int)xy[0]+x_][(int)xy[1]+y_]==0) continue;
					if(depth < dmap[(int)xy[0]+x_][(int)xy[1]+y_]){
						depth = dmap[(int)xy[0]+x_][(int)xy[1]+y_];
					}
				}
			}

			if(depth > 0)
			{
				TooN::Vector<4> skelspace = Vector2TooN(NuiTransformDepthImageToSkeleton((LONG)xy[0], (LONG)xy[1], depth, NUI_IMAGE_RESOLUTION_640x480));

				//PTAM2Kinect = (double)skelspace.z/PTAMpoint[2];
				//std::cout << "P2K: " << PTAM2Kinect << std::endl;
				//using std::cout;
				//using std::endl;
				//cout << "SKELSPACE: " << endl << "X: " << skelspace.x << endl << "Y: " << skelspace.y << endl << "Z: " << skelspace.z;
				//cout << endl << "PTAM point: " << endl << "X: " << PTAMpoint[0] << endl << "Y: " << PTAMpoint[1] << endl << "Z: " << PTAMpoint[2] << endl << endl;

				file << "Kinect: " << skelspace[0] << " " << skelspace[1] << " " << skelspace[2]  << " : " << depth<< std::endl;
				file << "PTAM: "  << PTAMpoint[0] << " " << PTAMpoint[1] << " " << PTAMpoint [2] << std::endl;
				file << "2D: " << xy[0] << " " << xy[1] << std::endl;

				ptamvec_.push_back(PTAMpoint);
				kinectvec_.push_back((skelspace));

				TooN::Vector<4> sqsum = (PTAMpoint - skelspace);
				for(int dim=0;dim<4;++dim){
					sqsum[dim] *= sqsum[dim];
				}

				double sum = sqsum*summer;
				sumvec.push_back(sum);
#if 0
				if(minDist > sum && sum > 250){ 
					minDist = sum;
				}
#endif

			}
		}

		file.close();

#if 0
		std::vector<TooN::Vector<4>> ptamvec;
		std::vector<TooN::Vector<4>> kinectvec;
		//remove outliers by only selecting accdg to min dist
		for(int i = 0; i< sumvec.size(); ++i){
			if(sumvec[i] < minDist * 5){
				ptamvec.push_back(ptamvec_[i]);
				kinectvec.push_back(kinectvec_[i]);
			}
		}
#endif

		PTAMfromKinect = TooN::Zeros;
		PTAMfromKinect(3,3) = 1;

		//LAPACK vars
		char* TRANS = "N";
		integer M = kinectvec_.size();
		integer N = 2;
		integer NRHS = 1;
		integer LDA = M;
		integer LDB = M;
		integer LWORK = M + M;
		double* WORK = (double*)malloc(sizeof(double)*LWORK);
		integer INFO;

		for(int dim=0;dim<3;++dim)
		{
			double * ptamarr = (double*)malloc(sizeof(double) * ptamvec_.size());
			double * kinarr  = (double*)malloc(sizeof(double) * kinectvec_.size() * N);

			for(int i=0;i<M;++i){
				ptamarr[i] = ptamvec_[i][dim];
				kinarr[N*i] = kinectvec_[i][dim];
				kinarr[N*i+1] = 1;
			}

			dgels_(TRANS, &M, &N, &NRHS, kinarr, &LDA, ptamarr, &LDB, WORK, &LWORK, &INFO);
			PTAMfromKinect(dim,dim) = ptamarr[0];
			PTAMfromKinect(dim,3) = ptamarr[1];
		}
	};
#endif
	//double getPTAM2Kinect(){ return PTAM2Kinect; };

}

#endif

#endif