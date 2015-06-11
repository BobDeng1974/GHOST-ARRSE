#pragma once

#include <Windows.h>
#include <Kinect.h>

// Safe release for interfaces
template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
	if (pInterfaceToRelease != NULL)
	{
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}

#define CAPTURE_SIZE_X_COLOR 1920
#define CAPTURE_SIZE_Y_COLOR 1080
#define CAPTURE_SIZE_X_DEPTH 512
#define CAPTURE_SIZE_Y_DEPTH 424

namespace KINECT{
	void InitKinect2Starter();

	void DestroyKinect2Starter();

	HRESULT InitializeDefaultSensor();
	
	//after calling this, get the depth fram with GetDepth or GetDepthRGBX
	void UpdateDepth();

	void ProcessDepth(INT64 nTime, const UINT16* pBuffer, int nWidth, int nHeight, USHORT nMinDepth, USHORT nMaxDepth);

	void ProcessDepthNoRGBX(INT64 nTime, const UINT16* pBuffer, int nWidth, int nHeight, USHORT nMinDepth, USHORT nMaxDepth);

	void ProcessDepthToColor(const USHORT * pDepthBuffer, int nDepthWidth, int nDepthHeight, const DepthSpacePoint * pColorDepthMap, int nColorWidth, int nColorHeight);

	void UpdateColor();

	void ProcessColor(INT64 nTime, RGBQUAD* pBuffer, int nWidth, int nHeight);

	void UpdateBody();

	void ProcessBody(unsigned int nTime, unsigned int nBodyCount, IBody * ppBodies[6]);

	void UpdateBodyFrameIndex();

	void ProcessBodyFrameIndexColor(unsigned char * pBodyIndexBuffer, unsigned int nDepthWidth, unsigned int nDepthHeight, const DepthSpacePoint * pColorDepthMap, int nColorWidth, int nColorHeight);
	
	void CalculateColorDepthMap();

	void mapDepthToSkeleton(float * nDepthX, float * nDepthY, long * nDepth, float * nCameraX, float * nCameraY, float * nCameraZ);

	RGBQUAD * GetDepthRGBX();
	USHORT * GetDepth();
	RGBQUAD * GetColorRGBX();
	USHORT * GetDepthMappedToColor();
	int * GetDepthXMappedToColor();
	int * GetDepthYMappedToColor();
	RGBQUAD * GetBodyColorRGBX();
	Joint * GetJoints();

	unsigned int getDepthWidth();
	unsigned int getDepthHeight();
	unsigned int getColorWidth();
	unsigned int getColorHeight();
	bool getSkeletonIsGood();
	ICoordinateMapper * getCoordinateMapper();
}