#include "ghostsettings.h"

#if GHOST_INPUT == INPUT_KINECT2

#include "Kinect2Starter.h"
#include "KinectManager.h"
#include <Kinect.h>
#include <iostream>

void DumpHR(HRESULT hr)
{
	//
	//if (hr < 0)
	//	hr += 0x100000000;
	//if (hr & 0x80000000)
	//	std::cout << ("Error code") << std::endl;
	//else
	//	std::cout << ("Success code") << std::endl;
	//
	//auto facility = (hr & 0x7FFF0000) >> 16;
	//
	//std::cout << "Facility " << facility << std::endl;
	//
	//auto scode = hr & 0x0000FFFF;
	//
	//std::cout << "SCode " << scode << std::endl;

}


namespace KINECT{

	IKinectSensor * m_pKinectSensor;
	ICoordinateMapper * m_pCoordinateMapper;
	IBodyFrameReader * m_pBodyFrameReader;
	IColorFrameReader * m_pColorFrameReader;
	IDepthFrameReader * m_pDepthFrameReader;
	IBodyIndexFrameReader * m_pBodyIndexFrameReader;

	INT64 m_nStartTime;

    RGBQUAD * m_pDepthRGBX;
	USHORT * m_pDepth;
    RGBQUAD * m_pColorRGBX;
	DepthSpacePoint * m_pColorDepthMap;
	USHORT * m_pDepthMappedToColor;
	int * m_pDepthXMappedToColor;
	int * m_pDepthYMappedToColor;
	RGBQUAD * m_pBodyColorRGBX;

	Joint * m_pJoints;

	UCHAR m_nBodyIndex;

	unsigned int m_nDepthWidth;
	unsigned int m_nDepthHeight;
	unsigned int m_nColorWidth;
	unsigned int m_nColorHeight;

	bool m_bCalculateDepthRGBX;
	bool m_bMapDepthToColor;
	bool m_bSkeletonIsGood;
	bool m_bColorDepthMapCalculated;

	void InitKinect2Starter(){
		
		//m_pDepthRGBX = new RGBQUAD[CAPTURE_SIZE_X_DEPTH * CAPTURE_SIZE_Y_DEPTH];
		m_pDepth = new USHORT[CAPTURE_SIZE_X_DEPTH * CAPTURE_SIZE_Y_DEPTH];
		m_pColorRGBX = new RGBQUAD[CAPTURE_SIZE_X_COLOR * CAPTURE_SIZE_Y_COLOR];
		//m_pColorDepthMap = new DepthSpacePoint[CAPTURE_SIZE_X_COLOR * CAPTURE_SIZE_Y_COLOR];
		//m_pDepthXMappedToColor = new int[CAPTURE_SIZE_X_COLOR * CAPTURE_SIZE_Y_COLOR];
		//m_pDepthYMappedToColor = new int[CAPTURE_SIZE_X_COLOR * CAPTURE_SIZE_Y_COLOR];
		//m_pDepthMappedToColor = new USHORT[CAPTURE_SIZE_X_COLOR * CAPTURE_SIZE_Y_COLOR];
		//m_pBodyColorRGBX = new RGBQUAD[CAPTURE_SIZE_X_COLOR * CAPTURE_SIZE_Y_COLOR];

		m_pJoints = new Joint[JointType_Count];


		m_nStartTime = 0;
		m_bCalculateDepthRGBX = false;
		m_bMapDepthToColor = true;
		m_bSkeletonIsGood = false;
		m_nBodyIndex = 0xff;
		m_bColorDepthMapCalculated = false;
	}

	void DestroyKinect2Starter(){
		if(m_pDepthRGBX){
			delete [] m_pDepthRGBX;
			m_pDepthRGBX = NULL;
		}
		if(m_pDepth){
			delete [] m_pDepth;
			m_pDepth = NULL;
		}
		if(m_pColorRGBX){
			delete [] m_pColorRGBX;
			m_pColorRGBX = NULL;
		}
		if(m_pColorDepthMap){
			delete [] m_pColorDepthMap;
		}
		if(m_pDepthMappedToColor){
			delete[] m_pDepthMappedToColor;
		}
		if (m_pDepthXMappedToColor){
			delete[] m_pDepthXMappedToColor;
		}
		if (m_pDepthYMappedToColor){
			delete[] m_pDepthYMappedToColor;
		}
		if(m_pBodyColorRGBX){
			delete [] m_pBodyColorRGBX;
		}
		if(m_pJoints){
			delete [] m_pJoints;
		}
	}

	HRESULT InitializeDefaultSensor()
	{
		HRESULT hr;

		hr = GetDefaultKinectSensor(&m_pKinectSensor);
		if (FAILED(hr))
		{
			return hr;
		}

		if (m_pKinectSensor)
		{
			// Initialize the Kinect and get coordinate mapper and the body reader
			IBodyFrameSource* pBodyFrameSource = NULL;

			hr = m_pKinectSensor->Open();

			if (SUCCEEDED(hr))
			{
				hr = m_pKinectSensor->get_CoordinateMapper(&m_pCoordinateMapper);
			}

			if (SUCCEEDED(hr))
			{
				hr = m_pKinectSensor->get_BodyFrameSource(&pBodyFrameSource);
			}

			if (SUCCEEDED(hr))
			{
				hr = pBodyFrameSource->OpenReader(&m_pBodyFrameReader);
			}

			SafeRelease(pBodyFrameSource);

			
			// get the color reader
			IColorFrameSource* pColorFrameSource = NULL;

			if (SUCCEEDED(hr))
			{

				hr = m_pKinectSensor->get_ColorFrameSource(&pColorFrameSource);
			}

			if (SUCCEEDED(hr))
			{
				hr = pColorFrameSource->OpenReader(&m_pColorFrameReader);
			} 

			SafeRelease(pColorFrameSource);

			
			// get the depth reader

			IDepthFrameSource* pDepthFrameSource = NULL;

			if (SUCCEEDED(hr))
			{
				hr = m_pKinectSensor->get_DepthFrameSource(&pDepthFrameSource);
			}

			if (SUCCEEDED(hr))
			{
				hr = pDepthFrameSource->OpenReader(&m_pDepthFrameReader);
			}

			SafeRelease(pDepthFrameSource);

			//get the body frame index reader

			IBodyIndexFrameSource * pBodyIndexFrameSource = NULL;

			if(SUCCEEDED(hr)){
				hr = m_pKinectSensor->get_BodyIndexFrameSource(&pBodyIndexFrameSource);
			}

			if(SUCCEEDED(hr)){
				hr = pBodyIndexFrameSource->OpenReader(&m_pBodyIndexFrameReader);
			}

			SafeRelease(pBodyIndexFrameSource);

		}

		if (!m_pKinectSensor || FAILED(hr))
		{
			std::cout << "no ready Kinect found!";
			return E_FAIL;
		}

		return hr;
	}

	void CalculateColorDepthMap(){
		HRESULT hr = m_pCoordinateMapper->MapColorFrameToDepthSpace(m_nDepthWidth*m_nDepthHeight, m_pDepth, m_nColorWidth*m_nColorHeight, m_pColorDepthMap);
		if(SUCCEEDED(hr)){
			m_bColorDepthMapCalculated = true;
		}
	}

	//after calling this, get the depth fram with GetDepth or GetDepthRGBX
	void UpdateDepth(){

		if (!m_pDepthFrameReader)
		{
			return;
		}

		IDepthFrame* pDepthFrame = NULL;

		HRESULT hr = m_pDepthFrameReader->AcquireLatestFrame(&pDepthFrame);

		if (SUCCEEDED(hr))
		{
			INT64 nTime = 0;
			IFrameDescription* pFrameDescription = NULL;
			int nWidth = 0;
			int nHeight = 0;
			USHORT nDepthMinReliableDistance = 0;
			USHORT nDepthMaxReliableDistance = 0;
			UINT nBufferSize = 0;
			UINT16 *pBuffer = NULL;

			hr = pDepthFrame->get_RelativeTime(&nTime);

			if (SUCCEEDED(hr))
			{
				hr = pDepthFrame->get_FrameDescription(&pFrameDescription);
			}

			if (SUCCEEDED(hr))
			{
				hr = pFrameDescription->get_Width(&nWidth);
			}

			if (SUCCEEDED(hr))
			{
				m_nDepthWidth = nWidth;
				hr = pFrameDescription->get_Height(&nHeight);
			}

			if (SUCCEEDED(hr))
			{
				m_nDepthHeight = nHeight;
				hr = pDepthFrame->get_DepthMinReliableDistance(&nDepthMinReliableDistance);
			}

			if (SUCCEEDED(hr))
			{
				hr = pDepthFrame->get_DepthMaxReliableDistance(&nDepthMaxReliableDistance);
			}

			if (SUCCEEDED(hr))
			{
				hr = pDepthFrame->AccessUnderlyingBuffer(&nBufferSize, &pBuffer);            
			}

			if (SUCCEEDED(hr))
			{

				if(m_bCalculateDepthRGBX)
					ProcessDepth(nTime, pBuffer, nWidth, nHeight, nDepthMinReliableDistance, nDepthMaxReliableDistance);
				else
					ProcessDepthNoRGBX(nTime, pBuffer, nWidth, nHeight, nDepthMinReliableDistance, nDepthMaxReliableDistance);

				if(!m_bColorDepthMapCalculated){
					CalculateColorDepthMap();
				}

				if(m_bMapDepthToColor && m_nColorWidth > 0 && m_nColorHeight > 0 && SUCCEEDED(hr) && m_bColorDepthMapCalculated){
					ProcessDepthToColor(m_pDepth, m_nDepthWidth, m_nDepthHeight, m_pColorDepthMap, m_nColorWidth, m_nColorHeight);
				}
			}

			SafeRelease(pFrameDescription);
		}
		else{
			DumpHR(hr);
		}

		SafeRelease(pDepthFrame);
	}

	void ProcessDepth(INT64 nTime, const UINT16* pBuffer, int nWidth, int nHeight, USHORT nMinDepth, USHORT nMaxDepth)
	{
		if (!m_nStartTime)
		{
			m_nStartTime = nTime;
		}

		// Make sure we've received valid data
		if (m_pDepthRGBX && m_pDepth && pBuffer)
		{
			RGBQUAD* pRGBX = m_pDepthRGBX;
			USHORT * pDepth = m_pDepth;

			// end pixel is start + width*height - 1
			const UINT16* pBufferEnd = pBuffer + (nWidth * nHeight);

			while (pBuffer < pBufferEnd)
			{
				USHORT depth = *pBuffer;
				*pDepth = depth;

				// To convert to a byte, we're discarding the most-significant
				// rather than least-significant bits.
				// We're preserving detail, although the intensity will "wrap."
				// Values outside the reliable depth range are mapped to 0 (black).

				// Note: Using conditionals in this loop could degrade performance.
				// Consider using a lookup table instead when writing production code.
				BYTE intensity = static_cast<BYTE>((depth >= nMinDepth) && (depth <= nMaxDepth) ? (depth % 256) : 0);

				pRGBX->rgbRed   = intensity;
				pRGBX->rgbGreen = intensity;
				pRGBX->rgbBlue  = intensity;

				++pRGBX;
				++pDepth;
				++pBuffer;
			}

		}
	}

	void ProcessDepthNoRGBX(INT64 nTime, const UINT16* pBuffer, int nWidth, int nHeight, USHORT nMinDepth, USHORT nMaxDepth)
	{
		if (!m_nStartTime)
		{
			m_nStartTime = nTime;
		}

		// Make sure we've received valid data
		if (m_pDepth && pBuffer)
		{
			USHORT * pDepth = m_pDepth;

			// end pixel is start + width*height - 1
			const UINT16* pBufferEnd = pBuffer + (nWidth * nHeight);

			while (pBuffer < pBufferEnd)
			{
				USHORT depth = *pBuffer;
				*pDepth = depth;

				++pDepth;
				++pBuffer;
			}

		}
	}

	void ProcessDepthToColor(const USHORT * pDepthBuffer, int nDepthWidth, int nDepthHeight, const DepthSpacePoint * pColorDepthMap, int nColorWidth, int nColorHeight){
		int colorSize = nColorWidth * nColorHeight;

		if(pDepthBuffer && pColorDepthMap){
			USHORT * pDepthMappedToColor = m_pDepthMappedToColor;
			int * pDepthXMappedToColor = m_pDepthXMappedToColor;
			int * pDepthYMappedToColor = m_pDepthYMappedToColor;

			const DepthSpacePoint * pBufferEnd = pColorDepthMap + colorSize;

			while(pColorDepthMap < pBufferEnd){
				DepthSpacePoint depthSpacePoint = *pColorDepthMap;
				int pointerValue = depthSpacePoint.X + depthSpacePoint.Y * nDepthWidth;
				*pDepthMappedToColor = pDepthBuffer[pointerValue];
				*pDepthXMappedToColor = depthSpacePoint.X;
				*pDepthYMappedToColor = depthSpacePoint.Y;

				++pColorDepthMap;
				++pDepthMappedToColor;
				++pDepthXMappedToColor;
				++pDepthYMappedToColor;
			}
		}
	}

	void UpdateColor()
	{
		if (!m_pColorFrameReader)
		{
			return;
		}

		IColorFrame* pColorFrame = NULL;

		HRESULT hr = m_pColorFrameReader->AcquireLatestFrame(&pColorFrame);

		if (SUCCEEDED(hr))
		{
			INT64 nTime = 0;
			IFrameDescription* pFrameDescription = NULL;
			int nWidth = 0;
			int nHeight = 0;
			ColorImageFormat imageFormat = ColorImageFormat_None;
			UINT nBufferSize = 0;
			RGBQUAD *pBuffer = NULL;

			hr = pColorFrame->get_RelativeTime(&nTime);

			if (SUCCEEDED(hr))
			{
				hr = pColorFrame->get_FrameDescription(&pFrameDescription);
			}

			if (SUCCEEDED(hr))
			{
				hr = pFrameDescription->get_Width(&nWidth);
			}

			if (SUCCEEDED(hr))
			{
				m_nColorWidth = nWidth;
				hr = pFrameDescription->get_Height(&nHeight);
			}

			if (SUCCEEDED(hr))
			{
				m_nColorHeight = nHeight;
				hr = pColorFrame->get_RawColorImageFormat(&imageFormat);
			}

			if (SUCCEEDED(hr))
			{
				if (imageFormat == ColorImageFormat_Bgra)
				{
					hr = pColorFrame->AccessRawUnderlyingBuffer(&nBufferSize, reinterpret_cast<BYTE**>(&pBuffer));
				}
				else if (m_pColorRGBX)
				{
					pBuffer = m_pColorRGBX;
					nBufferSize = nWidth * nHeight * sizeof(RGBQUAD);
					hr = pColorFrame->CopyConvertedFrameDataToArray(nBufferSize, reinterpret_cast<BYTE*>(pBuffer), ColorImageFormat_Bgra);            
				}
				else
				{
					hr = E_FAIL;
				}
			}

			if (SUCCEEDED(hr))
			{
				{
					ProcessColor(nTime, pBuffer, nWidth, nHeight);
				}
			}

			SafeRelease(pFrameDescription);
		}
		else{
			DumpHR(hr);
		}

		SafeRelease(pColorFrame);
	}

	void ProcessColor(INT64 nTime, RGBQUAD* pBuffer, int nWidth, int nHeight) 
	{
		if (!m_nStartTime)
		{
			m_nStartTime = nTime;
		}

		// Make sure we've received valid data
		if (pBuffer)
		{
			RGBQUAD * pColor = m_pColorRGBX;

			// end pixel is start + width*height - 1
			const RGBQUAD * pBufferEnd = pBuffer + (nWidth * nHeight);

			while (pBuffer < pBufferEnd)
			{
				RGBQUAD color = *pBuffer;
				*pColor = color;

				++pColor;
				++pBuffer;
			}
		}
	}

	
	void UpdateBody()
	{
		if (!m_pBodyFrameReader)
		{
			return;
		}

		IBodyFrame* pBodyFrame = NULL;

		HRESULT hr = m_pBodyFrameReader->AcquireLatestFrame(&pBodyFrame);

		if (SUCCEEDED(hr))
		{
			INT64 nTime = 0;

			hr = pBodyFrame->get_RelativeTime(&nTime);

			IBody* ppBodies[BODY_COUNT] = {0};

			if (SUCCEEDED(hr))
			{
				hr = pBodyFrame->GetAndRefreshBodyData(_countof(ppBodies), ppBodies);
			}

			if (SUCCEEDED(hr))
			{
				ProcessBody(nTime, _countof(ppBodies), ppBodies);
			}

			for (int i = 0; i < _countof(ppBodies); ++i)
			{
				SafeRelease(ppBodies[i]);
			}
		}

		SafeRelease(pBodyFrame);
	}

	void ProcessBody(unsigned int nTime, unsigned int nBodyCount, IBody * ppBodies[6]){
		UCHAR bestBody = 0xff;
		float bestScore = 0;
		
		float trackingStateTable[3];
		trackingStateTable[TrackingState_Inferred] = 0.5;
		trackingStateTable[TrackingState_NotTracked] = 0;
		trackingStateTable[TrackingState_Tracked] = 1;

		for(int i=0;i<nBodyCount;++i){
			IBody * body = ppBodies[i];
			
			BOOLEAN bodyTracked;
			HRESULT hr = body->get_IsTracked(&bodyTracked);

			if(!SUCCEEDED(hr) || !bodyTracked) continue;

			Joint joints[JointType_Count];
			hr = body->GetJoints(JointType_Count, joints);

			if(!SUCCEEDED(hr)) continue;

			float score=0;
			for(int j=0;j<JointType_Count;++j){
				score += trackingStateTable[joints[j].TrackingState];
			}

			if(score > bestScore){
				bestScore = score;
				bestBody = i;
			}
		}

		if (bestBody == 0xff){
			m_bSkeletonIsGood = false;
			m_nBodyIndex = 0xff;
			return;
		}
		HRESULT hr = ppBodies[bestBody]->GetJoints(JointType_Count, m_pJoints);

		if(!SUCCEEDED(hr)){
			std::cerr << "Error saving joints\n";
			m_bSkeletonIsGood = false;
			m_nBodyIndex = 0xff;
		}else{
			m_bSkeletonIsGood = true;
			m_nBodyIndex = bestBody;
		}
	}

	void UpdateBodyFrameIndex(){
		if(!m_pBodyIndexFrameReader){
			return;
		}

		if(m_nDepthWidth == 0 || m_nDepthHeight == 0 || m_nColorWidth == 0 || m_nColorHeight == 0){
			
			return;
		}

		if(m_nBodyIndex == 0xff){
			return;
		}

		IBodyIndexFrame *  pBodyIndexFrame = NULL;

		HRESULT hr = m_pBodyIndexFrameReader->AcquireLatestFrame(&pBodyIndexFrame);

		if(SUCCEEDED(hr)){
			unsigned int nBufferSize = 0;
			unsigned char * pBuffer = NULL;

			if(SUCCEEDED(hr)){
				hr = pBodyIndexFrame->AccessUnderlyingBuffer(&nBufferSize, &pBuffer);
			}
			
			if(SUCCEEDED(hr)){
				hr = m_pCoordinateMapper->MapColorFrameToDepthSpace(m_nDepthWidth*m_nDepthHeight, m_pDepth, m_nColorWidth*m_nColorHeight, m_pColorDepthMap);
			}

			if(SUCCEEDED(hr)){
				ProcessBodyFrameIndexColor(pBuffer, m_nDepthWidth, m_nDepthHeight, m_pColorDepthMap, m_nColorWidth, m_nColorHeight);
			}
		}

		SafeRelease(pBodyIndexFrame);
	}

	void ProcessBodyFrameIndexColor(unsigned char * pBodyIndexBuffer, unsigned int nDepthWidth, unsigned int nDepthHeight, const DepthSpacePoint * pColorDepthMap, int nColorWidth, int nColorHeight){
		
		int colorSize = nColorWidth * nColorHeight;

		if(pBodyIndexBuffer && pColorDepthMap){
			RGBQUAD * pBodyColorRGBX = m_pBodyColorRGBX;
			RGBQUAD * pColorRGBX = m_pColorRGBX;

			const DepthSpacePoint * pBufferEnd = pColorDepthMap + colorSize;

			while(pColorDepthMap < pBufferEnd){
				DepthSpacePoint depthSpacePoint = *pColorDepthMap;
				int pointerValue = depthSpacePoint.X + depthSpacePoint.Y * nDepthWidth;
				if (pointerValue > 0){
					unsigned char bodyIndex = pBodyIndexBuffer[pointerValue];

					if (bodyIndex == m_nBodyIndex)
						*pBodyColorRGBX = *pColorRGBX;
					else{
						pBodyColorRGBX->rgbBlue = 0xff;
						pBodyColorRGBX->rgbGreen = 0xff;
						pBodyColorRGBX->rgbRed = 0xff;
						pBodyColorRGBX->rgbReserved = 0;
					}
				}
				else{
					pBodyColorRGBX->rgbBlue = 0;
					pBodyColorRGBX->rgbGreen = 0;
					pBodyColorRGBX->rgbRed = 0;
					pBodyColorRGBX->rgbReserved = 0;

				}

				++pColorDepthMap;
				++pBodyColorRGBX;
				++pColorRGBX;
			}
		}
	}
	
	//legacy
	void mapDepthToSkeleton(float * nDepthX, float * nDepthY, long * nDepth, float * nCameraX, float * nCameraY, float * nCameraZ){
		if(!m_pCoordinateMapper){
			return;
		}

		DepthSpacePoint depthSpacePoint;
		depthSpacePoint.X = *nDepthX;
		depthSpacePoint.Y = *nDepthY;

		CameraSpacePoint cameraSpacePoint;
		m_pCoordinateMapper->MapDepthPointToCameraSpace(depthSpacePoint, *nDepth, &cameraSpacePoint);

		*nCameraX = cameraSpacePoint.X;
		*nCameraY = cameraSpacePoint.Y;
		*nCameraZ = cameraSpacePoint.Z;

	}

	RGBQUAD * GetDepthRGBX(){
		return m_pDepthRGBX;
	}

	USHORT * GetDepth(){
		return m_pDepth;
	}

	RGBQUAD * GetColorRGBX(){
		return m_pColorRGBX;
	}

	USHORT * GetDepthMappedToColor(){
		return m_pDepthMappedToColor;
	}

	int * GetDepthXMappedToColor(){
		return m_pDepthXMappedToColor;
	}
	int * GetDepthYMappedToColor(){
		return m_pDepthYMappedToColor;
	}

	RGBQUAD * GetBodyColorRGBX(){
		return m_pBodyColorRGBX;
	}

	Joint * GetJoints(){
		return m_pJoints;
	}

	unsigned int getDepthWidth(){
		return m_nDepthWidth;
	}

	unsigned int getDepthHeight(){
		return m_nDepthHeight;
	}

	unsigned int getColorWidth(){
		return m_nColorWidth;
	}

	unsigned int getColorHeight(){
		return m_nColorHeight;
	}

	bool getSkeletonIsGood(){
		return m_bSkeletonIsGood;
	}

	ICoordinateMapper * getCoordinateMapper(){
		return m_pCoordinateMapper;
	}
}

#endif