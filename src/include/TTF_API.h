/**

*@mainpage			Meerecompany TTF Introduction
*@section intro		Introduce
					- DLL's default header file. Meere ToF Camera(TTF) SDK(Windows Standard DLL Version)  
*@section Program	File name 
					- TTF_API.h
*@section Version	Version
                    - 0.93
*@section ModifyInfo
					- author : Support@cube-eye.co.kr
                    - date : 2019.05.02
@section Common	
- Copyright(c) Meerecompany, 2016 All rights reserved.

*
*/

#ifndef _TTF_API_H_
#define _TTF_API_H_


#include "CameraSystem.h"
#include "Common.h"
#include <functional>
using namespace Voxel;

/**
*
*@TTF_API
*@brief Meerecompany TTF SDK function description
*@details Meerecompany TTF SDK function description
*
*/

namespace TTF_API
{
    #define MAX_DEVICE 10
    typedef int ttfHandle;

    template <typename T>
    using Vector = std::vector<T>;
    typedef std::string String;


	//!Device Information
	typedef struct _ttfDeviceInfo
	{
		//!Device Handle
		ttfHandle hnd;	
		//!Device Name
		char szName[256];			
		//!Device Sereial Number
		char szSerialNum[256];		
		//!Device Vendor ID
		unsigned short nVendorId;	
		//!Device Product ID
		unsigned short nProductId;	
		//!Device Type(0:Only Depth, 1:Depth + RGB)
		unsigned short nDeviceType;	

	} ttfDeviceInfo;

	//!Record File Frame Header's information
	typedef struct _stFRAMEHEADER
	{
		//!Saved Image Number
		int nFrames;	
		//!Image Buffer Size
		int nFrameSize;	
		//!Saved Data Type - 1:Depth, 2:Point Cloud
		int nDataType;	
		//!Frame Rate
		int nFrameRate;
	} stFrameHeader;

	//!Lens Intrinsic Parameter
	typedef struct _ttfIntrinsicParam
	{
		//!Focal length X
		float fFx;
		//!Focal length Y
		float fFy;	
		//!Principal point X
		float fCx;	
		//!Principal point Y
		float fCy;	

	} ttfIntrinsicParam;
	//!lens distortion parameter

	typedef struct _ttfDistortionParam
	{
		//!Distortion K1
		float fK1;	
		//!Distortion K2
		float fK2;	
		//!Distortion K3
		float fK3;	
		//!Distortion P1
		float fP1;	
		//!Distortion P2
		float fP2;	


	} ttfDistortionParam;

	//!Extrinsic Parameter(Only Depth-RGB Camera Model)
	typedef struct _ttfExtrinsicParam
	{
		float fR11;
		float fR12;
		float fR13;
		float fR21;
		float fR22;
		float fR23;
		float fR31;
		float fR32;
		float fR33;
		float fTx;
		float fTy;
		float fTz;

	} ttfExtrinsicParam;

	//!Camera Space Coordinates - 3D XYZ Coordinates
	typedef struct _ttfCameraSpacePoint
	{
		//!X Coordinates (unit: m)
		float fX;				
		//!Y Coordinates (unit: m)
		float fY;					
		//!Z Coordinates (unit: m)
		float fZ;					

	} ttfCameraSpacePoint;

	//!Depth Space Coordinates - Pixel X, Pixel Y, Depth
	typedef struct _ttfDepthSpacePoint
	{
		//!X Pixel Point (unit: pixel pos)
		float fPointX;			
		//!Y Pixel Point (unit: pixel pos)
		float fPointY;		
		//!Depth (unit: mm)
		float fDepth;				

	} ttfDepthSpacePoint;

	//!Error Code List
	enum ttfError
	{
		ERROR_NO = 1,
		ERROR_FAIL = 0,
		ERROR_HANDLE = -1,
		ERROR_OPEN = -2,
		ERROR_TIME_OUT = -3,
		ERROR_PARAM = -4,
		ERROR_READ_DATA = -5,
		ERROR_CALLBACK_REG = -6,
	};

	//!Frame Type List
	enum FrameType
	{

		//!Depth Frame
		FRAME_DEPTH_FRAME = 2,				
		//!Point Cloud Frame
		FRAME_XYZI_POINT_CLOUD_FRAME = 3,	
		//!This is just used for number of callback types
		FRAME_TYPE_COUNT = 4				
	};

	extern "C"
	{
		//!Callback Function -  RAW Frame
		typedef std::function<int(const ToFRawFrame*)>			pCallbackToFRawFunc;
		//!Callback Function -  Depth Frame
		typedef std::function<int(const DepthFrame*)>			pCallbackDepthFunc;
		//!Callback Function -  Point Cloud Frame
		typedef std::function<int(const XYZIPointCloudFrame*)>  pCallbackPointCloudFunc;
		//!Callback Function -  RGB Function
		typedef std::function<int(const uint8_t*)>				pCallbackRGB888Func;

		/**
		*
		* @date		2016.11.30
		* @brief	Get device list.
		* @details	Number of connectable devices and information.
		* @param	ttfDeviceInfo pDevInfo - Device information.
		* @param	int nDevCount - Number of connectable devices.
		* @return	void.
		*
		*/
        void ttfGetDeviceList(ttfDeviceInfo *pDevInfo, int32_t &nDevCount);

		/**
		*
		* @date		2016.11.30
		* @brief	Open the device.
		* @details	Open and initialize the selected device.
		* @param	ttfHandle hnd - Device Handle.
		* @param	uint8_t nRGBCam_No - RGB camera identification number(Only for with RGB camera model).
		* @return	true|false Success or Fail.
		*
		*/
        int ttfDeviceOpen(ttfHandle hnd, uint8_t nRGBCam_No = 0);

		/**
		*
		* @date		2016.11.30
		* @brief	Close the device.
		* @details	Close the selected device.
		* @param	ttfHandle hnd - Device Handle.
		* @return	void.
		*
		*/
        void ttfDeviceClose(ttfHandle hnd);

		/**
		*
		* @date		2016.11.30
		* @brief	Start callback function.
		* @details	The registered callback function begins reading the frame type data.
		* @param	ttfHandle hnd - Device Handle.
		* @param	FrameType type - FRAME_RAW_FRAME_PROCESSED = 1, FRAME_DEPTH_FRAME = 2, FRAME_XYZI_POINT_CLOUD_FRAME = 3.
		* @return	true|false Success or Fail.
		*
		*/
        int ttfDeviceStart(ttfHandle hnd, FrameType type);

		/**
		*
		* @date		2016.11.30
		* @brief	Callback stop function.
		* @details	Stop reading device data callback.
		* @param	ttfHandle hnd - Device Handle.
		* @return	true|false Success or Fail.
		*
		*/
        int ttfDeviceStop(ttfHandle hnd);

		/**
		*
		* @date		2016.11.30
		* @brief	Registration of callback function for reading depth data.
		* @details	Register depth data of callback function type.
		* @param	pCallbackDepthFunc p_RegisterFunction - callback function => int Function(DepthFrame* pDepthFrame).
		* @return	true|false Success or Fail.
		*
		*/
        int ttfRegister_Callback_DepthFrame(pCallbackDepthFunc p_RegisterFunction);
	
		/**
		*
		* @date		2016.11.30
		* @brief	Registering callback function for reading point cloud data.
		* @details	Registering point cloud data of type callback function.
		* @param	pCallbackPointCloudFunc p_RegisterFunction - callback function => int Function(XYZIPointCloudFrame* pXYZIPointCloudFrame).
		* @return	true|false Success or Fail.
		*
		*/
        int ttfRegister_Callback_PointCloudFrame(pCallbackPointCloudFunc p_RegisterFunction);

		/**
		*
		* @date		2016.11.30
		* @brief	Clear callback function.
		* @details	Clear registered callback function.
		* @param	ttfHandle hnd - Device Handle.
		* @param	FrameType type - FRAME_DEPTH_FRAME = 2, FRAME_XYZI_POINT_CLOUD_FRAME = 3.
		* @return	true|false Success or Fail.
		*
		*/
        int ttfClearCallback(ttfHandle hnd, FrameType type);

		/**
		*
		* @date		2016.11.30
		* @brief	Read rgb888 data(onlt rgb model).
		* @details	Read the RGB camera data r(8bit), g(8bit), b(8bit) 640x480.
		* @param	ttfHandle hnd - Device Handle.
		* @param	RawDataFramePtr &pRGBData - rgb888 data buffer.
		* @return	true|false Success or Fail.
		*
		*/
        int ttfReadFromDeviceRGB888(ttfHandle hnd, RawDataFramePtr &pRGBData);
		
		/**
		*
		* @date		2016.11.30
		* @brief	Initialize buffer.
		* @details	Initialize depth data buffer.
		* @param	DepthFrame *pDepthFrame - DepthFrame buffer.
		* @return	true|false Success or Fail.
		*
		*/

        DepthFrame *ttfDepthFramebufInit(DepthFrame *pDepthFrame);

		/**
		*
		* @date		2016.11.30
		* @brief	Initialize buffer.
		* @details	Initialize point cloud data buffer.
		* @param	XYZIPointCloudFrame *pXYZIPointCloudFrame - XYZI PointCloud Frame buffer.
		* @return	true|false Success or Fail.
		*
		*/
        XYZIPointCloudFrame *ttfXYZIPointCloudFramebufInit(XYZIPointCloudFrame *pXYZIPointCloudFrame);
		
		/**
		*
		* @date		2016.11.30
		* @brief	Set status.
		* @details	Set illumination On/Off status.
		* @param	ttfHandle hnd - Device Handle.
		* @param	bool bOnOff - true(on), false(off).
		* @return	true|false Success or Fail.
		*
		*/
        int ttfSetIllumOnOff(ttfHandle hnd, bool bOnOff);

		/**
		*
		* @date		2016.11.30
		* @brief	Get status.
		* @details	Get illumination On/Off status.
		* @param	ttfHandle hnd - Device Handle.
		* @return	true|false On or Off.
		*
		*/
        bool ttfGetIllumOnOff(ttfHandle hnd);

		/**
		*
		* @date		2016.11.30
		* @brief	Get frame rate.
		* @details	Get the frame rate of the current device.
		* @param	ttfHandle hnd - Device Handle.
		* @param	fFps - Frame Rate(5~60).
		* @return	true|false Success or Fail.
		*
		*/
        int ttfGetFrameRate(ttfHandle hnd, float &fFps);

		/**
		*
		* @date		2016.11.30
		* @brief	Get Serial Number.
		* @details	Get the device serial number.
		* @param	ttfHandle hnd - Device Handle.
		* @param	strSerial Number - serial number.
		* @return	true|false Success or Fail.
		*
		*/		
        int ttfGetSerialNumber(ttfHandle hnd, String &strSerialNumber);

		/**
		*
		* @date		2016.11.30
		* @brief	Get temperature.
		* @details	Get the device temperature.
		* @param	ttfHandle hnd - Device Handle.
		* @param	float &fTIllum - temperature.
		* @return	true|false Success or Fail.
		*
		*/	
        int ttfGetTIllum(ttfHandle hnd, float &fTIllum);

		/**
		*
		* @date		2016.11.30
		* @brief	Get Unambiguous Range.
		* @details	Get the Unambiguous Range.
		* @param	ttfHandle hnd - Device Handle.
		* @param	uint &nUnambiguousRange - Unambiguous Range.
		* @return	true|false Success or Fail.
		*
		*/			
        int ttfGetUnambiguousRange(ttfHandle hnd, uint &nUnambiguousRange);

		/**
		*
		* @date		2016.11.30
		* @brief	Get modulation frequency1.
		* @details	Get the modulation frequency1.
		* @param	ttfHandle hnd - Device Handle.
		* @param	float &fFrequency - modulation frequency1.
		* @return	true|false Success or Fail.
		*
		*/	
        int ttfGetModFreq1(ttfHandle hnd, float &fFrequency);


		
		/**
		*
		* @date		2016.11.30
		* @brief	Get modulation frequency2.
		* @details	Get the modulation frequency2.
		* @param	ttfHandle hnd - Device Handle.
					float &fFrequency - modulation frequency2.
		* @return	true|false Success or Fail.
		*.
		*/	
        int ttfGetModFreq2(ttfHandle hnd, float &fFrequency);

		/**
		*
		* @date		2016.11.30
		* @brief	Set integration time.
		* @details	Set the integration time(If you change the default value, it is affects the exposure time & phase offset).
		* @param	ttfHandle hnd - Device Handle.
		* @param	uint nIntegrationTime - Integration time(0% ~ 31%).
		* @return	true|false Success or Fail.
		*
		*/	
        int ttfSetIntgDutyTime(ttfHandle hnd, uint nIntegrationTime);

		/**
		*
		* @date		2016.11.30
		* @brief	Get integration time.
		* @details	Get the integration time.
		* @param	ttfHandle hnd - Device Handle.
		* @param	nIntegrationTime - Integration time(0% ~ 31%).
		* @return	true|false Success or Fail.
		*
		*/	
        int ttfGetIntgDutyTime(ttfHandle hnd, uint &nIntegrationTime);

		/**
		*
		* @date		2017.09.26
		* @brief	Set Median Filter.
		* @details	Set 3x3 median filter on depth image.
		* @param	ttfHandle hnd - Device Handle.
		* @param	bool bOnOff - Enable/Disable filter.
		* @param	int nKernelSize - half kernel size which defines a kernel of size of 2n + 1 (The default value is 1).
		* @param	float fDeadband - pixel deaband (dont's change default value, The default value is 0.0f).
		* @param	float fDeadbandStep - pixel deadband step (dont's change default value, The default value is 0.0f).
		* @return	true|false Success or Fail.
		*
		*/
        int ttfSetMedianFilter(ttfHandle hnd, bool bOnOff, int nKernelSize = 1, float fDeadband = 0.0f, float fDeadbandStep = 0.0f);

        /**
        *
        * @date		2017.09.26
        * @brief	Get Median Filter.
        * @details	Get smooth filter status.
        * @param	ttfHandle hnd - Device Handle.
        * @return	true|false Success or Fail.
        *
        */
        int ttfGetMedianFilter(ttfHandle hnd);

		/**
		*
		* @date		2018.01.24
		* @brief	Set smooth filter.
		* @details	Set 3x3 mean spatial filter on depth image.
		* @param	ttfHandle hnd - Device Handle.
		* @param	bool bOnOff - Enable/Disable filter.
		* @param	float fSigma - Sigma size (The default value is 0.3f).
		* @return	true|false Success or Fail.
		*
		*/
        int ttfSetSmoothFilter(ttfHandle hnd, bool bOnOff, float fSigma = 0.3f);

        /**
        *
        * @date		2018.01.24
        * @brief	Get smooth filter status.
        * @details	Get filter status.
        * @param	ttfHandle hnd - Device Handle.
        * @return	true|false on or off.
        *
        */
        int ttfGetSmoothFilter(ttfHandle hnd);

		/**
		*
		* @date		2016.11.30
		* @brief	Set IIR filter.
		* @details	Set IIR filter on depth image.
					The infinite-impulse-response filter (IIR) is a simple first order filter of the form:
					Xnew = (1 - a) Xold + a Xin(1).
					The filter responds to change slower (stronger filter) for larger ес, but this will also lead to
					greater motion blur. Therefore, IIR filter is better suited for static scenes, such as 3D scanning applications.
		* @param	ttfHandle hnd - Device Handle.
		* @param	bool bOnOff - enable/disable filter.
		* @param	float fGain - gain (The default value is 0.5f).
		* @return	true|false Success or Fail.
		*
		*/
        int ttfSetIIRFilter(ttfHandle hnd, bool bOnOff, float fGain = 0.5f);

        /**
        *
        * @date		2018.01.24
        * @brief	Get IIR filter status.
        * @details	Get filter status.
        * @param	ttfHandle hnd - Device Handle.
        * @return	true|false on or off.
        *
        */
        int ttfGetIIRFilter(ttfHandle hnd);

		/**
		*
		* @date		2018.01.24
		* @brief	Set Flying Pixel Removal filter.
		* @details	Set the Flying Pixel Removal filter from the depth image.
		* @param	ttfHandle hnd - Device Handle.
		* @param	bool bOnOff - Enable/Disable filter.
		* @param	float thr - Remove depth threshold(The default value is 1500.0f).
		* @return	true|false Success or Fail.
		*
		*/
        int ttfSetFlyPixFilter(ttfHandle hnd, bool bOnOff, float thr = 1500.0f);

        /**
        *
        * @date		2018.01.24
        * @brief	Get FlyPixel filter status.
        * @details	Get filter status.
        * @param	ttfHandle hnd - Device Handle.
        * @return	true|false on or off.
        *
        */
        int ttfGetFlyPixFilter(ttfHandle hnd);
		
		/**
		*
		* @date		2018.01.24
        * @brief	Setting the Temporal filter.
        * @details	Setting the Temporal filter on depth image.
		* @param	ttfHandle hnd - Device Handle.
		* @param	bool bOnOff - Enable/Disable filter.
		* @return	true|false Success or Fail.
		*
		*/
        int ttfSetTemporalFilter(ttfHandle hnd, bool bOnOff);

        /**
        *
        * @date		2018.01.24
        * @brief	Get Temporal filter status.
        * @details	Get filter status.
        * @param	ttfHandle hnd - Device Handle.
        * @return	true|false on or off.
        *
        */
        int ttfGetTemporalFilter(ttfHandle hnd);

		/**
		*
		* @date		2016.11.30
		* @brief	Software offset.
		* @details	Set the software offset value.
		* @param	ttfHandle hnd - Device Handle.
		* @param	int8_t nValue - software offset value(unit: raw phase, -127~127).
		* @return	true|false Success or Fail.
		*
		*/	
        int ttfSetPhaseOffset(ttfHandle hnd, int8_t nValue);
		
		/**
		*
		* @date		2016.11.30
		* @brief	Software offset.
		* @details	Get the software offset value.
		* @param	ttfHandle hnd - Device Handle.
		* @param	int8_t &nValue - software offset value(unit: raw phase, -127~127).
		* @return	true|false Success or Fail.
		*
		*/	
        int ttfGetPhaseOffset(ttfHandle hnd, int8_t &nValue);

		/**
		*
		* @date		2016.11.30
		* @brief	Amplitude strength check threshold settings(small signal removal threshold).
		* @details	This method sets the threshold minimum IR level and used during 
					small signal removal. When the IR signal is below the threshold, 
					the output depth value is set to the 0.
		* @param	ttfHandle hnd - Device Handle.
		* @param	uint8_t nValue - threshold value(0~255).
		* @return	true|false Success or Fail.
		*
		*/
        int ttfSetAmplitudeCheckTh(ttfHandle hnd, uint8_t nValue);

		/**
		*
		* @date		2016.11.30
		* @brief	Get threshold.
		* @details	Get the current amplitude strength check threshold value.
		* @param	ttfHandle hnd - Device Handle.
		* @param	nValue - threshold value(0~255).
		* @return	true|false Success or Fail.
		*
		*/
        int ttfGetAmplitudeCheckTh(ttfHandle hnd, uint8_t &nValue);

		/**
		*
		* @date		2016.11.30
		* @brief	Set threshold.
		* @details	Set the scattering check threshold value.
		* @param	ttfHandle hnd - Device Handle.
		* @param	uint8_t nValue - threshold value(0~255).
		* @return	true|false Success or Fail.
		*
		*/
        int ttfSetScatteringCheckTh(ttfHandle hnd, uint8_t nValue);

		/**
		*
		* @date		2016.11.30
		* @brief	Get threshold.
		* @details	Get the scattering check threshold value.
		* @param	ttfHandle hnd - Device Handle.
		* @param	uint8_t nValue - threshold value(0~255).
		* @return	true|false Success or Fail.
		*
		*/
        int ttfGetScatteringCheckTh(ttfHandle hnd, uint8_t &nValue);

		/**
		*
		* @date		2016.11.30
		* @brief	Get parameter.
		* @details	Get the depth lens intrinsic parameter.
		* @param	ttfHandle hnd - Device Handle.
		* @param	ttfIntrinsicParam *stIntrinsicParam - Depth lens intrinsic parameter structure.
		* @param	ttfDistortionParam *stDistortionParam - Depth lens distortion parameter structure.
		* @return	true|false Success or Fail.
		*
		*/
        int ttfGetLensParameter(ttfHandle hnd, ttfIntrinsicParam *stIntrinsicParam, ttfDistortionParam *stDistortionParam);

		/**
		*
		* @date		2016.11.30
		* @brief	Get parameter.
		* @details	Get the RGB lens intrinsic parameter.
		* @param	ttfHandle hnd - Device Handle.
		* @param	ttfIntrinsicParam *stIntrinsicParam - RGB lens intrinsic parameter structure.
		* @param	ttfDistortionParam *stDistortionParam - RGB lens distortion parameter structure.
		* @return	true|false Success or Fail.
		*
		*/
        int ttfGetRGBLensParameter(ttfHandle hnd, ttfIntrinsicParam *stIntrinsicParam, ttfDistortionParam *stDistortionParam);

		/**
		*
		* @date		2018.04.24
		* @brief	Get parameter.
		* @details	Get Depth-RGB lens extrinsic parameter.
		* @param	ttfHandle hnd - Device Handle.
		* @param	ttfExtrinsicParam *stExtrinsicParam - Depth-RGB lens extrinsic parameter structure.
		* @return	true|false Success or Fail.
		*
		*/
        int ttfGetExtrinsicParameter(ttfHandle hnd, ttfExtrinsicParam *stExtrinsicParam);

		/**
		*
		* @date		2018.04.24
		* @brief	Set parameter.
		* @details	Set Depth-RGB lens extrinsic parameter.
		* @param	ttfHandle hnd - Device Handle.
		* @param	ttfExtrinsicParam stExtrinsicParam - Depth-RGB lens extrinsic parameter structure.
		* @return	true|false Success or Fail.
		*
		*/
        int ttfSetExtrinsicParameter(ttfHandle hnd, ttfExtrinsicParam stExtrinsicParam);

		/**
		*
		*	@date			2018.04.24
		*	@brief			Get parameter.
		*	@description	Get reverse extrinsic parameter.
		*	@param			ttfExtrinsicParam ExtrinsicParam - input extrinsic parameter.
		*	@return			ttfExtrinsicParam - reverse extrinsic parameter(int).
		*
		*/
        ttfExtrinsicParam ttfGetRevers(ttfExtrinsicParam ExtrinsicParam);

		/**
		*
		*	@date			2018.04.24
		*	@brief			Convert coordinate parameter.
		*	@description	Coordinates conversion with extrinsic parameter.
		*	@param			ttfCameraSpacePoint pInCameraPoint - Input X,Y,Z coordinates(unit:m)
		*	@param			ttfCameraSpacePoint pOutCameraPoint - Output X,Y,Z coordinates(unit:m)
		*	@param			ttfExtrinsicParam ExtrinsicParam - Depth-RGB extrinsic parameter
		*	@return			void
		*
		*/
        void ttfXYZ3DConversion(ttfCameraSpacePoint pInCameraPoint, ttfCameraSpacePoint *pOutCameraPoint, ttfExtrinsicParam ExtrinsicParam);

		/**
		*
		*	@date			2018.04.24
		*	@brief			Set image.
		*	@description	Set Undistort 2D(Depth or Z) Image(320x240).
		*	@param			ttfHandle hnd - Device Handle.
		*	@param			bool bEnable - Enable/Disable function.
		*	@return			true|false Success or Fail.
		*
		*/
        int ttfSetUndistort_2d(ttfHandle hnd, bool bEnable);

		/**
		*
		*	@date			2018.04.24
		*	@brief			Set RGB888 image.
		*	@description	Set undistort RGB888 image(640x480).
		*	@param			ttfHandle hnd - Device Handle.
		*	@param			bool bEnable - Enable/Disable function.
		*	@return			true|false Success or Fail.
		*
		*/
        int ttfSetUndistort_rgb888(ttfHandle hnd, bool bEnable);

		/**
		*
		*	@date			2018.04.24
		*	@brief			Convert screen point.
		*	@description	Convert camera space point to depth screen point.
		*	@param			ttfCameraSpacePoint pCameraSpacePoint - Camera space point.
		*	@param			ttfDepthSpacePoint *pDepthPoint - Depth screen point.
		*	@param			TTF_API::ttfIntrinsicParam IntrinsicParam - Intrinsic parameter.
		*	@return			true|false Success or Fail.
		*
		*/	
        void ttfCameraSpacePointToDepth(ttfCameraSpacePoint pCameraSpacePoint, ttfDepthSpacePoint *pDepthPoint, TTF_API::ttfIntrinsicParam IntrinsicParam);

		/**
		*
		*	@date			2018.04.24
		*	@brief			Convert camera space point.
		*	@description	Convert depth screen point to camera space point.
		*	@param			ttfDepthSpacePoint pDepthPoint - Depth screen point.
		*	@param			ttfCameraSpacePoint pCameraSpacePoint - Camera space point.
		*	@param			TTF_API::ttfIntrinsicParam IntrinsicParam - Intrinsic parameter.
		*	@return			true|false Success or Fail.
		*
		*/	
        void ttfDepthPointToCameraSpace(ttfDepthSpacePoint pDepthPoint, ttfCameraSpacePoint *pCameraSpacePoint, TTF_API::ttfIntrinsicParam IntrinsicParam);

		/**
		*
		*	@date			2018.09.10
		*	@brief			Setting Depth range.
		*	@description	Setting the minimum depth and maximum depth range.
		*	@param			ttfHandle hnd - Device Handle.
		*	@param			uint16_t nMinDepth - Minimum depth(unit: mm).
		*	@param			uint16_t nMaxDepth - Maximum depth(unit: mm).
		*	@return			true|false Success or Fail.
		*
		*/
        int ttfSetDepthRange(ttfHandle hnd, uint16_t nMinDepth, uint16_t nMaxDepth);

		/**
		*
		*	@date			2018.09.10
		*	@brief			Get the Depth range
		*	@description	Get the minimum depth & maximum depth range
		*	@param			ttfHandle hnd - Device Handle
		*	@param			uint16_t &nMinDepth - Minimum depth(unit: mm)
		*	@param			uint16_t &nMaxDepth - Maximum depth(unit: mm)
		*	@return			true|false Success or Fail
		*
		*/
        int ttfGetDepthRange(ttfHandle hnd, uint16_t &nMinDepth, uint16_t &nMaxDepth);

	}

}

#endif
