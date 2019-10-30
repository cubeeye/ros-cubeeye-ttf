/*
* TI Voxel Lib component.
*
* Copyright (c) 2014 Texas Instruments Inc.
*/

#ifndef VOXEL_DEPTHCAMERA_H
#define VOXEL_DEPTHCAMERA_H

#include <Device.h>
#include <Parameter.h>
#include <Frame.h>
#include "VideoMode.h"
#include <FrameBuffer.h>

#include <RegisterProgrammer.h>
#include <Streamer.h>

#include <PointCloudTransform.h>

#include <Filter/FilterSet.h>
#include "FrameStream.h"
#include "FrameGenerator.h"
#include "PointCloudFrameGenerator.h"
#include "Configuration.h"

#define CALIB_SECT_LENS "lens"
#define CALIB_SECT_LENS_ID 0

namespace Voxel
{

	/**
	* \ingroup CamSys
	*
	* \brief This is primary class which provides API for a depth camera.
	*
	* DepthCamera is an abstract class which needs to be derived and implemented
	* for individual depth camera types.
	*/
	class VOXEL_EXPORT DepthCamera
	{
	public:
		enum FrameType
		{
			FRAME_RAW_FRAME_UNPROCESSED = 0,
			FRAME_RAW_FRAME_PROCESSED = 1,
			FRAME_DEPTH_FRAME = 2,
			FRAME_XYZI_POINT_CLOUD_FRAME = 3,
			FRAME_TYPE_COUNT = 4 // This is just used for number of callback types
		};


		//Lens Parameter Infomation(support to new model - later 2016.07~)
		typedef struct _ttfDistortionParam
		{
			float fK1;
			float fK2;
			float fK3;
			float fP1;
			float fP2;
			float fSkew;

		} ttfDistortionParam;

		typedef struct _ttfIntrinsicParam
		{
			float fFx;
			float fFy;
			float fCx;
			float fCy;

		} ttfIntrinsicParam;

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

		typedef Function<void(DepthCamera &camera, const Frame &frame, FrameType callBackType)> CallbackType;

	private:
		mutable Mutex _accessMutex; // This is locked by getters and setters which are public
		mutable Mutex _frameStreamWriterMutex;

	protected:
		DevicePtr _device;

		String _name, _id, _chipset;

		Map<String, ParameterPtr> _parameters;

		Ptr<RegisterProgrammer> _programmer;
		Ptr<Streamer> _streamer;

		// NOTE: Constructors of derived classes need to initialize the first two entries in this array
		FrameGeneratorPtr _frameGenerators[3];

		Ptr<PointCloudFrameGenerator> _pointCloudFrameGenerator;

		bool _parameterInit;

		FrameBufferManager<RawFrame> _rawFrameBuffers;
		FrameBufferManager<DepthFrame> _depthFrameBuffers;
		FrameBufferManager<PointCloudFrame> _pointCloudBuffers;

		FilterSet<RawFrame> _unprocessedFilters, _processedFilters;

		FilterSet<DepthFrame> _depthFilters;

		FrameStreamWriterPtr _frameStreamWriter;

		bool _addParameters(const Vector<ParameterPtr> &params);

		CallbackType _callback[FRAME_TYPE_COUNT];

		uint32_t _callBackTypesRegistered = 0;

		ThreadPtr _captureThread;

		// Callback the registered function for 'type' if present and decide whether continue processing or not
		virtual bool _callbackAndContinue(uint32_t &callBackTypesToBeCalled, FrameType type, const Frame &frame);

		virtual bool _start() = 0;
		virtual bool _stop() = 0;

		virtual bool _captureRawUnprocessedFrame(RawFramePtr &rawFrame) = 0;
		virtual bool _processRawFrame(const RawFramePtr &rawFrameInput, RawFramePtr &rawFrameOutput) = 0; // here output raw frame will have processed data, like ToF data for ToF cameras
		virtual bool _convertToDepthFrame(const RawFramePtr &rawFrame, DepthFramePtr &depthFrame) = 0;
		virtual bool _convertToPointCloudFrame(const DepthFramePtr &depthFrame, PointCloudFramePtr &pointCloudFrame);
		virtual bool _UndistortionFrame(const PointCloudFramePtr &PCLInputFrame, PointCloudFramePtr &PCLOutFrame);
		
		//virtual bool _convertToPointCloudFrame(const RawFramePtr &depthFrame, PointCloudFramePtr &pointCloudFrame);//meere test
		

		virtual void _captureLoop(); // the main capture loop

		void _captureThreadWrapper(); // this is non-virtual and simply calls _captureLoop

		bool _running, _isPaused; // is capture running?

		bool _writeToFrameStream(RawFramePtr &rawUnprocessed);

		// These protected getters and setters are not thread-safe. These are to be directly called only when nested calls are to be done from getter/setter to another. 
		// Otherwise use the public functions
		template <typename T>
		bool _get(const String &name, T &value, bool refresh = false) const;

		template <typename T>
		bool _set(const String &name, const T &value);

		//meere add - ykw
		virtual bool _getSensorTemp(float &temp) const = 0;

		virtual bool _setStandardTempParam(const float &fStandardTempeature, const float &fChangeRate) = 0;
		virtual bool _getStandardTempParam(float &fStandardTempeature, float &fChangeRate) const = 0;

		virtual bool _setFppnEnable(const bool &bEnable) = 0;
		virtual bool _getFppnEnable(bool &bEnable) const = 0;

		virtual bool _setWigglingEnable(const bool &bEnable) = 0;
		virtual bool _getWigglingEnable(bool &bEnable) const = 0;

		virtual bool _setWigglingData(const uint8_t *pWigglingData, const uint8_t &nCh) = 0;
		virtual bool _getWigglingData(uint8_t *pWigglingData, uint8_t &nCh) const = 0;

		virtual bool _setPhaseOffset(const int8_t &nValue) = 0;
		virtual bool _getPhaseOffset(int8_t &nValue) const = 0;

		virtual bool _setPhaseOffset1(const int8_t &nValue) = 0;
		virtual bool _getPhaseOffset1(int8_t &nValue) const = 0;
		virtual bool _setPhaseOffset2(const int8_t &nValue) = 0;
		virtual bool _getPhaseOffset2(int8_t &nValue) const = 0;
		virtual bool _setPhaseOffset3(const int8_t &nValue) = 0;
		virtual bool _getPhaseOffset3(int8_t &nValue) const = 0;

		virtual bool _setNegativeVoltage(const uint8_t &nValue) = 0;
		virtual bool _getNegativeVoltage(uint8_t &nValue) const = 0;

		virtual bool _setEdgeCheckTh(const uint8_t &nValue) = 0;
		virtual bool _getEdgeCheckTh(uint8_t &nValue) const = 0;

		virtual bool _setScatteringCheckTh(const uint8_t &nValue) = 0;
		virtual bool _getScatteringCheckTh(uint8_t &nValue) const = 0;

		virtual bool _setScatteringCheckTh2(const uint8_t &nValue, const uint8_t &nValue2 = 50, const uint16_t &nHoriLine = 240, const uint16_t &nVertLine = 320) = 0;
		virtual bool _getScatteringCheckTh2(uint8_t &nValue, uint8_t &nValue2, uint16_t &nHoriLine, uint16_t &nVertLine) const = 0;
		
		virtual bool _setAmplitudeCheckTh(const uint8_t &nValue) = 0;
		virtual bool _getAmplitudeCheckTh(uint8_t &nValue) const = 0;

        virtual bool _setAmplitudeCheckTh_ROI(const uint8_t &Weight, const uint16_t &DepthTh,const uint16_t &x1, const uint16_t &x2, const uint16_t &y1, const uint16_t &y2) = 0;
        virtual bool _getAmplitudeCheckTh_ROI(uint8_t &Weight, uint16_t &DepthTh,uint16_t &x1, uint16_t &x2, uint16_t &y1, uint16_t &y2) const = 0;

		virtual bool _setPtoDSkipEnable(const bool &bEnable) = 0;
		virtual bool _getPtoDSkipEnable(bool &bEnable) const = 0;

		virtual bool _setDepthRange(const uint16_t &nMinDepth, const uint16_t &nMaxDepth) = 0;
		virtual bool _getDepthRange(uint16_t &nMinDepth, uint16_t &nMaxDepth) const = 0;

		virtual bool _getFX2FWVersion(uint16_t &nValue) const = 0;
		virtual bool _getTFCFWVersion(uint32_t &nValue) const = 0;

		virtual bool _setTempOffsetParam(const uint8_t &nTempOffsetPixel, const float &fTempOffsetRate) = 0;
		virtual bool _getTempOffsetParam(uint8_t &nTempOffsetPixel, float &fTempOffsetRate) const = 0;

		virtual bool _setProperty(uint16_t &address, uint16_t &length, const uint8_t &nValue) = 0;
		virtual bool _getProperty(uint16_t &address, uint16_t &length, uint8_t &nValue) const = 0;

		virtual bool _setProperty(uint16_t &address, uint16_t &length, const uint16_t &nValue) = 0;
		virtual bool _getProperty(uint16_t &address, uint16_t &length, uint16_t &nValue) const = 0;

		//**** Lens Parameter ****//
		virtual bool _setLensParameter(const ttfIntrinsicParam &stIntrinsicParam, const ttfDistortionParam &stDistortionParam) = 0;
		virtual bool _getLensParameter(ttfIntrinsicParam &stIntrinsicParam, ttfDistortionParam &stDistortionParam) const = 0;

		virtual bool _setRGBLensParameter(const ttfIntrinsicParam &stIntrinsicParam, const ttfDistortionParam &stDistortionParam) = 0;
		virtual bool _getRGBLensParameter(ttfIntrinsicParam &stIntrinsicParam, ttfDistortionParam &stDistortionParam) const = 0;

		virtual bool _setExtrinsicParameter(const ttfExtrinsicParam &stExtrinsicParam) = 0;
		virtual bool _getExtrinsicParameter(ttfExtrinsicParam &stExtrinsicParam) const = 0;

		virtual bool _setPixelDegree(const float &fPixelDegree1, const float &fPixelDegree2, const float &fPixelDegree3) = 0;
		virtual bool _getPixelDegree(float &fPixelDegree1, float &fPixelDegree2, float &fPixelDegree3) const = 0;

		//**** only fpga version ****//
		virtual bool _setFPGAHDRFilter(const bool &bEnable) = 0;
		virtual bool _getFPGAHDRFilter(bool &bEnable) const = 0;

		virtual bool _setFPGAEdgeCheckTh(const uint8_t &nValue) = 0;
		virtual bool _getFPGAEdgeCheckTh(uint8_t &nValue) const = 0;

		virtual bool _setFPGAAmplitudeCheckTh(const uint8_t &nValue) = 0;
		virtual bool _getFPGAAmplitudeCheckTh(uint8_t &nValue) const = 0;

		virtual bool _setFPGAScatteringCheckTh(const uint8_t &nValue) = 0;
		virtual bool _getFPGAScatteringCheckTh(uint8_t &nValue) const = 0;

		virtual bool _setFPGAWigglingEnable(const bool &bEnable) = 0;
		virtual bool _getFPGAWigglingEnable(bool &bEnable) const = 0;

		virtual bool _setFPGAFPPNEnable(const bool &bEnable) = 0;
		virtual bool _getFPGAFPPNEnable(bool &bEnable) const = 0;

		virtual bool _saveFPGAParameter(const uint8_t *pData) = 0;
		virtual bool _loadFPGAParameter(uint8_t *pData) const = 0;

		virtual bool _setFPGAWigglingData(const uint8_t *pData, const uint8_t &nCh) = 0;
		virtual bool _getFPGAWigglingData(uint8_t *pData, uint8_t &nCh) const = 0;

		virtual bool _setFPGAFppnData(const uint8_t *pData) = 0;
		virtual bool _getFPGAFppnData(uint8_t *pData) const = 0;

		virtual bool _setFPGAOrthogonalData(const uint8_t *pData) = 0;
		virtual bool _getFPGAOrthogonalData(uint8_t *pData) const = 0;

		//////////////////////////////////////////////////////////////////////////////////////////////////////

		virtual bool _setFrameRate(const FrameRate &r) = 0;
		virtual bool _getFrameRate(FrameRate &r) const = 0;

		virtual bool _setFrameSize(const FrameSize &s) = 0;
		virtual bool _getFrameSize(FrameSize &s) const = 0;
		virtual bool _getMaximumFrameSize(FrameSize &s) const = 0;
		virtual bool _getMaximumFrameRate(FrameRate &frameRate, const FrameSize &forFrameSize) const = 0;
		virtual bool _getSupportedVideoModes(Vector<SupportedVideoMode> &supportedVideoModes) const = 0;
		virtual bool _getMaximumVideoMode(VideoMode &videoMode) const = 0;

		virtual bool _getBytesPerPixel(uint &bpp) const = 0;
		virtual bool _setBytesPerPixel(const uint &bpp) = 0;


		virtual bool _getROI(RegionOfInterest &roi) const = 0;
		virtual bool _setROI(const RegionOfInterest &roi) = 0;
		virtual bool _allowedROI(String &message) = 0;

		virtual bool _getFieldOfView(float &fovHalfAngle) const = 0;

		inline void _makeID() { _id = _name + "(" + _device->id() + ")"; }

		virtual bool _reset() = 0;
		virtual bool _onReset() = 0;

		virtual bool _applyConfigParams(const ConfigSet *params);

		virtual bool _saveCurrentProfileID(const int id) = 0;
		virtual bool _getCurrentProfileID(int &id) = 0;

		bool _init();

		inline Map<String, CalibrationInformation> &_getCalibrationInformationStructure() { return configFile._calibrationInformation; }

	public:
		MainConfigurationFile configFile; // This corresponds to camera specific configuration file

		DepthCamera(const String &name, const String &chipset, DevicePtr device);

		uint _nIntgTimeFlag;
		uint8_t n2dUndistortion;


		virtual bool isInitialized() const
		{
			return _programmer && _programmer->isInitialized() &&
				_streamer && _streamer->isInitialized() && _parameterInit;
		}

		inline const String &name() const { return _name; }

		inline const String &id() const { return _id; }

		inline const String &chipset() const { return _chipset; }

		virtual bool getSerialNumber(String &serialNumber) const;
		virtual bool setSerialNumber(const String &serialNumber);

		inline const DevicePtr &getDevice() const { return _device; }

		inline bool isRunning() const { return _running; }

		inline bool isPaused() const { return _isPaused; }

		bool pause();
		bool resume();

		template <typename T>
		bool getStreamParam(const String &name, T &value) const;

		bool refreshParams();

		template <typename T>
		bool get(const String &name, T &value, bool refresh = false) const;

		template <typename T>
		bool set(const String &name, const T &value);

		// WARNING: Avoid using get() and set() on ParameterPtr, obtained via getParam() or getParameters(). It is not thread-safe. Instead use get() and set() on DepthCamera
		inline const ParameterPtr getParam(const String &name) const;
		inline const Map<String, ParameterPtr> &getParameters() const { return _parameters; }

		//meere add - ykw
		inline bool getSensorTemp(float &temp) const;

		inline bool setStandardTempParam(const float &fStandardTempeature, const float &fChangeRate);
		inline bool getStandardTempParam(float &fStandardTempeature, float &fChangeRate) const;

		inline bool setFppnEnable(const bool &bEnable);
		inline bool getFppnEnable(bool &bEnable) const;

		inline bool setWigglingEnable(const bool &bEnable);
		inline bool getWigglingEnable(bool &bEnable) const;

		inline bool setWigglingData(const uint8_t *pWigglingData, const uint8_t &nCh);
		inline bool getWigglingData(uint8_t *pWigglingData, uint8_t &nCh) const;

		inline bool setPhaseOffset(const int8_t &nValue);
		inline bool getPhaseOffset(int8_t &nValue) const;

		inline bool setPhaseOffset1(const int8_t &nValue);
		inline bool getPhaseOffset1(int8_t &nValue) const;
		inline bool setPhaseOffset2(const int8_t &nValue);
		inline bool getPhaseOffset2(int8_t &nValue) const;
		inline bool setPhaseOffset3(const int8_t &nValue);
		inline bool getPhaseOffset3(int8_t &nValue) const;

		inline bool setNegativeVoltage(const uint8_t &nValue);
		inline bool getNegativeVoltage(uint8_t &nValue) const;

		inline bool setEdgeCheckTh(const uint8_t &nValue);
		inline bool getEdgeCheckTh(uint8_t &nValue) const;

		inline bool setScatteringCheckTh(const uint8_t &nValue);
		inline bool getScatteringCheckTh(uint8_t &nValue) const;

		inline bool setScatteringCheckTh2(const uint8_t &nValue, const uint8_t &nValue2 = 50, const uint16_t &nHoriLine = 240, const uint16_t &nVertLine = 320);
		inline bool getScatteringCheckTh2(uint8_t &nValue, uint8_t &nValue2, uint16_t &nHoriLine, uint16_t &nVertLine) const;

		inline bool setAmplitudeCheckTh(const uint8_t &nValue);
		inline bool getAmplitudeCheckTh(uint8_t &nValue) const;

        inline bool setAmplitudeCheckTh_ROI(const uint8_t &Weight, const uint16_t &DepthTh,const uint16_t &x1, const uint16_t &x2, const uint16_t &y1, const uint16_t &y2);
        inline bool getAmplitudeCheckTh_ROI(uint8_t &Weight, uint16_t &DepthTh, uint16_t &x1, uint16_t &x2, uint16_t &y1, uint16_t &y2) const;

		inline bool setPtoDSkipEnable(const bool &bEnable);
		inline bool getPtoDSkipEnable(bool &bEnable) const;

		inline bool setLensParameter(const ttfIntrinsicParam &stIntrinsicParam, const ttfDistortionParam &stDistortionParam);
		inline bool getLensParameter(ttfIntrinsicParam &stIntrinsicParam, ttfDistortionParam &stDistortionParam) const;

		inline bool setRGBLensParameter(const ttfIntrinsicParam &stIntrinsicParam, const ttfDistortionParam &stDistortionParam);
		inline bool getRGBLensParameter(ttfIntrinsicParam &stIntrinsicParam, ttfDistortionParam &stDistortionParam) const;

		inline bool setExtrinsicParameter(const ttfExtrinsicParam &stExtrinsicParam);
		inline bool getExtrinsicParameter(ttfExtrinsicParam &stExtrinsicParam) const;

		inline bool setPixelDegree(const float &fPixelDegree1, const float &fPixelDegree2, const float &fPixelDegree3);
		inline bool getPixelDegree(float &fPixelDegree1, float &fPixelDegree2, float &fPixelDegree3) const;

		inline bool setDepthRange(const uint16_t &nMinDepth, const uint16_t &nMaxDepth);
		inline bool getDepthRange(uint16_t &nMinDepth, uint16_t &nMaxDepth) const;

		inline bool getFX2FWVersion(uint16_t &nValue) const;
		inline bool getTFCFWVersion(uint32_t &nValue) const;

		inline bool setTempOffsetParam(const uint8_t &nTempOffsetPixel, const float &fTempOffsetRate);
		inline bool getTempOffsetParam(uint8_t &nTempOffsetPixel, float &fTempOffsetRate) const;

		inline bool setProperty(uint16_t &address, uint16_t &length, const uint8_t &nValue);
		inline bool getProperty(uint16_t &address, uint16_t &length, uint8_t &nValue) const;

		inline bool setProperty(uint16_t &address, uint16_t &length, const uint16_t &nValue);
		inline bool getProperty(uint16_t &address, uint16_t &length, uint16_t &nValue) const;

		//*** only fpga version ***//
		inline bool setFPGAHDRFilter(const bool &bEnable);
		inline bool getFPGAHDRFilter(bool &bEnable) const;

		inline bool setFPGAEdgeCheckTh(const uint8_t &nValue);
		inline bool getFPGAEdgeCheckTh(uint8_t &nValue) const;

		inline bool setFPGAScatteringCheckTh(const uint8_t &nValue);
		inline bool getFPGAScatteringCheckTh(uint8_t &nValue) const;

		inline bool setFPGAAmplitudeCheckTh(const uint8_t &nValue);
		inline bool getFPGAAmplitudeCheckTh(uint8_t &nValue) const;

		inline bool setFPGAWigglingEnable(const bool &bEnable);
		inline bool getFPGAWigglingEnable(bool &bEnable) const;

		inline bool setFPGAFPPNEnable(const bool &bEnable);
		inline bool getFPGAFPPNEnable(bool &bEnable) const;

		inline bool saveFPGAParameter(const uint8_t *pData);
		inline bool loadFPGAParameter(uint8_t *pData) const;

		inline bool setFPGAWigglingData(const uint8_t *pData, const uint8_t &nCh);
		inline bool getFPGAWigglingData(uint8_t *pData, uint8_t &nCh) const;

		inline bool setFPGAFppnData(const uint8_t *pData);
		inline bool getFPGAFppnData(uint8_t *pData) const;

		inline bool setFPGAOrthogonalData(const uint8_t *pData);
		inline bool getFPGAOrthogonalData(uint8_t *pData) const;

		////////////////////////////////////////////////////////////////////////

		inline bool setFrameRate(const FrameRate &r);
		inline bool getFrameRate(FrameRate &r) const;

		inline bool setFrameSize(const FrameSize &s);
		inline bool getFrameSize(FrameSize &s) const;
		inline bool getMaximumFrameSize(FrameSize &s) const;
		inline bool getMaximumFrameRate(FrameRate &frameRate, const FrameSize &forFrameSize) const;
		inline bool getSupportedVideoModes(Vector<SupportedVideoMode> &supportedVideoModes) const;
		inline bool getMaximumVideoMode(VideoMode &videoMode) const;

		inline bool getBytesPerPixel(uint &bpp) const;
		inline bool setBytesPerPixel(const uint &bpp);

		inline bool getROI(RegionOfInterest &roi);
		inline bool setROI(const RegionOfInterest &roi);
		inline bool allowedROI(String &message);

		inline bool getFieldOfView(float &fovHalfAngle) const;

		virtual bool saveFrameStream(const String &fileName);
		virtual bool isSavingFrameStream();
		virtual bool closeFrameStream();

		virtual bool registerCallback(FrameType type, CallbackType f);
		virtual bool clearAllCallbacks();
		virtual bool clearCallback(FrameType type);

		// beforeFilterIndex = -1 => at the end, otherwise at location before the given filter index.
		// Return value: 
		//   >= 0 => add successfully with return value as filter ID.
		//     -1 => failed to add filter
		virtual int addFilter(FilterPtr p, FrameType frameType, int beforeFilterID = -1);
		virtual FilterPtr getFilter(int filterID, FrameType frameType) const;
		virtual bool removeFilter(int filterID, FrameType frameType);
		virtual bool removeAllFilters(FrameType frameType);
		virtual void resetFilters();

		inline const FilterSet<RawFrame> &getUnprocessedRawFilterSet() { return _unprocessedFilters; }
		inline const FilterSet<RawFrame> &getProcessedRawFilterSet() { return _processedFilters; }
		inline const FilterSet<DepthFrame> &getDepthFilterSet() { return _depthFilters; }

		bool start();
		bool stop();

		void wait();

		bool reset();

		inline Ptr<RegisterProgrammer> getProgrammer() { return _programmer; } // RegisterProgrammer is usually thread-safe to use outside directly
		inline Ptr<Streamer> getStreamer() { return _streamer; } // Streamer may not be thread-safe

		inline bool reloadConfiguration() { return configFile.read(_name + ".conf"); }
		inline const Map<int, String> &getCameraProfileNames() { return configFile.getCameraProfileNames(); }
		inline int getCurrentCameraProfileID() { return configFile.getCurrentProfileID(); }

		int addCameraProfile(const String &profileName, const int parentID);
		bool setCameraProfile(const int id, bool softApply = false);
		bool removeCameraProfile(const int id);
		inline bool saveCameraProfileToHardware(int &id, bool saveParents = false, bool setAsDefault = false, const String &namePrefix = "") { return configFile.saveCameraProfileToHardware(id, saveParents, setAsDefault, namePrefix); }

		bool close();
		virtual ~DepthCamera();
	};

	template <typename T>
	bool DepthCamera::get(const String &name, T &value, bool refresh) const
	{
		Lock<Mutex> _(_accessMutex);
		return _get(name, value, refresh);
	}

	template <typename T>
	bool DepthCamera::set(const String &name, const T &value)
	{
		Lock<Mutex> _(_accessMutex);
		return _set(name, value);
	}

	bool DepthCamera::getFieldOfView(float &fovHalfAngle) const
	{
		Lock<Mutex> _(_accessMutex);
		return _getFieldOfView(fovHalfAngle);
	}

	//****** meere add ******//
	bool DepthCamera::getSensorTemp(float &temp) const
	{
		Lock<Mutex> _(_accessMutex);
		return _getSensorTemp(temp);
	}

	bool DepthCamera::setStandardTempParam(const float &fStandardTempeature, const float &fChangeRate)
	{
		Lock<Mutex> _(_accessMutex);
		return _setStandardTempParam(fStandardTempeature, fChangeRate);

	}

	bool DepthCamera::getStandardTempParam(float &fStandardTempeature, float &fChangeRate) const
	{
		Lock<Mutex> _(_accessMutex);
		return _getStandardTempParam(fStandardTempeature, fChangeRate);

	}

	bool DepthCamera::setFppnEnable(const bool &bEnable)
	{
		Lock<Mutex> _(_accessMutex);
		return _setFppnEnable(bEnable);

	}

	bool DepthCamera::getFppnEnable(bool &bEnable) const
	{
		Lock<Mutex> _(_accessMutex);
		return _getFppnEnable(bEnable);

	}

	bool DepthCamera::setWigglingEnable(const bool &bEnable)
	{
		Lock<Mutex> _(_accessMutex);
		return _setWigglingEnable(bEnable);

	}

	bool DepthCamera::getWigglingEnable(bool &bEnable) const
	{
		Lock<Mutex> _(_accessMutex);
		return _getWigglingEnable(bEnable);

	}

	bool DepthCamera::setWigglingData(const uint8_t *pWigglingData, const uint8_t &nCh)
	{
		Lock<Mutex> _(_accessMutex);
		return _setWigglingData(pWigglingData, nCh);

	}

	bool DepthCamera::getWigglingData(uint8_t *pWigglingData, uint8_t &nCh) const
	{
		Lock<Mutex> _(_accessMutex);
		return _getWigglingData(pWigglingData, nCh);

	}

	bool DepthCamera::setPhaseOffset(const int8_t &nValue)
	{
		Lock<Mutex> _(_accessMutex);
		return _setPhaseOffset(nValue);

	}

	bool DepthCamera::getPhaseOffset(int8_t &nValue) const
	{
		Lock<Mutex> _(_accessMutex);
		return _getPhaseOffset(nValue);

	}

	bool DepthCamera::setPhaseOffset1(const int8_t &nValue)
	{
		Lock<Mutex> _(_accessMutex);
		return _setPhaseOffset1(nValue);

	}

	bool DepthCamera::getPhaseOffset1(int8_t &nValue) const
	{
		Lock<Mutex> _(_accessMutex);
		return _getPhaseOffset1(nValue);

	}
	bool DepthCamera::setPhaseOffset2(const int8_t &nValue)
	{
		Lock<Mutex> _(_accessMutex);
		return _setPhaseOffset2(nValue);

	}

	bool DepthCamera::getPhaseOffset2(int8_t &nValue) const
	{
		Lock<Mutex> _(_accessMutex);
		return _getPhaseOffset2(nValue);

	}
	bool DepthCamera::setPhaseOffset3(const int8_t &nValue)
	{
		Lock<Mutex> _(_accessMutex);
		return _setPhaseOffset3(nValue);

	}

	bool DepthCamera::getPhaseOffset3(int8_t &nValue) const
	{
		Lock<Mutex> _(_accessMutex);
		return _getPhaseOffset3(nValue);

	}
	bool DepthCamera::setNegativeVoltage(const uint8_t &nValue)
	{
		Lock<Mutex> _(_accessMutex);
		return _setNegativeVoltage(nValue);

	}

	bool DepthCamera::getNegativeVoltage(uint8_t &nValue) const
	{
		Lock<Mutex> _(_accessMutex);
		return _getNegativeVoltage(nValue);

	}

	bool DepthCamera::setEdgeCheckTh(const uint8_t &nValue)
	{
		Lock<Mutex> _(_accessMutex);
		return _setEdgeCheckTh(nValue);

	}

	bool DepthCamera::getEdgeCheckTh(uint8_t &nValue) const
	{
		Lock<Mutex> _(_accessMutex);
		return _getEdgeCheckTh(nValue);

	}

	bool DepthCamera::setScatteringCheckTh(const uint8_t &nValue)
	{
		Lock<Mutex> _(_accessMutex);
		return _setScatteringCheckTh(nValue);

	}

	bool DepthCamera::getScatteringCheckTh(uint8_t &nValue) const
	{
		Lock<Mutex> _(_accessMutex);
		return _getScatteringCheckTh(nValue);

	}

	bool DepthCamera::setScatteringCheckTh2(const uint8_t &nValue, const uint8_t &nValue2, const uint16_t &nHoriLine, const uint16_t &nVertLine)
	{
		Lock<Mutex> _(_accessMutex);
		return _setScatteringCheckTh2(nValue, nValue2, nHoriLine, nVertLine);

	}

	bool DepthCamera::getScatteringCheckTh2(uint8_t &nValue, uint8_t &nValue2, uint16_t &nHoriLine, uint16_t &nVertLine) const
	{
		Lock<Mutex> _(_accessMutex);
		return _getScatteringCheckTh2(nValue, nValue2, nHoriLine, nVertLine);

	}

	bool DepthCamera::setAmplitudeCheckTh(const uint8_t &nValue)
	{
		Lock<Mutex> _(_accessMutex);
		return _setAmplitudeCheckTh(nValue);

	}

	bool DepthCamera::getAmplitudeCheckTh(uint8_t &nValue) const
	{
		Lock<Mutex> _(_accessMutex);
		return _getAmplitudeCheckTh(nValue);

	}

    bool DepthCamera::setAmplitudeCheckTh_ROI(const uint8_t &Weight, const uint16_t &DepthTh,const uint16_t &x1, const uint16_t &x2, const uint16_t &y1, const uint16_t &y2)
    {
        Lock<Mutex> _(_accessMutex);
        return _setAmplitudeCheckTh_ROI(Weight, DepthTh, x1, x2, y1, y2);
    }

    bool DepthCamera::getAmplitudeCheckTh_ROI(uint8_t &Weight, uint16_t &DepthTh, uint16_t &x1, uint16_t &x2, uint16_t &y1, uint16_t &y2) const
    {
        Lock<Mutex> _(_accessMutex);
        return _getAmplitudeCheckTh_ROI(Weight, DepthTh, x1, x2, y1, y2);
    }

	bool DepthCamera::setPtoDSkipEnable(const bool &bEnable)
	{
		Lock<Mutex> _(_accessMutex);
		return _setPtoDSkipEnable(bEnable);

	}

	bool DepthCamera::getPtoDSkipEnable(bool &bEnable) const
	{
		Lock<Mutex> _(_accessMutex);
		return _getPtoDSkipEnable(bEnable);

	}

	bool DepthCamera::setTempOffsetParam(const uint8_t &nTempOffsetPixel, const float &fTempOffsetRate)
	{
		Lock<Mutex> _(_accessMutex);
		return _setTempOffsetParam(nTempOffsetPixel, fTempOffsetRate);

	}

	bool DepthCamera::getTempOffsetParam(uint8_t &nTempOffsetPixel, float &fTempOffsetRate) const
	{
		Lock<Mutex> _(_accessMutex);
		return _getTempOffsetParam(nTempOffsetPixel, fTempOffsetRate);

	}

	bool DepthCamera::setLensParameter(const ttfIntrinsicParam &stIntrinsicParam, const ttfDistortionParam &stDistortionParam)
	{
		Lock<Mutex> _(_accessMutex);
		return _setLensParameter(stIntrinsicParam, stDistortionParam);
	}

	bool DepthCamera::getLensParameter(ttfIntrinsicParam &stIntrinsicParam, ttfDistortionParam &stDistortionParam) const
	{
		Lock<Mutex> _(_accessMutex);
		return _getLensParameter(stIntrinsicParam, stDistortionParam);
	}

	bool DepthCamera::setRGBLensParameter(const ttfIntrinsicParam &stIntrinsicParam, const ttfDistortionParam &stDistortionParam)
	{
		Lock<Mutex> _(_accessMutex);
		return _setRGBLensParameter(stIntrinsicParam, stDistortionParam);
	}

	bool DepthCamera::getRGBLensParameter(ttfIntrinsicParam &stIntrinsicParam, ttfDistortionParam &stDistortionParam) const
	{
		Lock<Mutex> _(_accessMutex);
		return _getRGBLensParameter(stIntrinsicParam, stDistortionParam);
	}

	bool DepthCamera::setExtrinsicParameter(const ttfExtrinsicParam &stExtrinsicParam)
	{
		Lock<Mutex> _(_accessMutex);
		return _setExtrinsicParameter(stExtrinsicParam);
	}

	bool DepthCamera::getExtrinsicParameter(ttfExtrinsicParam &stExtrinsicParam) const
	{
		Lock<Mutex> _(_accessMutex);
		return _getExtrinsicParameter(stExtrinsicParam);
	}

	bool DepthCamera::setPixelDegree(const float &fPixelDegree1, const float &fPixelDegree2, const float &fPixelDegree3)
	{
		Lock<Mutex> _(_accessMutex);
		return _setPixelDegree(fPixelDegree1, fPixelDegree2, fPixelDegree3);
	}

	bool DepthCamera::getPixelDegree(float &fPixelDegree1, float &fPixelDegree2, float &fPixelDegree3) const
	{
		Lock<Mutex> _(_accessMutex);
		return _getPixelDegree(fPixelDegree1, fPixelDegree2, fPixelDegree3);
	}

	bool DepthCamera::setDepthRange(const uint16_t &nMinDepth, const uint16_t &nMaxDepth)
	{
		Lock<Mutex> _(_accessMutex);
		return _setDepthRange(nMinDepth, nMaxDepth);
	}

	bool DepthCamera::getDepthRange(uint16_t &nMinDepth, uint16_t &nMaxDepth) const
	{
		Lock<Mutex> _(_accessMutex);
		return _getDepthRange(nMinDepth, nMaxDepth);
	}

	bool DepthCamera::getFX2FWVersion(uint16_t &nValue) const
	{
		Lock<Mutex> _(_accessMutex);
		return _getFX2FWVersion(nValue);
	}

	bool DepthCamera::getTFCFWVersion(uint32_t &nValue) const
	{
		Lock<Mutex> _(_accessMutex);
		return _getTFCFWVersion(nValue);
	}

	bool DepthCamera::setProperty(uint16_t &address, uint16_t &length, const uint8_t &nValue)
	{
		Lock<Mutex> _(_accessMutex);
		return _setProperty(address, length, nValue);
	}

	bool DepthCamera::getProperty(uint16_t &address, uint16_t &length, uint8_t &nValue) const
	{
		Lock<Mutex> _(_accessMutex);
		return _getProperty(address, length, nValue);
	}

	bool DepthCamera::setProperty(uint16_t &address, uint16_t &length, const uint16_t &nValue)
	{
		Lock<Mutex> _(_accessMutex);
		return _setProperty(address, length, nValue);
	}

	bool DepthCamera::getProperty(uint16_t &address, uint16_t &length, uint16_t &nValue) const
	{
		Lock<Mutex> _(_accessMutex);
		return _getProperty(address, length, nValue);
	}

	//***** only FPGA version *****//
	bool DepthCamera::setFPGAEdgeCheckTh(const uint8_t &nValue)
	{
		Lock<Mutex> _(_accessMutex);
		return _setFPGAEdgeCheckTh(nValue);
	}

	bool DepthCamera::getFPGAEdgeCheckTh(uint8_t &nValue) const
	{
		Lock<Mutex> _(_accessMutex);
		return _getFPGAEdgeCheckTh(nValue);
	}

	bool DepthCamera::setFPGAAmplitudeCheckTh(const uint8_t &nValue)
	{
		Lock<Mutex> _(_accessMutex);
		return _setFPGAAmplitudeCheckTh(nValue);
	}

	bool DepthCamera::getFPGAAmplitudeCheckTh(uint8_t &nValue) const
	{
		Lock<Mutex> _(_accessMutex);
		return _getFPGAAmplitudeCheckTh(nValue);
	}

	bool DepthCamera::setFPGAScatteringCheckTh(const uint8_t &nValue)
	{
		Lock<Mutex> _(_accessMutex);
		return _setFPGAScatteringCheckTh(nValue);
	}

	bool DepthCamera::getFPGAScatteringCheckTh(uint8_t &nValue) const
	{
		Lock<Mutex> _(_accessMutex);
		return _getFPGAScatteringCheckTh(nValue);
	}

	bool DepthCamera::setFPGAHDRFilter(const bool &bEnable)
	{
		Lock<Mutex> _(_accessMutex);
		return _setFPGAHDRFilter(bEnable);
	}

	bool DepthCamera::getFPGAHDRFilter(bool &bEnable) const
	{
		Lock<Mutex> _(_accessMutex);
		return _getFPGAHDRFilter(bEnable);
	}

	bool DepthCamera::setFPGAWigglingEnable(const bool &bEnable)
	{
		Lock<Mutex> _(_accessMutex);
		return _setFPGAWigglingEnable(bEnable);
	}

	bool DepthCamera::getFPGAWigglingEnable(bool &bEnable) const
	{
		Lock<Mutex> _(_accessMutex);
		return _getFPGAWigglingEnable(bEnable);
	}

	bool DepthCamera::setFPGAFPPNEnable(const bool &bEnable)
	{
		Lock<Mutex> _(_accessMutex);
		return _setFPGAFPPNEnable(bEnable);
	}

	bool DepthCamera::getFPGAFPPNEnable(bool &bEnable) const
	{
		Lock<Mutex> _(_accessMutex);
		return _getFPGAFPPNEnable(bEnable);
	}

	bool DepthCamera::saveFPGAParameter(const uint8_t *pData)
	{
		Lock<Mutex> _(_accessMutex);
		return _saveFPGAParameter(pData);
	}

	bool DepthCamera::loadFPGAParameter(uint8_t *pData) const
	{
		Lock<Mutex> _(_accessMutex);
		return _loadFPGAParameter(pData);
	}

	bool DepthCamera::setFPGAWigglingData(const uint8_t *pData, const uint8_t &nCh)
	{
		Lock<Mutex> _(_accessMutex);
		return _setFPGAWigglingData(pData, nCh);
	}

	bool DepthCamera::getFPGAWigglingData(uint8_t *pData, uint8_t &nCh) const
	{
		Lock<Mutex> _(_accessMutex);
		return _getFPGAWigglingData(pData, nCh);
	}

	bool DepthCamera::setFPGAFppnData(const uint8_t *pData)
	{
		Lock<Mutex> _(_accessMutex);
		return _setFPGAFppnData(pData);
	}

	bool DepthCamera::getFPGAFppnData(uint8_t *pData) const
	{
		Lock<Mutex> _(_accessMutex);
		return _getFPGAFppnData(pData);
	}

	bool DepthCamera::setFPGAOrthogonalData(const uint8_t *pData)
	{
		Lock<Mutex> _(_accessMutex);
		return _setFPGAOrthogonalData(pData);
	}

	bool DepthCamera::getFPGAOrthogonalData(uint8_t *pData) const
	{
		Lock<Mutex> _(_accessMutex);
		return _getFPGAOrthogonalData(pData);
	}

//////////////////////////////////////// meere add function ///////////////////////////////////////////////

	bool DepthCamera::getFrameRate(FrameRate &r) const
	{
		Lock<Mutex> _(_accessMutex);
		return _getFrameRate(r);
	}

	bool DepthCamera::setFrameRate(const FrameRate &r)
	{
		Lock<Mutex> _(_accessMutex);
		return _setFrameRate(r);
	}

	bool DepthCamera::getFrameSize(FrameSize &s) const
	{
		Lock<Mutex> _(_accessMutex);
		return _getFrameSize(s);
	}

	bool DepthCamera::setFrameSize(const FrameSize &s)
	{
		Lock<Mutex> _(_accessMutex);
		return _setFrameSize(s);
	}

	bool DepthCamera::getMaximumFrameSize(FrameSize &s) const
	{
		Lock<Mutex> _(_accessMutex);
		return _getMaximumFrameSize(s);
	}

	bool DepthCamera::getMaximumFrameRate(FrameRate &frameRate, const FrameSize &forFrameSize) const
	{
		Lock<Mutex> _(_accessMutex);
		return _getMaximumFrameRate(frameRate, forFrameSize);
	}


	bool DepthCamera::getSupportedVideoModes(Vector<SupportedVideoMode> &supportedVideoModes) const
	{
		Lock<Mutex> _(_accessMutex);
		return _getSupportedVideoModes(supportedVideoModes);
	}

	bool DepthCamera::getMaximumVideoMode(VideoMode &videoMode) const
	{
		Lock<Mutex> _(_accessMutex);
		return _getMaximumVideoMode(videoMode);
	}

	bool DepthCamera::getBytesPerPixel(uint &bpp) const
	{
		Lock<Mutex> _(_accessMutex);
		return _getBytesPerPixel(bpp);
	}

	bool DepthCamera::setBytesPerPixel(const uint &bpp)
	{
		Lock<Mutex> _(_accessMutex);
		return _setBytesPerPixel(bpp);
	}


	bool DepthCamera::allowedROI(String &message)
	{
		Lock<Mutex> _(_accessMutex);
		return _allowedROI(message);
	}

	bool DepthCamera::getROI(RegionOfInterest &roi)
	{
		Lock<Mutex> _(_accessMutex);
		return _getROI(roi);
	}

	bool DepthCamera::setROI(const RegionOfInterest &roi)
	{
		Lock<Mutex> _(_accessMutex);
		return _setROI(roi);
	}

	template <typename T>
	bool DepthCamera::_get(const String &name, T &value, bool refresh) const
	{
		auto p = _parameters.find(name);

		if (p != _parameters.end())
		{
			ParameterTemplate<T> *param = dynamic_cast<ParameterTemplate<T> *>(p->second.get());

			if (param == 0)
			{
				logger(LOG_ERROR) << "DepthCamera: Invalid value type '" << typeid(value).name() << "' used to get parameter " << _id << "." << name << std::endl;
				return false;
			}

			if (!param->get(value, refresh))
			{
				logger(LOG_ERROR) << "DepthCamera:Could not get value for parameter " << _id << "." << name << std::endl;
				return false;
			}

			return true;
		}
		else
		{
			logger(LOG_ERROR) << "DepthCamera: Unknown parameter " << _id << "." << name << std::endl;
			return false;
		}
	}

	template <typename T>
	bool DepthCamera::_set(const String &name, const T &value)
	{
		auto p = _parameters.find(name);

		if (p != _parameters.end())
		{
			logger(LOG_DEBUG) << "DepthCamera: Setting parameter '" << name << "' = " << value << std::endl;
			ParameterTemplate<T> *param = dynamic_cast<ParameterTemplate<T> *>(p->second.get());

			if (param == 0)
			{
				logger(LOG_ERROR) << "DepthCamera: Invalid value type '" << typeid(value).name() << "' used to set parameter " << this->name() << "(" << _device->id() << ")." << name << std::endl;
				return false;
			}

			if (!param->set(value))
			{
				logger(LOG_ERROR) << "DepthCamera: Could not set value " << value << " for parameter " << this->name() << "(" << _device->id() << ")." << name << std::endl;
				return false;
			}

			return true;
		}
		else
		{
			logger(LOG_ERROR) << "DepthCamera: Unknown parameter " << _id << "." << name << std::endl;
			return false;
		}
	}

	const ParameterPtr DepthCamera::getParam(const String &name) const
	{
		auto p = _parameters.find(name);

		if (p != _parameters.end())
			return p->second;
		else
		{
			logger(LOG_ERROR) << "DepthCamera: Unknown parameter " << _id << "." << name << std::endl;
			return 0;
		}
	}

	template <typename T>
	bool DepthCamera::getStreamParam(const String &name, T &value) const
	{
		if (!_frameGenerators[0]->get(name, value) && !_frameGenerators[1]->get(name, value) && !_frameGenerators[2]->get(name, value))
			return false;

		return true;
	}

	typedef Ptr<DepthCamera> DepthCameraPtr;

}

#endif // DEPTHCAMERA_H
