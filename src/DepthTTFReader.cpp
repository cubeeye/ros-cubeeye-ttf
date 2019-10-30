#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include "TTF_API.h"
#include "DepthTTFReader.h"

DepthTTFReader::DepthTTFReader(ros::Publisher &pub_depth_raw, ros::Publisher &pub_amplitude_raw, ros::Publisher &pub_pcl_raw)
    : m_PubDepthRaw(pub_depth_raw)
    , m_PubAmplitudeRaw(pub_amplitude_raw)
    , m_PubPCLRaw(pub_pcl_raw)
{
    /* allocates memory and initializes device handle */
    width = MDC200_WIDTH;
    height = MDC200_HEIGHT;
    m_nDeviceCount = 0;
    mLoopOk = true;

    TTF_API::ttfGetDeviceList(m_stDevInfo, m_nDeviceCount);
    if (m_nDeviceCount < 1) {
        ROS_ERROR("No Depth Camera TTF Found!!, Please Check Connection and Restart. Connection Error");
    }
    m_nDevHnd = m_stDevInfo[0].hnd;

    //generate Depth
    m_msgImgPtrDepth = sensor_msgs::ImagePtr(new sensor_msgs::Image);
    m_msgImgPtrDepth->header.frame_id = "distance";
    m_msgImgPtrDepth->width = width;
    m_msgImgPtrDepth->height = height;
    m_msgImgPtrDepth->is_bigendian = false;
    m_msgImgPtrDepth->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    m_msgImgPtrDepth->step = (uint32_t)(sizeof(float) * width);
    m_msgImgPtrDepth->data.resize(sizeof(float) * width * height);
    m_pDepthData = (float *)&m_msgImgPtrDepth->data[0];

    //generate Amplitude
    m_msgImgPtrAmplitude = sensor_msgs::ImagePtr(new sensor_msgs::Image);
    m_msgImgPtrAmplitude->header.frame_id = "amplitude";
    m_msgImgPtrAmplitude->width = width;
    m_msgImgPtrAmplitude->height = height;
    m_msgImgPtrAmplitude->is_bigendian = false;
    m_msgImgPtrAmplitude->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    m_msgImgPtrAmplitude->step = (uint32_t)(sizeof(float) * width);
    m_msgImgPtrAmplitude->data.resize(sizeof(float) * width * height);
    m_pAmplitudeData = (float *)&m_msgImgPtrAmplitude->data[0];

    //generate Pointcloud
    m_msgPCL2ptr = sensor_msgs::PointCloud2Ptr(new sensor_msgs::PointCloud2);
    m_msgPCL2ptr->header.frame_id = "pcl";
    m_msgPCL2ptr->header.stamp = m_msgImgPtrAmplitude->header.stamp;
    m_msgPCL2ptr->width = width;
    m_msgPCL2ptr->height = height;
    m_msgPCL2ptr->is_bigendian = false;
    m_msgPCL2ptr->is_dense = false;

    m_msgPCL2ptr->point_step = (uint32_t)(3 * sizeof(float));
    m_msgPCL2ptr->row_step = (uint32_t)(m_msgPCL2ptr->point_step * width);
    m_msgPCL2ptr->fields.resize(3);
    m_msgPCL2ptr->fields[0].name = "z";
    m_msgPCL2ptr->fields[0].offset = 0;
    m_msgPCL2ptr->fields[0].datatype = sensor_msgs::PointField::FLOAT32;
    m_msgPCL2ptr->fields[0].count = 1;

    m_msgPCL2ptr->fields[1].name = "y";
    m_msgPCL2ptr->fields[1].offset = m_msgPCL2ptr->fields[0].offset + (uint32_t)sizeof(float);
    m_msgPCL2ptr->fields[1].datatype = sensor_msgs::PointField::FLOAT32;
    m_msgPCL2ptr->fields[1].count = 1;

    m_msgPCL2ptr->fields[2].name = "x";
    m_msgPCL2ptr->fields[2].offset = m_msgPCL2ptr->fields[1].offset + (uint32_t)sizeof(float);
    m_msgPCL2ptr->fields[2].datatype = sensor_msgs::PointField::FLOAT32;
    m_msgPCL2ptr->fields[2].count = 1;
    m_msgPCL2ptr->data.resize(m_msgPCL2ptr->point_step * m_msgPCL2ptr->width * m_msgPCL2ptr->height);

}

int DepthTTFReader::bufferInit()
{
    m_pDepthFrame   = (DepthFrame*)malloc(sizeof(DepthFrame));;
    m_pPCLFrame     = (XYZIPointCloudFrame*)malloc(sizeof(XYZIPointCloudFrame));

    if (m_pDepthFrame == NULL || m_pPCLFrame == NULL)
    {
        ROS_ERROR("Failed to create nescecary buffers to display the image");
        return TTF_API::ERROR_FAIL;
    }
    else
        return TTF_API::ERROR_NO;
}

//0~255
int DepthTTFReader::setAmplitudeThreshold() {
    
    ROS_INFO("Set Amplitude Threshold %u ", nAmplitudeThres);
    TTF_API::ttfSetAmplitudeCheckTh(m_nDevHnd, nAmplitudeThres);

    return TTF_API::ERROR_NO;
}

//0~255
int DepthTTFReader::setScatteringCheckThreshold() {

    ROS_INFO("Set Scattering Check Threshold : %u ", nScatterThres);
    TTF_API::ttfSetScatteringCheckTh(m_nDevHnd, nScatterThres);

    return TTF_API::ERROR_NO;
}

//-127 ~ 127
int DepthTTFReader::setPhaseOffset() {

    ROS_INFO("Set Phase Offset : %u ", nPhaseOffset);
    TTF_API::ttfSetPhaseOffset(m_nDevHnd, nPhaseOffset);

    return TTF_API::ERROR_NO;
}

int DepthTTFReader::setMedianFilter() {

    TTF_API::ttfSetMedianFilter(m_nDevHnd, nMedianFilter, nKernelSize, nDeadband, nDeadbandStep);

    ROS_INFO("Set Median Filter : %s", nMedianFilter ? "true" : "false");
    ROS_INFO(" Kernel Size : %d", nKernelSize);
    ROS_INFO(" Deadband : %f", nDeadband);
    ROS_INFO(" Deadband Step : %f", nDeadbandStep);

    return TTF_API::ERROR_NO;
}

int DepthTTFReader::setSmoothFilter() {

    TTF_API::ttfSetSmoothFilter(m_nDevHnd, nSmoothFilter, nSigma);

    ROS_INFO("Set Smooth Filter : %s", nSmoothFilter ? "true" : "false");
    ROS_INFO(" Sigma : %f", nSigma);

    return TTF_API::ERROR_NO;
}

int DepthTTFReader::setIIRFilter() {

    TTF_API::ttfSetIIRFilter(m_nDevHnd, nIIRFilter, nGain);

    ROS_INFO("Set IIR Filter : %s", nIIRFilter ? "true" : "false");
    ROS_INFO(" Gain : %f", nGain);

    return TTF_API::ERROR_NO;
}

int DepthTTFReader::setFlyPixFilter() {

    TTF_API::ttfSetFlyPixFilter(m_nDevHnd, nFlyPixFilter, nThr);

    ROS_INFO("Set FlyPix Filter : %s", nFlyPixFilter ? "true" : "false");
    ROS_INFO(" Thr : %f", nThr);

    return TTF_API::ERROR_NO;
}


//0~31%
int DepthTTFReader::getIntegrationDutyTime() {

    unsigned int temp;

    TTF_API::ttfGetIntgDutyTime(m_nDevHnd, temp);
    ROS_INFO("Get Integration Duty Time %u ", temp);

    return TTF_API::ERROR_NO;
}

//0~255
int DepthTTFReader::getAmplitudeThreshold() {
    uint8_t temp;
    
    TTF_API::ttfGetAmplitudeCheckTh(m_nDevHnd, temp);
    ROS_INFO("Get Amplitude Threshold %u ", temp);


    return TTF_API::ERROR_NO;
}

//-127 ~ 127
int DepthTTFReader::getPhaseOffset() {
    int8_t temp;

    TTF_API::ttfGetPhaseOffset(m_nDevHnd, temp);
    ROS_INFO("Get Phase Offset : %u ", temp);

    return TTF_API::ERROR_NO;
}


//0~255
int DepthTTFReader::getScatteringCheckThreshold() {
    uint8_t temp;

    TTF_API::ttfGetScatteringCheckTh(m_nDevHnd, temp);
    ROS_INFO("Get Scattering Check Threshold : %u ", temp);

    return TTF_API::ERROR_NO;
}


int DepthTTFReader::getLensParameter() {
    TTF_API::ttfIntrinsicParam tempIntrinsicParam;
    TTF_API::ttfDistortionParam tempDistortionParam;

    TTF_API::ttfGetLensParameter(m_nDevHnd, &tempIntrinsicParam, &tempDistortionParam);

    ROS_INFO("Get IntrinsicParameter fFx : %f", tempIntrinsicParam.fFx);
    ROS_INFO("Get IntrinsicParameter fFy : %f", tempIntrinsicParam.fFy);
    ROS_INFO("Get IntrinsicParameter fCx : %f", tempIntrinsicParam.fCx);
    ROS_INFO("Get IntrinsicParameter fCx : %f", tempIntrinsicParam.fCy);

    ROS_INFO("Get DistortionParamter fK1 : %f", tempDistortionParam.fK1);
    ROS_INFO("Get DistortionParamter fK2 : %f", tempDistortionParam.fK2);
    ROS_INFO("Get DistortionParamter fK3 : %f", tempDistortionParam.fK3);
    ROS_INFO("Get DistortionParamter fP1 : %f", tempDistortionParam.fP1);
    ROS_INFO("Get DistortionParamter fP2 : %f", tempDistortionParam.fP2);

    return TTF_API::ERROR_NO;
}

bool DepthTTFReader::connect() {
    
    /* connects to TTF sensor; returns false if failed */

    if (TTF_API::ttfDeviceOpen(m_nDevHnd) != TTF_API::ERROR_NO) {
        ROS_ERROR("Device Open Failed");
        return false;
    } else {
        //Callback Register
        ROS_INFO("Registering Callback for Read TTF Depth/Point Cloud Frame");
        TTF_API::ttfRegister_Callback_DepthFrame(std::bind(&DepthTTFReader::readDepthFrame, this, std::placeholders::_1));
        TTF_API::ttfRegister_Callback_PointCloudFrame(std::bind(&DepthTTFReader::readPointCloudFrame, this, std::placeholders::_1));

        ROS_INFO("Name : %s", m_stDevInfo[0].szName);
        ROS_INFO("VendorID : %x, ProductID : %x", m_stDevInfo[0].nVendorId, m_stDevInfo[0].nProductId);
        ROS_INFO("DeviceType : %d", m_stDevInfo[0].nDeviceType);
        ROS_INFO("SerialNum : %s", m_stDevInfo[0].szSerialNum);
        ROS_INFO("Camera Open Success");

        TTF_API::ttfSetSmoothFilter(m_stDevInfo[0].hnd, true);
        TTF_API::ttfSetFlyPixFilter(m_stDevInfo[0].hnd, true);
    }

    bufferInit();
   // usleep(100); //

    if(setMDCParam) {

        if(debug) ROS_INFO("ERROR NO is %d", TTF_API::ERROR_NO);

        setAmplitudeThreshold();
        setScatteringCheckThreshold();
        setPhaseOffset();
        setMedianFilter();
        setSmoothFilter();
        setIIRFilter();
        setFlyPixFilter();
    }

    if(debug) {
        getAmplitudeThreshold();
        getScatteringCheckThreshold();
        getPhaseOffset();
        getLensParameter();
    }

//    dataDepth();
    dataPCL();

    return true;
}

void DepthTTFReader::dataDepth() {
    TTF_API::ttfClearCallback(m_stDevInfo[0].hnd, TTF_API::FrameType::FRAME_XYZI_POINT_CLOUD_FRAME);
    TTF_API::ttfDeviceStart(m_stDevInfo[0].hnd, TTF_API::FrameType::FRAME_DEPTH_FRAME);
}

void DepthTTFReader::dataPCL() {
    TTF_API::ttfClearCallback(m_stDevInfo[0].hnd, TTF_API::FrameType::FRAME_DEPTH_FRAME);
    TTF_API::ttfDeviceStart(m_stDevInfo[0].hnd, TTF_API::FrameType::FRAME_XYZI_POINT_CLOUD_FRAME);
}


bool DepthTTFReader::close()
{
    /* closes connection to TTF sensor gracefully */
    ROS_INFO("Stop Grab TTF");

    TTF_API::ttfDeviceStop(m_stDevInfo[0].hnd);
    ROS_INFO("Device Close TTF");
    TTF_API::ttfDeviceClose(m_stDevInfo[0].hnd);    

    return true;
}

int DepthTTFReader::readDepthFrame(const DepthFrame* pDepthFrame) {
    const DepthFrame *d = pDepthFrame;
    const float* dataDepth;
    const float* dataAmplitude;

    dataDepth = d->depth.data();
    dataAmplitude = d->amplitude.data();

#if 0
    //Test Code
    std::cout << "Depth Size : " << d->depth.size() << std::endl;
    std::cout << "Amplitude Size : " << d->depth.size() << std::endl;
    std::cout << "Depth:"<< d->id << "@" << d->timestamp << " data:" <<  dataDepth[320*120+160] << std::endl;
    std::cout << "Amplitude:"<< d->id << "@" << d->timestamp << " data:" <<  dataAmplitude[320*120+160] << std::endl;
#endif

    m_msgImgPtrDepth->header.stamp = ros::Time::now();
    m_msgImgPtrAmplitude->header.stamp = m_msgImgPtrDepth->header.stamp;//sync imgDepthRaw time

    for (unsigned index = 0; index < m_msgImgPtrDepth->height * m_msgImgPtrDepth->width; ++index)
    {
        m_pDepthData[index] = dataDepth[index];
        m_pAmplitudeData[index] = dataAmplitude[index];
    }

    m_PubDepthRaw.publish(m_msgImgPtrDepth);
    m_PubPCLRaw.publish(m_msgPCL2ptr);

    return TTF_API::ERROR_NO;
}

int DepthTTFReader::readPointCloudFrame(const XYZIPointCloudFrame* pXYZIPointCloudFrame) {

    const XYZIPointCloudFrame *d = pXYZIPointCloudFrame;

//    std_msgs::Header header;
    m_msgImgPtrAmplitude->header.stamp = ros::Time::now();
    m_msgPCL2ptr->header.stamp = m_msgImgPtrAmplitude->header.stamp;

    for (unsigned index = 0; index < m_msgPCL2ptr->height * m_msgPCL2ptr->width; ++index)
    {
        float *pcl_a = (float *)&m_msgPCL2ptr->data[index * m_msgPCL2ptr->point_step];
        float *pcl_b = pcl_a + 1;
        float *pcl_c = pcl_b + 1;

        *pcl_a = d->points[index].y*-1;
        *pcl_b = d->points[index].x*-1;
        *pcl_c = d->points[index].z;

        m_pDepthData[index] = d->points[index].z;
        m_pAmplitudeData[index] = d->points[index].i;
    }

    m_PubAmplitudeRaw.publish(m_msgImgPtrAmplitude);
    m_PubDepthRaw.publish(m_msgImgPtrDepth);
    m_PubPCLRaw.publish(m_msgPCL2ptr);

    return TTF_API::ERROR_NO;
}

