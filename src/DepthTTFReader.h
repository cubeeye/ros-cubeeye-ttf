#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "include/TTF_API.h"

#define IMAGE_NUM 2
#define DATA_AMPLITUDE 0
#define DATA_DEPTH 1
#define MDC200_WIDTH 320
#define MDC200_HEIGHT 240
#define MDC200_MIN_DEPTH 0
#define MDC200_MAX_DEPTH 3750

/* PUB_DEPTH and PUB_AMPLITUDE need to move on common */
#define PUB_DEPTH "/depth/depth_raw"
#define PUB_AMPLITUDE "/depth/amplitude_raw"
#define PUB_PCL "/depth/points"

class DepthTTFReader {
    private:
        int m_nDeviceCount;//device count
        int m_nDevHnd; //device handle

        TTF_API::_ttfDeviceInfo m_stDevInfo[MAX_DEVICE]; //device information structure
        DepthFrame          *m_pDepthFrame;
        XYZIPointCloudFrame *m_pPCLFrame;

        ros::Publisher &m_PubDepthRaw;
        ros::Publisher &m_PubAmplitudeRaw;
        ros::Publisher &m_PubPCLRaw;

        sensor_msgs::ImagePtr m_msgImgPtrDepth;
        sensor_msgs::ImagePtr m_msgImgPtrAmplitude;

        float* m_pDepthData;
        float* m_pAmplitudeData;
        sensor_msgs::PointCloud2 m_msgPCL2;
        sensor_msgs::PointCloud2Ptr m_msgPCL2ptr;
        
        int bufferInit();        
        int setIntegrationDutyTime();
        
        int getLensParameter();
        int getIntegrationDutyTime();

        int getAmplitudeThreshold();
        int getScatteringCheckThreshold();
        int getPhaseOffset();
        
        int readDepthFrame(const DepthFrame* pDepthFrame);
        int readPointCloudFrame(const XYZIPointCloudFrame* pXYZIPointCloudFrame);

    public:
        bool    debug;
        bool    setMDCParam;
        bool    setMDCLensParam;
        int     width;
        int     height;


        int     nAmplitudeThres;        // 0 ~ 255
        int     nPhaseOffset;           // -127 ~ 127
        int     nScatterThres;          // 0 ~ 255

        bool    nMedianFilter;
        bool    nSmoothFilter;
        bool    nIIRFilter;
        bool    nFlyPixFilter;

        int     nKernelSize;
        float   nDeadband;
        float   nDeadbandStep;
        float   nSigma;
        float   nGain;
        float   nThr;

        bool mLoopOk;

        DepthTTFReader(ros::Publisher &pub_depth_raw, ros::Publisher &pub_amplitude_raw, ros::Publisher &pub_pcl_raw);

        bool connect();
        bool close();
        
        void dataDepth();
        void dataPCL();

        void deviceClose();
        void deviceStop();

        int setAmplitudeThreshold();
        int setScatteringCheckThreshold();
        int setPhaseOffset();

        int setMedianFilter();
        int setSmoothFilter();
        int setIIRFilter();
        int setFlyPixFilter();
};
