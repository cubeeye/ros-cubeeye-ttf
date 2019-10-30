#include <ros/ros.h>
#include <signal.h>
#include <std_msgs/UInt16MultiArray.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include "include/TTF_API.h"
#include "DepthTTFReader.h"
#include <sstream>

#include <dynamic_reconfigure/server.h>
#include <depth_ttf/depth_ttfConfig.h>

DepthTTFReader* mr;

void gracefulShutdown(int sigNum) {
    mr->mLoopOk = false;
}

void callbackConfig(depth_ttf_node::depth_ttfConfig &config, uint32_t level)
{
    if(mr==NULL)
        return;

    switch (level) {
    case 1://Amplitude
        mr->nAmplitudeThres = config.Amplitude;
        mr->setAmplitudeThreshold();
        break;
    case 2://Scattering
        mr->nScatterThres = config.Scattering;
        mr->setScatteringCheckThreshold();
        break;
    case 3://Offset
        mr->nPhaseOffset = config.Offset;
        mr->setPhaseOffset();
        break;
    case 4://SmoothFilter
        mr->nScatterThres = config.SmoothFilter;
        mr->setSmoothFilter();
        break;
    case 5://FlyPixelFilter
        mr->nFlyPixFilter = config.FlyPixelFilter;
        mr->setFlyPixFilter();
        break;
    case 6://MedianFilter
        mr->nMedianFilter = config.MedianFilter;
        mr->setMedianFilter();
        break;
    case 7://IIRFilter
        mr->nIIRFilter = config.IIRFilter;
        mr->setIIRFilter();
        break;
    default:
        break;
    }



}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "depth_ttf_node");
    ros::NodeHandle nh;

    //get parameter for TTF
    ros::Publisher pub_depth_raw = nh.advertise<sensor_msgs::Image>(PUB_DEPTH, 1);
    ros::Publisher pub_amplitude_raw = nh.advertise<sensor_msgs::Image>(PUB_AMPLITUDE, 1);
    ros::Publisher pub_pcl_raw = nh.advertise<sensor_msgs::PointCloud2>(PUB_PCL, 1);

    dynamic_reconfigure::Server<depth_ttf_node::depth_ttfConfig> server;
    dynamic_reconfigure::Server<depth_ttf_node::depth_ttfConfig>::CallbackType f;

    f = boost::bind(&callbackConfig, _1, _2);
    server.setCallback(f);
	
    mr = new DepthTTFReader(pub_depth_raw,pub_amplitude_raw, pub_pcl_raw);

    nh.getParam("/depth_camera/debug", mr->debug);
    nh.getParam("/depth_camera/setMDCParam", mr->setMDCParam);
    nh.getParam("/depth_camera/setLens", mr->setMDCLensParam);

    nh.getParam("/depth_camera/amplitudeThres", mr->nAmplitudeThres);
    nh.getParam("/depth_camera/scatterThres", mr->nScatterThres);
    nh.getParam("/depth_camera/phaseOffset", mr->nPhaseOffset);

    nh.getParam("/depth_camera/medianFilter", mr->nMedianFilter);
    nh.getParam("/depth_camera/smoothFilter", mr->nSmoothFilter);
    nh.getParam("/depth_camera/IIRFilter", mr->nIIRFilter);
    nh.getParam("/depth_camera/FlyPixFilter", mr->nFlyPixFilter);

    nh.getParam("/depth_camera/kernelSize", mr->nKernelSize);
    nh.getParam("/depth_camera/deadband", mr->nDeadband);
    nh.getParam("/depth_camera/deadbandStep", mr->nDeadbandStep);
    nh.getParam("/depth_camera/sigma", mr->nSigma);
    nh.getParam("/depth_camera/gain", mr->nGain);
    nh.getParam("/depth_camera/thr", mr->nThr);

    ROS_INFO("DEBUG : %d\n", mr->debug);
    ROS_INFO("setMDCParam : %d\n", mr->setMDCParam);

    if(mr->debug) 
    {        
        ROS_INFO("AmplitudcallbackConfige Threshold : %u", mr->nAmplitudeThres);
        ROS_INFO("Scattering Check Threshold : %u", mr->nScatterThres);
        ROS_INFO("Phase Offset : %u", mr->nPhaseOffset);

        ROS_INFO("Median Filter : %s", mr->nMedianFilter ? "true" : "false");
        ROS_INFO("kernel Size : %d", mr->nKernelSize);
        ROS_INFO("Deadband : %f", mr->nDeadband);
        ROS_INFO("Deadband Step : %f", mr->nDeadbandStep);
        ROS_INFO("Smooth Filter : %s", mr->nSmoothFilter ? "true" : "false");
        ROS_INFO("Sigma : %f", mr->nSigma);
        ROS_INFO("IIR Filter : %s", mr->nIIRFilter ? "true" : "false");
        ROS_INFO("Gain : %f", mr->nGain);
        ROS_INFO("FlyPix Filter : %s", mr->nFlyPixFilter ? "true" : "false");
        ROS_INFO("Thr : %f", mr->nThr);
    }


    //loop speed 10Hz : It should be modified
    //to be in inverse proportion to the fps delay(nDelay) parameter. 
    ros::Rate loop_rate(10);

    if(!mr->connect()) {
        ROS_ERROR( "Depth Camera Connection Failed!...");
        std::exit(1);
    }
    
    //Ctrl+C handler for standalone TEST.
    //It must be removed at Release version.
    signal(SIGINT, gracefulShutdown);

    ROS_INFO("Depth Camera Start\n");

    while(mr->mLoopOk)
    {
            ros::spinOnce();
            loop_rate.sleep();
    }
    mr->close();
    ROS_INFO("Depth Camera Stop\n");
    return 0;
}
