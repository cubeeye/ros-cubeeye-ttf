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
        mr->m_nAmplitudeThres = config.Amplitude;
        mr->SetAmplitudeThreshold();
        break;
    case 2://Scattering
        mr->m_nScatterThres = config.Scattering;
        mr->SetScatteringCheckThreshold();
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



    nh.getParam("/depth_camera/amplitudeThres", mr->m_nAmplitudeThres);
    nh.getParam("/depth_camera/scatterThres", mr->m_nScatterThres);


    //loop speed 10Hz : It should be modified
    //to be in inverse proportion to the fps delay(nDelay) parameter. 
    ros::Rate loop_rate(10);

    if(!mr->connect()) {
        ROS_ERROR( "Depth Camera Connection Failed!...");
        std::exit(1);
    }    

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
