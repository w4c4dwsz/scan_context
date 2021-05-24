#include <iostream>
#include <pcl/pcl_base.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/io.h>
#include <vector>
#include <string>
#include <dirent.h>
#include <unistd.h>
#include "Scancontext.h"
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

SCManager scManager;
ros::Publisher currFramePub;
ros::Publisher globalMapPub;

ros::Subscriber sub;

static int
dirSelect (const struct dirent *unused)
{
    if(unused->d_name[0] == '.'){
        return 0;
    }
    
  return 1;
}


void
getFilenames(std::string dirName, std::vector<std::string>& filenameVector){
    struct dirent ** eps;
    int fileNum = scandir(dirName.c_str(), &eps, dirSelect, alphasort);
    if(fileNum == -1){
        puts("In function getFIlenames, scandir fault\n");
        perror("The following error occurred");
    }
    for(int i = 0; i < fileNum; i++){
        filenameVector.push_back(dirName + "/" + std::string(eps[i]->d_name));
    }
    free(eps);
}

void
PointXYZ2PointXYZI(pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud, pcl::PointCloud<pcl::PointXYZI>::Ptr outCloud) {
    for(auto p : inCloud->points){
        pcl::PointXYZI point;
        point.x = p.x;
        point.y = p.y;
        point.z = p.z;
        point.intensity = 0;
        outCloud->push_back(point);
    }

}

void
PointXYZI2PointXYZ(pcl::PointCloud<pcl::PointXYZI>::Ptr inCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr outCloud) {
    for(auto p : inCloud->points){
            pcl::PointXYZ point;
            point.x = p.x;
            point.y = p.y;
            point.z = p.z;
            outCloud->push_back(point);
        }
}

void
pubMap(pcl::PointCloud<pcl::PointXYZ>::Ptr pubCloud) {
    sensor_msgs::PointCloud2 mapMsgs;
    pcl::toROSMsg(*pubCloud, mapMsgs);
    mapMsgs.header.frame_id = "map";
    globalMapPub.publish(mapMsgs);
}


void
cloudHandle(const sensor_msgs::PointCloud2 msgs){
    pcl::PointCloud<pcl::PointXYZ>::Ptr msgsCloudXYZ(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr msgsCloudXYZI(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(msgs, *msgsCloudXYZI);
    scManager.makeAndSaveScancontextAndKeys(*msgsCloudXYZI);
    std::pair<int, double> result = scManager.detectLoopClosureID();
    printf("[%d, %f]\n", result.first, result.second);
    // std::cout << scManager.polarcontext_vkeys_.back() << std::endl;
}


int main(int argc, char** argv) {
    // init ros pub&sub
    ros::init(argc,argv,"scan_context");
    ros::NodeHandle nh;
    sub = nh.subscribe<sensor_msgs::PointCloud2>("/lslidar_point_cloud", 10, cloudHandle);
    currFramePub = nh.advertise<sensor_msgs::PointCloud2>("scan_context/curr_frame", 10);
    globalMapPub = nh.advertise<sensor_msgs::PointCloud2>("scan_context/global_map", 10);

    std::string savePCDDirectory = "/home/alinx/Downloads/SCLOAM/20200129";
    // set saveSCDDirectory and saveNodePCDDirectory
    std::string saveSCDDirectory = savePCDDirectory + "/SCDs";
    std::string saveNodePCDDirectory = savePCDDirectory + "/Scans";
    std::cout<< "savePCDDirectory=" << savePCDDirectory << endl;
    std::cout<< "saveSCDDirectory=" << saveSCDDirectory << endl;
    std::cout<< "saveNodePCDDirectory=" << saveNodePCDDirectory << endl;

    // get node pcd file name
    std::vector<std::string> fileNameList;
    getFilenames(saveNodePCDDirectory, fileNameList);

    // get scd file name
    std::vector<std::string> scdFileNameList;
    getFilenames(saveSCDDirectory, scdFileNameList);

    // get pcd file and load it to vector
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> keyFrameCloudVector;
    for(auto filename : fileNameList){
        pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud (new pcl::PointCloud<pcl::PointXYZI>);
        pcl::io::loadPCDFile(filename, *temp_cloud);
        keyFrameCloudVector.push_back(temp_cloud);
    }
    
    // TODO: get scd file and load it to vector

    
    // init scMagner
    pcl::PointCloud<pcl::PointXYZ>::Ptr globalMap;
    for(auto cloud : keyFrameCloudVector){
        pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud;
        PointXYZI2PointXYZ(cloud, tempCloud);
        *globalMap += *tempCloud;
        scManager.makeAndSaveScancontextAndKeys(*cloud);
        std::pair<int,double> result = scManager.detectLoopClosureID();
        if(result.first != -1){
            scManager.polarcontexts_.pop_back();
            scManager.polarcontext_invkeys_.pop_back();
            scManager.polarcontext_vkeys_.pop_back();
            scManager.polarcontext_invkeys_mat_.pop_back();
        }
        
    }

    // pub global map
    pubMap(globalMap);

    ros::spin();
    return 0;
}
