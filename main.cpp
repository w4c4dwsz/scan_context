#include <iostream>

#include <vector>
#include <string>
#include <dirent.h>
#include <unistd.h>
#include "Scancontext.h"
#include <ros/ros.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <pcl/pcl_base.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>

/*
    * A point cloud type that has 6D pose info ([x,y,z,roll,pitch,yaw] intensity is time stamp)
    */
struct PointXYZIRPYT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;                  // preferred way of adding a XYZ+padding
    float roll;
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRPYT,
                                   (float, x, x) (float, y, y)
                                   (float, z, z) (float, intensity, intensity)
                                   (float, roll, roll) (float, pitch, pitch) (float, yaw, yaw)
                                   (double, time, time))

typedef PointXYZIRPYT  PointTypePose;

SCManager scManager;
ros::Publisher currFramePub;
ros::Publisher globalMapPub;
ros::Publisher currPosPUb;
ros::Subscriber sub;

std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> keyFrameCloudVector;
pcl::PointCloud<PointTypePose>::Ptr keyFramePos(new pcl::PointCloud<PointTypePose>);

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

pcl::PointCloud<pcl::PointXYZ>::Ptr
transformPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud, PointTypePose &inTransform){
    pcl::PointCloud<pcl::PointXYZ>::Ptr outCloud(new pcl::PointCloud<pcl::PointXYZ>);
    Eigen::Affine3f currTrans = pcl::getTransformation(inTransform.x, inTransform.y,inTransform.z,
                                                        inTransform.roll, inTransform.pitch, inTransform.yaw);
    size_t cloudSize = inCloud->size();
    outCloud->resize(cloudSize);
    pcl::PointXYZ *pointFrom;
    #pragma omp parallel for num_threads(4)
    for (int i = 0; i < cloudSize; ++i)
    {
        pointFrom = &inCloud->points[i];
        outCloud->points[i].x = currTrans(0,0) * pointFrom->x + currTrans(0,1) * pointFrom->y + currTrans(0,2) * pointFrom->z + currTrans(0,3);
        outCloud->points[i].y = currTrans(1,0) * pointFrom->x + currTrans(1,1) * pointFrom->y + currTrans(1,2) * pointFrom->z + currTrans(1,3);
        outCloud->points[i].z = currTrans(2,0) * pointFrom->x + currTrans(2,1) * pointFrom->y + currTrans(2,2) * pointFrom->z + currTrans(2,3);
    }
    return outCloud;
}

void
pubMap(pcl::PointCloud<pcl::PointXYZ>::Ptr pubCloud) {
    sensor_msgs::PointCloud2 mapMsgs;
    pcl::toROSMsg(*pubCloud, mapMsgs);
    mapMsgs.header.frame_id = "map";
    globalMapPub.publish(mapMsgs);
}



int makeLoopCourse(pcl::PointCloud<pcl::PointXYZI>::Ptr cloudIn, PointTypePose *pose) {
    scManager.makeAndSaveScancontextAndKeys(*cloudIn);
    std::pair<int, double> result = scManager.detectLoopClosureID();
    scManager.polarcontexts_.pop_back();
    scManager.polarcontext_invkeys_.pop_back();
    scManager.polarcontext_vkeys_.pop_back();
    scManager.polarcontext_invkeys_mat_.pop_back();
    if(result.first == -1 && result.first < keyFramePos->points.size()){
        return 0;
    }
    *pose = keyFramePos->points[result.first];
    geometry_msgs::PoseStamped poseMsgs;
    tf::Quaternion q;
    geometry_msgs::Quaternion q_msgs;
    q.setRPY(pose->roll, pose->pitch, pose->yaw);
    tf::quaternionTFToMsg(q, q_msgs);
    poseMsgs.pose.position.x = pose->x;
    poseMsgs.pose.position.y = pose->y;
    poseMsgs.pose.position.z = pose->z;
    poseMsgs.pose.orientation = q_msgs;
    poseMsgs.header.frame_id = "map";
    currPosPUb.publish(poseMsgs);

    // pub tf
    tf::TransformBroadcaster tfBroad;
    tf::Transform trans;
    trans.setRotation(q);
    trans.setOrigin(tf::Vector3(pose->x, pose->y, pose->z));
    tfBroad.sendTransform(tf::StampedTransform(trans, ros::Time::now(), "map", "laser_link"));
    return 1;
}

void
cloudHandle(const sensor_msgs::PointCloud2 msgs){
    pcl::PointCloud<pcl::PointXYZ>::Ptr msgsCloudXYZ(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr msgsCloudXYZI(new pcl::PointCloud<pcl::PointXYZI>);
    PointTypePose pose;
    pcl::fromROSMsg(msgs, *msgsCloudXYZ);
    PointXYZ2PointXYZI(msgsCloudXYZ, msgsCloudXYZI);
    if(makeLoopCourse(msgsCloudXYZI, &pose)){
        printf("loop fault\n");
    }
    printf("loop success\n");
}


int main(int argc, char** argv) {
    // init ros pub&sub
    ros::init(argc,argv,"scan_context");
    ros::NodeHandle nh;
    ros::Rate rate(1);
    sub = nh.subscribe<sensor_msgs::PointCloud2>("/lslidar_point_cloud", 10, cloudHandle);
    currFramePub = nh.advertise<sensor_msgs::PointCloud2>("scan_context/curr_frame", 10);
    globalMapPub = nh.advertise<sensor_msgs::PointCloud2>("scan_context/global_map", 10);
    currPosPUb = nh.advertise<geometry_msgs::PoseStamped>("scan_context/curr_pose",10);
    // TODO:use TF2 to set tf

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
    //  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> keyFrameCloudVector;
    for(auto filename : fileNameList){
        pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud (new pcl::PointCloud<pcl::PointXYZI>);
        pcl::io::loadPCDFile(filename, *temp_cloud);
        keyFrameCloudVector.push_back(temp_cloud);
    }
    

    puts("start load keyFramePose PCD file");

    std::string posFileName = savePCDDirectory + "/transformations.pcd";
    pcl::io::loadPCDFile(posFileName, *keyFramePos);
    puts("success load keyFramPose PCD file");

    // init scMagner
    pcl::PointCloud<pcl::PointXYZ>::Ptr globalMap(new pcl::PointCloud<pcl::PointXYZ>);
    int i = 0;
    for(auto cloud : keyFrameCloudVector){
        // add keyFrame cloud to scManager
        scManager.makeAndSaveScancontextAndKeys(*cloud);
        std::pair<int,double> result = scManager.detectLoopClosureID();
        // if(result.first != -1){
        //     scManager.polarcontexts_.pop_back();
        //     scManager.polarcontext_invkeys_.pop_back();
        //     scManager.polarcontext_vkeys_.pop_back();
        //     scManager.polarcontext_invkeys_mat_.pop_back();
        // }
        // get keyFram pose
        PointTypePose pose;
        pose = keyFramePos->points[i];
        // transformation keyFrame to correct pos
        pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud(new pcl::PointCloud<pcl::PointXYZ>);
        PointXYZI2PointXYZ(cloud, tempCloud);
        tempCloud = transformPointCloud(tempCloud, pose);
        *globalMap += *tempCloud;
        i++;
    }

    // pub global map
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    pcl::PointCloud<pcl::PointXYZ>::Ptr globalMapDS(new pcl::PointCloud<pcl::PointXYZ>);
    voxel.setLeafSize(0.25, 0.25, 0.25);
    voxel.setInputCloud(globalMap);
    voxel.filter(*globalMapDS);
    while(ros::ok()){
        pubMap(globalMapDS);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
