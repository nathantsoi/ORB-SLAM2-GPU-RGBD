/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include<tf/transform_listener.h>
#include<opencv2/core/core.hpp>

#include"std_msgs/Float32.h"
#include"std_msgs/Float32MultiArray.h"
#include"geometry_msgs/PoseStamped.h"
#include"../../../include/System.h"
#include"../../../include/Converter.h"

using namespace std;


bool operator ! (const cv::Mat&m) { return m.empty();}
class rosNode
{
public:
    rosNode(ros::NodeHandle& nh, ORB_SLAM2::System* pSLAM);
    bool spin();
    void GrabImage(const sensor_msgs::ImageConstPtr& msg);
    void publish_pose(geometry_msgs::PoseStamped& Pose);
    void publish_mVel(geometry_msgs::PoseStamped& mVel);
    ORB_SLAM2::System* mpSLAM;
private:
    cv::Mat vTcw;
    ros::NodeHandle nodeHandler;
    ros::Publisher pose_pub;
    ros::Publisher mVel_pub;
    ros::Subscriber cam_sub;
    ros::Rate rate;
    
};

rosNode::rosNode(ros::NodeHandle& nh, ORB_SLAM2::System* pSLAM):rate((float)20){
    mpSLAM=pSLAM;
    vTcw=mpSLAM->mVelocity.clone();
    nodeHandler=nh;
    pose_pub=nodeHandler.advertise<geometry_msgs::PoseStamped>("orb_pose", 10);
    mVel_pub=nodeHandler.advertise<std_msgs::Float32MultiArray>("orb_mVel", 10);
    cam_sub=nodeHandler.subscribe("camera/image_raw",10,&rosNode::GrabImage, this);
}

bool rosNode::spin (){
    while(!ros::isShuttingDown()) {
        while(nodeHandler.ok()) {
            ros::spinOnce();
            rate.sleep();
        }
    }
    return 0;
}

void rosNode::publish_pose(geometry_msgs::PoseStamped& Pose){
    pose_pub.publish(Pose);
}
void rosNode::publish_mVel(geometry_msgs::PoseStamped& mVel){
    mVel_pub.publish(mVel);
}

void rosNode::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    

    cv::Mat Tcw = mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());
    
    vTcw=mpSLAM->mVelocity.clone();
    cout<<"Tcw= "<<endl<<" "<<Tcw<<endl<<endl;
    cout<<"vTcw= "<<endl<<" "<<vTcw<<endl<<endl;

    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id ="map";


    std_msgs::Float32MultiArray mVel;
    
    tf::Transform new_transform;
    /*
    mVel.layout.dim[0].label="rows";
    cout<<"test"<<endl;
    mVel.layout.dim[0].size=4;
    cout<<"test"<<endl;
    mVel.layout.dim[0].stride=16;
    cout<<"test"<<endl;
    mVel.layout.dim[1].label="columns";
    cout<<"test"<<endl;
    mVel.layout.dim[1].size=4;
    cout<<"test"<<endl;
    mVel.layout.dim[1].stride=4;
    cout<<"test"<<endl;
    */
    // test if 

    if(!Tcw==1||!vTcw==1){
        tf::Quaternion quaternion(1, 0 , 0, 0);

        new_transform.setRotation(quaternion);
        tf::poseTFToMsg(new_transform, pose.pose);

        cout<<"test"<<endl;
        for(int i=0;i<15;i++){
            cout<<"test"<<endl;
            mVel.data.push_back(vTcw.at<float>(i));
        } 
        cout<<"test"<<endl;       
    }
    else{
    cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t(); // Rotation information
    cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3); // translation information
    vector<float> q1 = ORB_SLAM2::Converter::toQuaternion(Rwc);
   
        for(int i=0;i<15;i++){
            mVel.data.push_back(vTcw.at<float>(i));
        }
    
    new_transform.setOrigin(tf::Vector3(twc.at<float>(0, 0), twc.at<float>(0, 1), twc.at<float>(0, 2)));

    tf::Quaternion quaternion(q1[0], q1[1], q1[2], q1[3]);
    new_transform.setRotation(quaternion);

    tf::poseTFToMsg(new_transform, pose.pose);
    }
    cout<<"test"<<endl;
    pose_pub.publish(pose);
    cout<<"test"<<endl;
    mVel_pub.publish(mVel);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono");
    ros::start();
    // Create SLAM system. It initializes all system threads and gets ready to process frames.

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Mono path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    } 
    // Create SLAM system. It initializes all system threads and gets ready to process frames.   
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

    ros::NodeHandle nh;
    rosNode igb(nh, &SLAM);
    
    igb.spin();
    
    //stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();
    return 0;
}



