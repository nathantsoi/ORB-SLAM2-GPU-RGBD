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

Usage: 

./csi_camera path_to_vocabulary path_to_settings WIDTH HEIGHT FPS TIME 


*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <iomanip>

#include<opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include<ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include<tf/transform_listener.h>

#include"std_msgs/Float32.h"
#include"std_msgs/Float32MultiArray.h"
#include"geometry_msgs/PoseStamped.h"
#include"../../../include/System.h"
#include"../../../include/Converter.h"
#include<../../../include/Utils.hpp>

//#include<opencv2/core/opengl.hpp>
//#include<opencv2/cudacodec.hpp>

using namespace std;

/*
#define SET_CLOCK(t0) \
        std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();


#define TIME_DIFF(t1, t0) \
        (std::chrono::duration_cast<std::chrono::duration<double>>((t1) - (t0)).count())
*/

std::string get_tegra_pipeline(int width, int height, int fps) {
    return "nvcamerasrc ! video/x-raw(memory:NVMM), width=(int)" + std::to_string(width) + ", height=(int)" +
           std::to_string(height) + ", format=(string)I420, framerate=(fraction)" + std::to_string(fps) +
           "/1 ! nvtee ! nvvidconv flip-method=0 ! video/x-raw, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
}

bool operator ! (const cv::Mat&m) { return m.empty();}

//appsink

int main(int argc, char **argv)
{

    ros::init(argc, argv, "csi");
    ros::start();

    if(argc < 3)
    {
        cerr << endl << "Usage: ./csi_camera path_to_vocabulary path_to_settings" << endl;
        return 1;
    } else if (argc > 7) {
        cerr << endl << "Usage: ./csi_camera path_to_vocabulary path_to_settings" << endl;
        return 1;
    }

    int WIDTH, HEIGHT, FPS;
    double TIME; 
    if (argc > 3) WIDTH = std::atoi(argv[3]); else WIDTH = 640;
    if (argc > 4) HEIGHT = std::atoi(argv[4]); else HEIGHT = 480;
    if (argc > 5) FPS = std::atoi(argv[5]); else FPS = 30;
    if (argc > 6) TIME = std::atof(argv[6]); else TIME = 30.0;


    // Define the gstream pipelin 
    std::string pipeline = get_tegra_pipeline(WIDTH, HEIGHT, FPS);
    std::cout << "Using pipeline: \n\t" << pipeline << "\n";

    // Create OpenCV capture object, ensure it works.
    cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);
    if (!cap.isOpened()) {
        std::cout << "Connection failed";
        return -1;
    }

    bool bUseViz = true;

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,bUseViz);

    // Initialize publisher 
    ros::NodeHandle nodeHandler;
    ros::Publisher pose_pub = nodeHandler.advertise<geometry_msgs::PoseStamped>("orb_pose", 10);
    ros::Rate loop_rate(30);


    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;


    double tsum = 0;
    double tbuf[10] = {0.0};
    int tpos = 0;
    double trackTimeSum = 0.0;
    // Main loop
    cv::Mat im;

    //cv::cuda::GpuMat im2;
    //cv::Ptr<cv::cudacodec::VideoReader> d_reader = cv::cudacodec::createVideoReader(pipeline, cv::CAP_GSTREAMER);
    // If I want to feed a GpuMat directly to the frame 

    SET_CLOCK(t0);
    int frameNumber = 0;
    while (ros::ok()) {
      cap >> im;  

      //d_reader >> im2;
 
      if (im.empty()) continue;
      SET_CLOCK(t1);
      double tframe = TIME_DIFF(t1, t0);
      if (tframe > TIME) {
        break;
      }

      PUSH_RANGE("Track image", 4);
      // Pass the image to the SLAM system
      cv::Mat Tcw = SLAM.TrackMonocular(im,tframe);
      POP_RANGE;
      SET_CLOCK(t2);

      double trackTime = TIME_DIFF(t2, t1);
      trackTimeSum += trackTime;
      tsum = tframe - tbuf[tpos];
      tbuf[tpos] = tframe;
      tpos = (tpos + 1) % 10;
      //cerr << "Frame " << frameNumber << " : " << tframe << " " << trackTime << " " << 10 / tsum << "\n";
      ++frameNumber;

      // Now publish the 6DOF pose 
      geometry_msgs::PoseStamped pose;
      tf::Transform new_transform;
      //pose.header.stamp = ros::Time::Time(tframe);
      pose.header.stamp.sec = tframe;
      pose.header.frame_id ="map";

      if(!Tcw==1) { //||!vTcw==1){
          tf::Quaternion quaternion(1, 0 , 0, 0);

          new_transform.setRotation(quaternion);
          tf::poseTFToMsg(new_transform, pose.pose);

          /*
          for(int i=0;i<15;i++){
              cout<<"test"<<endl;
              mVel.data.push_back(vTcw.at<float>(i));
          } 
          */       
      }
      else {
          cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t(); // Rotation information
          cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3); // translation information
          vector<float> q1 = ORB_SLAM2::Converter::toQuaternion(Rwc);
          new_transform.setOrigin(tf::Vector3(twc.at<float>(0, 0), twc.at<float>(0, 1), twc.at<float>(0, 2)));

          tf::Quaternion quaternion(q1[0], q1[1], q1[2], q1[3]);
          new_transform.setRotation(quaternion);

          tf::poseTFToMsg(new_transform, pose.pose);
      }
      pose_pub.publish(pose);

      ros::spinOnce();
      loop_rate.sleep();
    }
    // Stop all threads
    SLAM.Shutdown();

    cerr << "Mean track time: " << trackTimeSum / frameNumber << " , mean fps: " << frameNumber / TIME << "\n";


    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;
}

