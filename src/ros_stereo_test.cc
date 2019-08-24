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
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_broadcaster.h>

#include<opencv2/core/core.hpp>

#include"System.h"
#include"RosPublisher.h"

using namespace std;
cv::Mat M1l,M2l,M1r,M2r;

class ImageGrabberStero : public RosPublisher
{
public:
    ImageGrabberStero (const ORB_SLAM2::System::eSensor sensor, ros::NodeHandle &node_handle, image_transport::ImageTransport &image_transport);
    ~ImageGrabberStero ();
    void ImageCallback (const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight);

    bool do_rectify;

private:
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Subscriber<sensor_msgs::Image> *left_sub_;
    message_filters::Subscriber<sensor_msgs::Image> *right_sub_;
    message_filters::Synchronizer<sync_pol> *sync_;

};

int main(int argc, char **argv)
{
    int count = 0;
    ros::init(argc, argv, "Stereo");
    ros::start();

    if(argc > 1) {

    }

    ros::NodeHandle node_handle;

    if(count==0)
    {
        cout<<"Entered rectify\n";
        std::string calibration;
        node_handle.param<std::string>(ros::this_node::getName() + "/settings_file", calibration, "file_not_set");
        cv::FileStorage fsSettings(calibration, cv::FileStorage::READ);
        if(!fsSettings.isOpened())
        {
            cerr << "ERROR: Wrong path to settings" << endl;
            return -1;
        }

        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
        fsSettings["LEFT.K"] >> K_l;
        fsSettings["RIGHT.K"] >> K_r;

        fsSettings["LEFT.P"] >> P_l;
        fsSettings["RIGHT.P"] >> P_r;

        fsSettings["LEFT.R"] >> R_l;
        fsSettings["RIGHT.R"] >> R_r;

        fsSettings["LEFT.D"] >> D_l;
        fsSettings["RIGHT.D"] >> D_r;

        int rows_l = fsSettings["LEFT.height"];
        int cols_l = fsSettings["LEFT.width"];
        int rows_r = fsSettings["RIGHT.height"];
        int cols_r = fsSettings["RIGHT.width"];

        if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
                rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0)
        {
            cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
            return -1;
        }

        cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,M1l,M2l);
        cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,M1r,M2r);
        count++;
    }
    image_transport::ImageTransport image_transport (node_handle);

    // initilaize
    ImageGrabberStero node (ORB_SLAM2::System::STEREO, node_handle, image_transport);

    ros::spin();

    return 0;
}

ImageGrabberStero::ImageGrabberStero (const ORB_SLAM2::System::eSensor sensor, ros::NodeHandle &node_handle, image_transport::ImageTransport &image_transport) : RosPublisher (sensor, node_handle, image_transport) {
    left_sub_ = new message_filters::Subscriber<sensor_msgs::Image> (node_handle, "image_left/image_color_rect", 1);
    right_sub_ = new message_filters::Subscriber<sensor_msgs::Image> (node_handle, "image_right/image_color_rect", 1);

    node_handle.param(ros::this_node::getName()+ "/do_rectfy", do_rectify, true);

    sync_ = new message_filters::Synchronizer<sync_pol> (sync_pol(10), *left_sub_, *right_sub_);
    sync_->registerCallback(boost::bind(&ImageGrabberStero::ImageCallback, this, _1, _2));
}

ImageGrabberStero::~ImageGrabberStero () {
    delete left_sub_;
    delete right_sub_;
    delete sync_;
}

void ImageGrabberStero::ImageCallback (const sensor_msgs::ImageConstPtr& msgLeft, const sensor_msgs::ImageConstPtr& msgRight) {
  cv_bridge::CvImageConstPtr cv_ptrLeft;
  try {
      cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
  } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
  }

  cv_bridge::CvImageConstPtr cv_ptrRight;
  try {
      cv_ptrRight = cv_bridge::toCvShare(msgRight);
  } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
  }

  if(do_rectify)
  {
    cv::Mat imLeft, imRight;
    cv::remap(cv_ptrLeft->image,imLeft,M1l,M2l,cv::INTER_LINEAR);
    cv::remap(cv_ptrRight->image,imRight,M1r,M2r,cv::INTER_LINEAR);
    orb_slam_->TrackStereo(imLeft,imRight,cv_ptrLeft->header.stamp.toSec());
  }
  else
  {
    orb_slam_->TrackStereo(cv_ptrLeft->image,cv_ptrRight->image,cv_ptrLeft->header.stamp.toSec());
  }
  


  current_frame_time_ = msgLeft->header.stamp;


  Update ();
}