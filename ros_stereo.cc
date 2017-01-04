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

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include<Eigen/Dense>

#include <opencv2/core/core.hpp>
#include <math.h>
#include "../../../include/System.h"

using namespace std;
using namespace cv;

// Checks if a matrix is a valid rotation matrix.
bool isRotationMatrix(Mat &R)
{
    Mat Rt;
    transpose(R, Rt);
    Mat shouldBeIdentity = Rt * R;
    Mat I = Mat::eye(3, 3, shouldBeIdentity.type());

    return norm(I, shouldBeIdentity) < 1e-6;
}

// Calculates rotation matrix to euler angles
// The result is the same as MATLAB except the order
// of the euler angles ( x and z are swapped ).
Vec3f rotationMatrixToEulerAngles(Mat &R)
{

    assert(isRotationMatrix(R));

    float sy = sqrt(R.at<float>(0, 0) * R.at<float>(0, 0) + R.at<float>(1, 0) * R.at<float>(1, 0));

    bool singular = sy < 1e-6; // If
    float x, y, z;
    if (!singular)
    {
        x = atan2(R.at<float>(2, 1), R.at<float>(2, 2));
        y = atan2(-R.at<float>(2, 0), sy);
        z = atan2(R.at<float>(1, 0), R.at<float>(0, 0));
    }
    else
    {
        x = atan2(-R.at<float>(1, 2), R.at<float>(1, 1));
        y = atan2(-R.at<float>(2, 0), sy);
        z = 0;
    }
    return Vec3f(x, y, z);
}

Eigen::Matrix<double,3,3> toMatrix3d(const cv::Mat &cvMat3)
{
    Eigen::Matrix<double,3,3> M;

    M << cvMat3.at<float>(0,0), cvMat3.at<float>(0,1), cvMat3.at<float>(0,2),
         cvMat3.at<float>(1,0), cvMat3.at<float>(1,1), cvMat3.at<float>(1,2),
         cvMat3.at<float>(2,0), cvMat3.at<float>(2,1), cvMat3.at<float>(2,2);

    return M;
}

class visualOdomPublisher
{
    public:
        visualOdomPublisher(){
            first_flag = 1;
            ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("visual_odom", 30);
            tf::TransformBroadcaster odom_broadcaster;

            };

        void setNew(float x, float y, float theta, float stamp){
            cur_x = x;
            cur_y = y;
            cur_theta = theta;
            cur_stamp = stamp;
        }

        nav_msgs::Odometry update(float x, float y, float theta, float stamp)
        {
            setNew(x, y, theta, stamp);
            nav_msgs::Odometry odom;
            if (first_flag == 0)
            {
                compute_v();
                //printf("vx = %5.3f vy = %5.3f vtheta=%5.3f\n",v_x, v_y, v_theta);
                ros::Time current_time = ros::Time::now();
                geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(cur_theta);
                geometry_msgs::TransformStamped odom_trans;
                odom_trans.header.stamp = current_time;
                odom_trans.header.frame_id = "visual_odom";
                odom_trans.child_frame_id = "base_footprint";
                odom_trans.transform.translation.x = cur_x;
                odom_trans.transform.translation.y = cur_y;
                odom_trans.transform.translation.z = 0.0;
                odom_trans.transform.rotation = odom_quat;
                odom_broadcaster.sendTransform(odom_trans);
                odom.header.stamp = current_time;
                odom.header.frame_id = "visual_odom";
                odom.pose.pose.position.x = cur_x;
                odom.pose.pose.position.y = cur_y;
                odom.pose.pose.position.z = 0.0;
                odom.pose.pose.orientation = odom_quat;
                odom.child_frame_id = "base_footprint";
                odom.twist.twist.linear.x = v_x;
                odom.twist.twist.linear.y = v_y;
                odom.twist.twist.angular.z = v_theta;
                odom_pub.publish(odom);
                pre_x = cur_x;
                pre_y = cur_y;
                pre_theta = cur_theta;
                pre_stamp = cur_stamp;
            }
            else{
                pre_x = cur_x;
                pre_y = cur_y;
                pre_theta = cur_theta;
                pre_stamp = cur_stamp;
                first_flag = 0;
            }
            return odom;
        }

        float v_x, v_y, v_theta;

    protected:

    void compute_v(){
            d_x = cur_x - pre_x;
            d_y = cur_y - pre_y;
            d_theta = cur_theta - pre_theta;
            d_stamp = cur_stamp - pre_stamp;
            v_x = (d_x * cos(cur_theta) + d_y * sin(cur_theta)) / d_stamp;
            v_y = (d_y * cos(cur_theta) - d_x * sin(cur_theta)) / d_stamp;
            v_theta = d_theta / d_stamp;
        }
    
    bool first_flag;
    float pre_x, pre_y, pre_theta, pre_stamp;
    float cur_x, cur_y, cur_theta, cur_stamp;
    float d_x, d_y, d_theta, d_stamp;
    ros::NodeHandle n;
    ros::Publisher odom_pub;
    tf::TransformBroadcaster odom_broadcaster;
};

class ImageGrabber
{
  public:
    ImageGrabber(ORB_SLAM2::System *pSLAM) : mpSLAM(pSLAM) {}

    void GrabStereo(const sensor_msgs::ImageConstPtr &msgLeft, const sensor_msgs::ImageConstPtr &msgRight);

    ORB_SLAM2::System *mpSLAM;
    bool do_rectify;
    cv::Mat M1l, M2l, M1r, M2r;
    visualOdomPublisher vop = visualOdomPublisher();
    ros::NodeHandle nh;
    ros::Publisher p_pub = nh.advertise<geometry_msgs::Pose>("robot_pose", 1);
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("visual_odom", 30);
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();

    if (argc != 4)
    {
        cerr << endl
             << "Usage: rosrun ORB_SLAM2 Stereo path_to_vocabulary path_to_settings do_rectify" << endl;
        ros::shutdown();
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::STEREO, true);

    ImageGrabber igb(&SLAM);

    stringstream ss(argv[3]);
    ss >> boolalpha >> igb.do_rectify;

    if (igb.do_rectify)
    {
        // Load settings related to stereo calibration
        cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
        if (!fsSettings.isOpened())
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

        if (K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
            rows_l == 0 || rows_r == 0 || cols_l == 0 || cols_r == 0)
        {
            cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
            return -1;
        }

        cv::initUndistortRectifyMap(K_l, D_l, R_l, P_l.rowRange(0, 3).colRange(0, 3), cv::Size(cols_l, rows_l), CV_32F, igb.M1l, igb.M2l);
        cv::initUndistortRectifyMap(K_r, D_r, R_r, P_r.rowRange(0, 3).colRange(0, 3), cv::Size(cols_r, rows_r), CV_32F, igb.M1r, igb.M2r);
    }

    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::Image> left_sub(nh, "/stereo/left/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> right_sub(nh, "/stereo/right/image_raw", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), left_sub, right_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabStereo, &igb, _1, _2));

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory_TUM_Format.txt");
    //SLAM.SaveTrajectoryTUM("FrameTrajectory_TUM_Format.txt");
    //SLAM.SaveTrajectoryKITTI("FrameTrajectory_KITTI_Format.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabStereo(const sensor_msgs::ImageConstPtr &msgLeft, const sensor_msgs::ImageConstPtr &msgRight)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrLeft;
    try
    {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrRight;
    try
    {
        cv_ptrRight = cv_bridge::toCvShare(msgRight);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    if (do_rectify)
    {
        cv::Mat imLeft, imRight;
        cv::remap(cv_ptrLeft->image, imLeft, M1l, M2l, cv::INTER_LINEAR);
        cv::remap(cv_ptrRight->image, imRight, M1r, M2r, cv::INTER_LINEAR);
        mpSLAM->TrackStereo(imLeft, imRight, cv_ptrLeft->header.stamp.toSec());
    }
    else
    {
        Mat Tcw = mpSLAM->TrackStereo(cv_ptrLeft->image, cv_ptrRight->image, cv_ptrLeft->header.stamp.toSec());
        if (!Tcw.empty())
        {
            cv::Mat Rcw = Tcw.rowRange(0, 3).colRange(0, 3);
            cv::Mat tcw = Tcw.rowRange(0, 3).col(3);
            cv::Mat Rwc = Rcw.t();
            cv::Mat Ow = -Rwc * tcw;

            cv::Mat Twc = cv::Mat::eye(4, 4, Tcw.type());
            Rwc.copyTo(Twc.rowRange(0, 3).colRange(0, 3));
            Ow.copyTo(Twc.rowRange(0, 3).col(3));
            cv::Mat center = (cv::Mat_<float>(4, 1) << 0.05, 0, -0.175, 1);
            cv::Mat Cw = Twc * center;
            //Vec3f t = rotationMatrixToEulerAngles(Rwc);

            geometry_msgs::PoseStamped pose_stamped;
            pose_stamped.header.frame_id = "map_frame";
            pose_stamped.header.stamp = ros::Time::now();

            tf::Matrix3x3 m(Twc.at<float>(0,0), Twc.at<float>(0,1), Twc.at<float>(0,2),
            Twc.at<float>(1,0), Twc.at<float>(1,1), Twc.at<float>(1,2),
            Twc.at<float>(2,0), Twc.at<float>(2,1), Twc.at<float>(2,2));

            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            if(abs(roll) > 2.0){
                if(pitch > 0){
                    pitch = M_PI - pitch;
                }
                else{
                    pitch = -M_PI - pitch;
                }
            }
            //std::cout<<std::endl<<roll<<std::endl;
            //std::cout<<pitch<<std::endl;
            //std::cout<<yaw<<std::endl;
            yaw = pitch;

            //std::cout<<yaw<<std::endl;

            tf::Quaternion quaternion = tf::createQuaternionFromRPY(0, 0, -yaw);
            pose_stamped.pose.orientation.x = quaternion.x();
            pose_stamped.pose.orientation.y = quaternion.y();
            pose_stamped.pose.orientation.z = quaternion.z();
            pose_stamped.pose.orientation.w = quaternion.w();


            pose_stamped.pose.position.x =  Cw.at<float>(2);
            pose_stamped.pose.position.y =  -Cw.at<float>(0);
            pose_stamped.pose.position.z = 0;
            p_pub.publish(pose_stamped.pose);
    
            nav_msgs::Odometry odom = vop.update(Cw.at<float>(2), -Cw.at<float>(0), -yaw, (float)cv_ptrLeft->header.stamp.toSec());
            odom_pub.publish(odom);
        }
    }
}
