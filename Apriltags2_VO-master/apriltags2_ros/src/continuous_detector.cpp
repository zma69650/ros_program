/**
 * Copyright (c) 2017, California Institute of Technology.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are
 * those of the authors and should not be interpreted as representing official
 * policies, either expressed or implied, of the California Institute of
 * Technology.
 */

#include "apriltags2_ros/continuous_detector.h"
#include "fstream"
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(apriltags2_ros::ContinuousDetector, nodelet::Nodelet);

namespace apriltags2_ros
{

const int tag_num = 13;

tag_setting::tag_setting(){
        if_set = false;
        tag_pose.setIdentity();
        my_count = 0;
}

ContinuousDetector::ContinuousDetector ()
{
    tags = new tag_setting[tag_num];
    tag_id = new int[tag_num];
    odom_pose.setIdentity();
}

void compute_pose(nav_msgs::Odometry& pose, Eigen::Isometry3d &odom, AprilTagDetectionArray& April, int i) {
    nav_msgs::Odometry temp;
    pose.header = April.detections[i].pose.header;
    //odometry.header.frame_id = "my_bundle";
    pose.pose.pose = April.detections[i].pose.pose.pose;
    pose.pose.pose.position.y = -pose.pose.pose.position.y;
    double temp_x = 100*(pose.pose.pose.position.x - odom.translation().x());
    double temp_y = 100*(pose.pose.pose.position.y - odom.translation().y());
    temp.pose.pose.position.x = temp_x > 0?log(1.0+fabs(temp_x)):-log(1.0+fabs(temp_x));
    temp.pose.pose.position.y = temp_y > 0?log(1.0+fabs(temp_y)):-log(1.0+fabs(temp_y));
    pose.pose.pose.position.x = odom.translation().x()+0.01*temp.pose.pose.position.x;
    pose.pose.pose.position.y = odom.translation().y()+0.01*temp.pose.pose.position.y;
    std::cout << "error:" << 0.01*temp.pose.pose.position.x << ' ' << 0.01*temp.pose.pose.position.y << std::endl;
    pose.pose.pose.position.z = 0;
}

void ContinuousDetector::onInit ()
{
  ros::NodeHandle& nh = getNodeHandle();
  ros::NodeHandle& pnh = getPrivateNodeHandle();

  tag_detector_ = std::shared_ptr<TagDetector>(new TagDetector(pnh));
  draw_tag_detections_image_ = getAprilTagOption<bool>(pnh, 
      "publish_tag_detections_image", false);

  it_ = std::shared_ptr<image_transport::ImageTransport>(
      new image_transport::ImageTransport(nh));

  camera_image_subscriber_ =
      it_->subscribeCamera("image_rect", 1,
                          &ContinuousDetector::imageCallback, this);

  odom_sub =
          nh.subscribe("odom", 10, &ContinuousDetector::odomCallback, this);

  tag_detections_publisher_ =
      nh.advertise<AprilTagDetectionArray>("tag_detections", 1);

  if (draw_tag_detections_image_)
  {
    tag_detections_image_publisher_ = it_->advertise("tag_detections_image", 1);
  }

  odomtry_publisher_ = nh.advertise<nav_msgs::Odometry>("tag_Odometry", 1);
  path_pubilsher = nh.advertise<nav_msgs::Path>("path", 1);

}

void ContinuousDetector::imageCallback (
    const sensor_msgs::ImageConstPtr& image_rect,
    const sensor_msgs::CameraInfoConstPtr& camera_info)
{
  // Convert ROS's sensor_msgs::Image to cv_bridge::CvImagePtr in order to run
  // AprilTags 2 on the iamge
  try
  {
    cv_image_ = cv_bridge::toCvCopy(image_rect,
                                    sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // Publish detected tags in the image by AprilTags 2
  AprilTagDetectionArray tag_detection_array(tag_detector_->detectTags(cv_image_,camera_info,tag_id));
  tag_detections_publisher_.publish(tag_detection_array);
    //std::cout << "imgcallback_pose  :\n"<<odom_pose.translation()<<std::endl;
  //static std::ofstream creame_data("/home/lab306/Documents/creame.txt",std::ios::out);
  int count = tag_detection_array.detections.size();
  if (count>0) {
      /*for(int i=0; i < count-1; i++)
      {
          for(int j=0; j < count-1-i; j++) {
              if(tag_id[j] > tag_id[j+1])
              {
                  int temp = tag_id[j];
                  tag_id[j] = tag_id[j+1];
                  tag_id[j+1] = temp;
              }
          }
      }*/
      //zarray_sort(tag_detection_array,&id_compare);
    for (unsigned int i = 0; i < count; i++) {
      nav_msgs::Odometry odometry;
      odometry.header = tag_detection_array.detections[i].pose.header;
      //odometry.header.frame_id = "my_bundle";
      //odometry.pose.pose = tag_detection_array.detections[i].pose.pose.pose;
        compute_pose(odometry,odom_pose,tag_detection_array,i);
      //odomtry_publisher_.publish(odometry);
      //std::cout << "cd" << i << ' ' << tag_id[i] << std::endl;
      //std::cout <<tag_detection_array.detections.size()<<' ' <<i<<' ' <<tag_id[i]<<"   pose  :\n"<<odometry.pose.pose<<std::endl;
      geometry_msgs::PoseStamped pose_stamped;
      pose_stamped.header = odometry.header;
      pose_stamped.header.frame_id = "base_link";

      /*Eigen::Isometry3d tag_temp;
      tag_temp.setIdentity();
      Eigen::Vector3d tran_temp;
      tran_temp.x() = odometry.pose.pose.position.x;
      tran_temp.y() = odometry.pose.pose.position.y;
      tran_temp.z() = odometry.pose.pose.position.z;
      Eigen::Quaternion<double> quat_temp;
      quat_temp.x() = odometry.pose.pose.orientation.x;
      quat_temp.y() = odometry.pose.pose.orientation.y;
      quat_temp.z() = odometry.pose.pose.orientation.z;
      quat_temp.w() = odometry.pose.pose.orientation.w;



      tag_temp.pretranslate(tran_temp);
      tag_temp.rotate(quat_temp.toRotationMatrix().transpose());
      //tag_temp.translation().y() = tag_temp.translation().y();
      tag_temp = tag_temp.inverse();

      //std::cout << "temp_pose  :\n"<<tag_temp.translation()<<std::endl;

      if(!tags[tag_id[i]].if_set) {
          tags[tag_id[i]].my_count++;
          if(tags[tag_id[i]].my_count > 5) {
              Eigen::Isometry3d temp1, temp2;
              temp1.setIdentity();
              temp2.setIdentity();
              temp1 = odom_pose * tag_temp;
              temp2 = odom_pose.inverse() * tag_temp;
              tags[tag_id[i]].tag_pose.translate(temp1.translation());
              tags[tag_id[i]].tag_pose.rotate(temp2.rotation());
              //tags[tag_id[i]].tag_pose.translation().y() = -tags[tag_id[i]].tag_pose.translation().y();
              //tags[tag_id[i]].tag_pose.translation() =   tags[tag_id[i]].tag_pose.translation() - odom_pose.translation();
              //tags[tag_id[i]].tag_pose.translation() = -tags[tag_id[i]].tag_pose.translation();
              //tags[tag_id[i]].tag_pose.translation().y() = -tags[tag_id[i]].tag_pose.translation().y();
              tags[tag_id[i]].if_set = true;
              std::cout << "set " << tag_id[i] << " tag's pose" << std::endl;
              std::cout << "tag_pose  :\n" << tags[tag_id[i]].tag_pose.translation() << std::endl;
              std::cout << tags[tag_id[i]].tag_pose.rotation() << std::endl;
              odometry.pose.pose.position.x = tags[tag_id[i]].tag_pose.translation().x();
              odometry.pose.pose.position.y = tags[tag_id[i]].tag_pose.translation().y();
              odometry.pose.pose.position.z = 0.0;
              Eigen::Quaterniond q(tags[tag_id[i]].tag_pose.rotation());
              odometry.pose.pose.orientation.x = q.x();
              odometry.pose.pose.orientation.y = q.y();
              odometry.pose.pose.orientation.z = q.z();
              odometry.pose.pose.orientation.w = q.w();
          }
      }
      else {
          //std::cout << "odom_pose  :\n"<<odom_pose.translation()<<std::endl;
          Eigen::Isometry3d my_pose;
          my_pose.setIdentity();
          //std::cout << "mypose1  :\n"<<my_pose.translation()<<std::endl;// << ' ' <<pose_stamped.pose.position.y << std::endl;
          //std::cout << "tag_pose  :\n"<<tags[tag_id[i]].tag_pose.translation()<<std::endl;
          my_pose = tags[tag_id[i]].tag_pose * tag_temp.inverse();
          //std::cout << "mypose  :\n"<<my_pose.translation()<<std::endl;
/*
      pose_stamped.pose.position.x = tag_detection_array.detections[i].pose.pose.pose.position.x;
      pose_stamped.pose.position.y = tag_detection_array.detections[i].pose.pose.pose.position.y;
      pose_stamped.pose.position.z = tag_detection_array.detections[i].pose.pose.pose.position.z;*//*

          pose_stamped.pose.position.x = my_pose.translation().x();
          pose_stamped.pose.position.y = -my_pose.translation().y();
          pose_stamped.pose.position.z = 0;*/

          pose_stamped.pose = odometry.pose.pose;
          //pose_stamped.pose.position.y = -pose_stamped.pose.position.y;
          camera_path.header = pose_stamped.header;
          camera_path.header.frame_id = "base_link";
          camera_path.poses.push_back(pose_stamped);

          path_pubilsher.publish(camera_path);
      //odomtry_publisher_.publish(odometry);

      //std::cout << "Robot pose:\n"<<pose_stamped.pose.position.x << ' ' <<pose_stamped.pose.position.y << ' ' <<pose_stamped.pose.position.z <<std::endl;
    }
  }

  // Publish the camera image overlaid by outlines of the detected tags and
  // their payload values
  if (draw_tag_detections_image_)
  {
    tag_detector_->drawDetections(cv_image_);
    tag_detections_image_publisher_.publish(cv_image_->toImageMsg());
  }
}

void ContinuousDetector::odomCallback(const nav_msgs::Odometry &odom){//get the odom pose
    //ROS_INFO("subscribe odom info: %f  %f",(odom.pose.pose.position.x,odom.pose.pose.position.y));
    Eigen::Vector3d odom_trans;
    odom_trans.x() = odom.pose.pose.position.x;
    odom_trans.y() = odom.pose.pose.position.y;
    odom_trans.z() = 0.0;

    Eigen::Quaternion<double> odom_quat;
    odom_quat.x()=odom.pose.pose.orientation.x;
    odom_quat.y()=odom.pose.pose.orientation.y;
    odom_quat.z()=odom.pose.pose.orientation.z;
    odom_quat.w()=odom.pose.pose.orientation.w;
    odom_pose.setIdentity();
    odom_pose.pretranslate(odom_trans);
    odom_pose.rotate(odom_quat.toRotationMatrix());

    //std::cout << "odomcallback_pose  :\n"<<odom_pose.translation()<<std::endl;
}

} // namespace apriltags2_ros
