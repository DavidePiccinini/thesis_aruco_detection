/*****************************
 Copyright 2011 Rafael Muñoz Salinas. All rights reserved.

 Redistribution and use in source and binary forms, with or without modification, are
 permitted provided that the following conditions are met:

 1. Redistributions of source code must retain the above copyright notice, this list of
 conditions and the following disclaimer.

 2. Redistributions in binary form must reproduce the above copyright notice, this list
 of conditions and the following disclaimer in the documentation and/or other materials
 provided with the distribution.

 THIS SOFTWARE IS PROVIDED BY Rafael Muñoz Salinas ''AS IS'' AND ANY EXPRESS OR IMPLIED
 WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL Rafael Muñoz Salinas OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 The views and conclusions contained in the software and documentation are those of the
 authors and should not be interpreted as representing official policies, either expressed
 or implied, of Rafael Muñoz Salinas.
 ********************************/
/**
 * @file marker_publish.cpp
 * @author Bence Magyar
 * @date June 2014
 * @brief Modified copy of simple_single.cpp to publish all markers visible
 * (modified by Josh Langsfeld, 2014)
 */

#include <iostream>
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <aruco_msgs/IdentifiedMarkersIDs.h>

class ArucoMarkerPublisher
{
private:
  // ArUco stuff
  aruco::MarkerDetector mDetector_;
  std::vector<aruco::Marker> markers_;
  aruco::CameraParameters camParam_;

  // Node params
  double marker_size_;
  double distanceThreshold; // defines the distance in m less than which markers are identified
  double norm;
  bool useCamInfo_;
  std::string imageTopic_;
  cv::Mat inImage_;
  std::vector<int> validMarkerIndices;
  std::vector<double> distance;
  
  // ROS pub-sub
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  image_transport::Publisher debug_pub_;
  ros::Publisher id_pub_;
  
  // Create the camera parameters
  cv::Mat camMatrix_ = (cv::Mat_<double>(3, 3) << 274.977861325, 0, 274.798529539, 0, 275.08073962, 249.626065136, 0, 0, 1);
  cv::Mat distCoeff_ = (cv::Mat_<double>(4, 1) << 0.33994470,  0.07618041,  0.17066567, -0.04964407);
  cv::Size camSize_ = cv::Size(550, 500);
  
public:
  ArucoMarkerPublisher() :
      nh_("~"), it_(nh_), useCamInfo_(true)
  {
    // Parameters
    nh_.param<double>("marker_size", marker_size_, 0.05);
    nh_.param<bool>("use_camera_info", useCamInfo_, false);
    nh_.param<std::string>("image_topic", imageTopic_, "/image");
    
    image_sub_ = it_.subscribe(imageTopic_, 1, &ArucoMarkerPublisher::image_callback, this);
    image_pub_ = it_.advertise("result", 1);
    debug_pub_ = it_.advertise("debug", 1);
    id_pub_ = nh_.advertise<aruco_msgs::IdentifiedMarkersIDs>("identified_marker_ids", 1);
    
    camParam_ = aruco::CameraParameters(camMatrix_, distCoeff_, camSize_);
    distanceThreshold = 3.5; 
  }
  
  // Computes the l2 norm of a vector, it's used to check for identifiable markers
  double l2_norm(std::vector<double> const& u) 
  {
    double accum = 0.;
    for (int i = 0; i < u.size(); ++i) {
        accum += u[i] * u[i];
    }
    return sqrt(accum);
  }

  // Callback function for the camera image topic subscriber
  void image_callback(const sensor_msgs::ImageConstPtr& msg)
  {
    // bool publishImage = image_pub_.getNumSubscribers() > 0;
    // bool publishDebug = debug_pub_.getNumSubscribers() > 0;
    // bool publishIDs = image_pub_.getNumSubscribers() > 0;
    bool publishImage = true;
    bool publishDebug = false;
    bool publishIDs = true;

    ros::Time curr_stamp = msg->header.stamp;
    cv_bridge::CvImagePtr cv_ptr;
    std::vector<int> identifiedMarkersIDs;
    
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      inImage_ = cv_ptr->image;
      
      // Clear out previous detection results
      markers_.clear();
      validMarkerIndices.clear();
      distance.clear();
      
      // Ok, let's detect
      mDetector_.detect(inImage_, markers_, camParam_, marker_size_, true);

      // If there are detected markers in the image, check if they are close enough to be identified      
      if (markers_.size() > 0)
      {
        // Check if there are identifiable markers
        for (std::size_t i = 0; i < markers_.size(); ++i)
        {
          // Compute markers' extrinsics
          markers_[i].calculateExtrinsics(marker_size_, camParam_);
          
          // If the norm of the distance to the marker is less than the threshold then identify the marker
          markers_.at(i).Tvec.col(0).copyTo(distance);
          norm = l2_norm(distance);
          if (norm <= distanceThreshold)
          {
            // Save the indices of the identified markers
            validMarkerIndices.push_back(i);
          }
        }
          
        // Draw detected markers on the image for visualization and save their IDs
        for (std::size_t i = 0; i < validMarkerIndices.size(); ++i)
        {
          markers_[validMarkerIndices[i]].draw(inImage_, cv::Scalar(0, 0, 255), 2);
          
          identifiedMarkersIDs.push_back(markers_.at(validMarkerIndices[i]).id);
        }
      }
      
      // Publish image 
      if (publishImage)
      {
        // Show input with augmented information
        cv_bridge::CvImage out_msg;
        out_msg.header.stamp = curr_stamp;
        out_msg.encoding = sensor_msgs::image_encodings::RGB8;
        out_msg.image = inImage_;
        image_pub_.publish(out_msg.toImageMsg());
      }

      /* // publish image after internal image processing
      if (publishDebug)
      {
        // show also the internal image resulting from the threshold operation
        cv_bridge::CvImage debug_msg;
        debug_msg.header.stamp = curr_stamp;
        debug_msg.encoding = sensor_msgs::image_encodings::MONO8;
        debug_msg.image = mDetector_.getThresholdedImage();
        debug_pub_.publish(debug_msg.toImageMsg());
      } */
        
      // Publish the IDs of the identified markers
      if (publishIDs)
      {
        aruco_msgs::IdentifiedMarkersIDs id_msg;
        id_msg.ids = identifiedMarkersIDs;
        id_pub_.publish(id_msg);
      }
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
    }
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "aruco_marker_publisher");

  ArucoMarkerPublisher node;

  ros::spin();
}
