/*******************************************************************************
 * Copyright (C) 2017 Udacity Inc.
 *
 * This file is part of Robotic Arm: Pick and Place project for Udacity
 * Robotics nano-degree program
 *
 * All Rights Reserved.
 ******************************************************************************/

// Author: Harsh Pandya

#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include "air_object_segmentation/PcTransform.h"
#include<string>
/*
* Brief:
* This node transforms point cloud from /camera_link frame to /base_link frame
*/

//  void pclCallback(const sensor_msgs::PointCloud2ConstPtr& pcl_msg)
bool pclCallback(air_object_segmentation::PcTransformRequest &req,
                 air_object_segmentation::PcTransformResponse &resp)
{
  tf::TransformListener listener;
  tf::StampedTransform transform;
  sensor_msgs::PointCloud2::Ptr buffer;
  buffer.reset(new sensor_msgs::PointCloud2);
  buffer->header.frame_id = "base_link";
  listener.waitForTransform("base_link", "camera_color_optical_frame", ros::Time::now(), ros::Duration(3.0));
  listener.lookupTransform("/base_link", "/camera_color_optical_frame",
                             ros::Time(0), transform);
  pcl_ros::transformPointCloud("camera_color_optical_frame", transform, req.input_pc, *buffer);
  resp.response_pc = *buffer;
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "point_cloud_tf_service");
  ros::NodeHandle nh;

  ros::ServiceServer service = nh.advertiseService("transform_pc_srv", pclCallback);

  ROS_INFO_STREAM("Ready to transform PointCloud!!!");

  ros::spin();

  return 0;
}
