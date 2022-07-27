#include "common.h"
#include <Eigen/Core>
#include <cv_bridge/cv_bridge.h>
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <pcl/ModelCoefficients.h>
#include <pcl/common/io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sstream>
#include <std_msgs/Header.h>
#include <stdio.h>
#include <string>
#include <time.h>
#include <unordered_map>

#define width_ 3072
#define height_ 2048
#define min_depth_ 2.5
#define max_depth_ 50

void projection(
    const Vector6d &extrinsic_params,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &lidar_cloud, //projection type: intensity
    float fx_, float cx_, float fy_, float cy_, float k1_, float k2_, float p1_, float p2_, float k3_,
    cv::Mat &projection_img) {
  std::vector<cv::Point3f> pts_3d;
  std::vector<float> intensity_list;
  Eigen::AngleAxisd rotation_vector3;
  rotation_vector3 =
      Eigen::AngleAxisd(extrinsic_params[0], Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(extrinsic_params[1], Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(extrinsic_params[2], Eigen::Vector3d::UnitX());
  for (size_t i = 0; i < lidar_cloud->size(); i++) {
    pcl::PointXYZI point_3d = lidar_cloud->points[i];
    float depth =
        sqrt(pow(point_3d.x, 2) + pow(point_3d.y, 2) + pow(point_3d.z, 2));
    if (depth > min_depth_ && depth < max_depth_) {
      pts_3d.emplace_back(cv::Point3f(point_3d.x, point_3d.y, point_3d.z));
      intensity_list.emplace_back(lidar_cloud->points[i].intensity);
    }
  }
  cv::Mat camera_matrix =
      (cv::Mat_<double>(3, 3) << fx_, 0.0, cx_, 0.0, fy_, cy_, 0.0, 0.0, 1.0);
  cv::Mat distortion_coeff =
      (cv::Mat_<double>(1, 5) << k1_, k2_, p1_, p2_, k3_);
  cv::Mat r_vec =
      (cv::Mat_<double>(3, 1)
           << rotation_vector3.angle() * rotation_vector3.axis().transpose()[0],
       rotation_vector3.angle() * rotation_vector3.axis().transpose()[1],
       rotation_vector3.angle() * rotation_vector3.axis().transpose()[2]);
  cv::Mat t_vec = (cv::Mat_<double>(3, 1) << extrinsic_params[3],
                   extrinsic_params[4], extrinsic_params[5]);
  // project 3d-points into image view
  std::vector<cv::Point2f> pts_2d;
  cv::projectPoints(pts_3d, r_vec, t_vec, camera_matrix, distortion_coeff,
                    pts_2d);
  cv::Mat image_project = cv::Mat::zeros(height_, width_, CV_16UC1);
  cv::Mat rgb_image_project = cv::Mat::zeros(height_, width_, CV_8UC3);
  for (size_t i = 0; i < pts_2d.size(); ++i) {
    cv::Point2f point_2d = pts_2d[i];
    if (point_2d.x <= 0 || point_2d.x >= width_ || point_2d.y <= 0 ||
        point_2d.y >= height_) {
      continue;
    } else {
      // test depth and intensity both
  //    if (projection_type == DEPTH) {
  //      float depth = sqrt(pow(pts_3d[i].x, 2) + pow(pts_3d[i].y, 2) +
  //                         pow(pts_3d[i].z, 2));
  //      float intensity = intensity_list[i];
  //      float depth_weight = 1;
  //      float grey = depth_weight * depth / max_depth_ * 65535 +
  //                   (1 - depth_weight) * intensity / 150 * 65535;
  //      if (image_project.at<ushort>(point_2d.y, point_2d.x) == 0) {
  //       image_project.at<ushort>(point_2d.y, point_2d.x) = grey;
  //        rgb_image_project.at<cv::Vec3b>(point_2d.y, point_2d.x)[0] =
  //            depth / max_depth_ * 255;
  //        rgb_image_project.at<cv::Vec3b>(point_2d.y, point_2d.x)[1] =
  //            intensity / 150 * 255;
  //      } else if (depth < image_project.at<ushort>(point_2d.y, point_2d.x)) {
  //        image_project.at<ushort>(point_2d.y, point_2d.x) = grey;
  //        rgb_image_project.at<cv::Vec3b>(point_2d.y, point_2d.x)[0] =
  //            depth / max_depth_ * 255;
  //        rgb_image_project.at<cv::Vec3b>(point_2d.y, point_2d.x)[1] =
  //            intensity / 150 * 255;
  //      }

  //    } else {
       float intensity = intensity_list[i];
       if (intensity > 100) {
         intensity = 65535;
       } else {
         intensity = (intensity / 150.0) * 65535;
       }
       image_project.at<ushort>(point_2d.y, point_2d.x) = intensity;
    }
  }
  cv::Mat grey_image_projection;
  cv::cvtColor(rgb_image_project, grey_image_projection, cv::COLOR_BGR2GRAY);

  image_project.convertTo(image_project, CV_8UC1, 1 / 256.0);
//  if (is_fill_img) {
//    for (int i = 0; i < 5; i++) {
//      image_project = fillImg(image_project, UP, LEFT);
//    }
//  }
//  if (is_fill_img) {
//    for (int i = 0; i < 5; i++) {
//      grey_image_projection = fillImg(grey_image_projection, UP, LEFT);
//    }
//  }
  projection_img = image_project.clone();
}


int main(){

	cv::Mat image_ = cv::imread("/home/ubuntu/.ros/frame0000.jpg", cv::IMREAD_UNCHANGED);
	pcl::PointCloud<pcl::PointXYZI>::Ptr raw_lidar_cloud_;
	raw_lidar_cloud_ =  pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::io::loadPCDFile("/home/ubuntu/pcdfiles/1.pcd",*raw_lidar_cloud_);
	Eigen::Matrix3d rotation_matrix_;
	rotation_matrix_ <<  0.00863318, -0.999762, 0.0200449, -0.00260174, -0.0200681, -0.999795, 0.999959, 0.00857926, -0.00277437;
	Eigen::Vector3d translation_vector_;
	translation_vector_ << -0.0116852, 0.0668396, -0.187995;
	float fx_, fy_, cx_, cy_, k1_, k2_, p1_, p2_, k3_;
/*	fx_ = 3448.433002409948;
	cx_ = 1314.074819867915;
	fy_ = 3442.646620187229;
	cy_ = 1198.050887261081;
	k1_ = -0.1275485768317373;
	k2_ = 0.6008831116825363;
	p1_ = 0.006951366701960344;
	p2_ = -0.01028018911975231;
	k3_ = -0.2600939436926761;*/
	fx_ = 3366.716078713289;
	cx_ = 1513.975349430124;
	fy_ = 3365.209257566892;
	cy_ = 1048.847262742901;
	k1_ = -0.1052957347643846;
	k2_ = -0.2735498184181512;
	p1_ = -0.001022950615333698;
	p2_ = -0.00006181656029164018;
	k3_ = 3.353226222768285;
	Eigen::Vector3d euler_angle = rotation_matrix_.eulerAngles(2,1,0);
	Vector6d calib_params;
	calib_params << euler_angle(0), euler_angle(1), euler_angle(2), translation_vector_(0), translation_vector_(1), translation_vector_(2);

	cv::Mat depth_projection_img;
	projection(calib_params, raw_lidar_cloud_, fx_, cx_, fy_, cy_, k1_, k2_, p1_, p2_, k3_, depth_projection_img);
	cv::Mat map_img = cv::Mat::zeros(height_, width_, CV_8UC3);
	for(int x = 0; x < map_img.cols; x++) {
		for(int y = 0; y < map_img.rows; y++) {
			uint8_t r,g,b;
			float norm = depth_projection_img.at<uchar>(y,x) / 256.0;
			mapJet(norm, 0, 1, r, g, b);
			map_img.at<cv::Vec3b>(y,x)[0] = b;
			map_img.at<cv::Vec3b>(y,x)[1] = g;
			map_img.at<cv::Vec3b>(y,x)[2] = r;
		}
       	}
	 cv::Mat merge_img;
 	 if (image_.type() == CV_8UC3) {
   		 merge_img = 0.5 * map_img + 0.8 * image_;
 	 } else {
    		cv::Mat src_rgb;
   		cv::cvtColor(image_, src_rgb, cv::COLOR_GRAY2BGR);
   		merge_img = 0.5 * map_img + 0.8 * src_rgb;
 	 }
  	cv::imwrite("/home/ubuntu/test2.png",merge_img);
	return 0;
}
