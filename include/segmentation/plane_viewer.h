/**
 * \file plane_viewer.h
 *
 * \class PlanViewer
 *
 * \brief Class that extract the plan view of the point cloud 
 *
 **/

#ifndef _PLAN_VIEWER_H_
#define _PLAN_VIEWER_H_

#include <pcl/point_types.h>
#include <pcl/pcl_base.h>
#include <pcl/surface/concave_hull.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/project_inliers.h>
#include <opencv2/opencv.hpp>

#include "homography.h"
#include <memory>
#include <iostream>

namespace segmentation
{
  class PlanViewer
  {
    public:
      /**
       * \brief Create a new PlanViewer object
       */
      PlanViewer();
      /**
       * \brief Extract the plan view from a point cloud
       * \param _cloud pcl::PointCloud that is manipulated for obtaining the plan view
       */
      void process(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloud);
      /**
       * \brief Return the plan view image
       * \return a cv::Mat containing the plan view image
       */
      const inline cv::Mat plan_view() const { return m_plan_view; } 
      /**
       * \brief Return the rotation matrix
       * \return a cv::Mat containing rotation matrix
       */
      const inline cv::Mat rotationMatrix() const { return m_rotation; }
      /**
       * \brief Return the inverse rotation matrix
       * \return a cv::Mat containing inverse rotation matrix
       */
      const inline cv::Mat invRotationMatrix() const { return m_invRotation; }
      /**
       * \brief Return the homography matrix
       * \return a cv::Mat containing homography matrix
       */
      const inline cv::Mat homographyMatrix() const { return homography->getHomography(); }
    private:
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr  m_plane;
      static constexpr float m_plane_threshold = 1.8; //180 cm
      cv::Mat m_rotation, m_invRotation;
      cv::Mat m_plan_view;
      float plane_width, plane_height;
      std::shared_ptr<utils::Homography> homography;
    private:
      /**
       * \brief Detect the plane from a point cloud
       * \param _cloud pcl::PointCloud that is manipulated for obtaining the plan view
       */
      void planeDetection(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloud);
      /**
       * \brief Reduce the point cloud according a m_plane_threshold and a voxel_grid of 5cm x 5cm x 5cm
       * \param _cloud pcl::PointCloud to filter
       */
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtering(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloud) const;
      /**
       * \brief Compute the top left and the bottom right points of the 3D rectangle that delimits the plane
       * \param _cloud pcl::PointCloud containing the plane
       */
      std::vector<cv::Point2f> minMaxPoint3D(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloud);
      /**
       * \brief Compute the rotation of the image if is need
       * \param _plane_view cv::Mat containing the image to rotate
       */
      void getRotation(const cv::Mat& _plane_view);
      
  };
}

#endif