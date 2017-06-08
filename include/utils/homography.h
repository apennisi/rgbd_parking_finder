/**
 * \file homography.h
 *
 * \class Homography
 *
 * \brief Class that implements the homography
 *
 **/

#ifndef HOMOGRAPHY_HPP_
#define HOMOGRAPHY_HPP_

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <pcl/point_types.h>
#include <pcl/pcl_base.h>

#include <vector>
#include <string>
#include <iostream>
#include <fstream>

namespace utils
{
  class Homography {
    private:
      std::vector<cv::Point2f> m_srcPoints;
      std::vector<cv::Point2f> m_dstPoints;
      cv::Mat H, H_inv;
      cv::Mat R, R_inv;
    public:
      /**
       * \brief Create a new Homography object
       */
      Homography() {;}
      /**
       * \brief Create a new Homography object
       * \param _srcPoints std::vector of cv::Point2f containing the source points
       * \param _dstPoints std::vector of cv::Point2f containing the destination points
       */
      Homography(const std::vector<cv::Point2f>& _srcPoints, const std::vector<cv::Point2f>& _dstPoints);
      /**
       * \brief Add the points for computing the homography
       * \param _srcPoints std::vector of cv::Point2f containing the source points
       * \param _dstPoints std::vector of cv::Point2f containing the destination points
       */
      void addPoints(const std::vector<cv::Point2f>& _srcPoints, const std::vector<cv::Point2f>& _dstPoints);
      /**
       * \brief Compute the homography
       */
      void calcHomography();
      /**
       * \brief Convert a 3D point cloud into an image
       * \param _cloud pcl::PointCloud containing the 3D points
       * \param _img cv::Mat where the 2D points will be stored
       */
      void convertPCL2Image(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloud, cv::Mat& _img);
      /**
       * \brief Convert Image points to 3D Points
       * \param _points std::vector of cv::Point2f containing the image points
       * \param _converted std::vector of pcl::PointXYZ where the 3D points will be stored
       */
      void convertImage2PCL(const std::vector<cv::Point2f>& _points, std::vector<pcl::PointXYZ>& _converted);
      /**
       * \brief Return the homography matrix
       * \return a cv::Mat containing the homography matrix
       */
      const inline cv::Mat getHomography() const { return H; }
      /**
       * \brief Return the rotation matrix
       * \return a cv::Mat containing the rotation matrix
       */
      const inline cv::Mat getRotation() const { return R; }
      /**
       * \brief Return the inverse rotation matrix
       * \return a cv::Mat containing the inverse rotation matrix
       */
      const inline cv::Mat getRotationInv() const { return R_inv; }
      /**
       * \brief Return a set of rotated points
       * \param _input std::vector of cv::Point2f that will be rotated
       * \return a std::vector of cv::Point2f containing the rotated points
       */
      std::vector<cv::Point2f> rotatedPoints(const std::vector<cv::Point2f>& _input);
      /**
       * \brief Read the rotation and homography matrices from a file
       * \param _filename std::string containing the path to the file
       */
      void readFromFile(const std::string& _filename);
    private:
      /**
       * \brief Compute the projection of a point given the homography matrix
       * \param point cv::Point2f the point to be converted
       * \param x double the x coordinate of the converted point
       * \param y double the y coordinate of the converted point
       */
      void calcProjection(const cv::Point2f& point, double& x, double& y);
      /**
       * \brief Compute the projection of a point given the inverse homography matrix
       * \param point cv::Point2f the point to be converted
       * \param x double the x coordinate of the converted point
       * \param y double the y coordinate of the converted point
       */
      void calcInvProjection(const cv::Point2f& point, double& x, double& y);
  };
}

#endif