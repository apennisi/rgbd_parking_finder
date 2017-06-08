/**
 * \file bayfeatures.h
 *
 * \class BayFeatures
 *
 * \brief Abstract class that implements a feature extractor
 *
 **/

#ifndef _BAYFEATURES_H_
#define _BAYFEATURES_H_

#include <iostream>
#include <memory>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types_conversion.h>
#include <pcl/features/intensity_gradient.h>
#include <pcl/features/rift.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/pcl_base.h>
#include "rectangle.h"
#include "homography.h"
#include "featurefilemanager.h"

namespace segmentation
{
  class BayFeatures 
  {
    public:
      /**
       * \brief Create a new BayFeatures object
       */
      BayFeatures() { ; }
      /**
       * \brief Compute the features given the planimetry _rectangles
       * \param _cloud pcl::PointCloud of pcl::PointXYZRGBNormal containing the cloud
       * \param _rectangles std::vector of utils::Rectangle containing the planimetry
       * \param _homography std::shared_ptr of utils::Homography that manages the geometric transformations
       * \return a std::vector of labels and features
       */
      std::vector<std::pair<float, std::vector<float> > > process(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr _cloud, std::vector<utils::Rectangle>& _rectangles, 
		    std::shared_ptr<utils::Homography>& _homography);
      /**
       * \brief Write the features to the disk
       * \param _filename std::string containing the path to the file
       */
      void write(const std::string& _filename = "features.txt");
      /**
       * \brief Read the features from a file
       * \param _filename std::string containing the path to the file which contains the features and the labels
       * \return a std::vector of labels and features
       */
      std::vector<std::pair<float, std::vector<float> > > read(const std::string& _filename);
      /**
       * \brief Extract the features given a point cloud
       * \param _bay pcl::PointCloud that contains the points related to a single bay
       * \return a std::vector of float containing a set of features
       */
      virtual std::vector<float> featureExtraction(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& _bay) = 0;
    private:
      std::vector<std::pair<float, std::vector<float> > > m_features;
      constexpr static float maxHeight = 1.8; //1.8 meters 
    protected:
      /**
       * \brief Select a single bay from a point cloud
       * \param _cloud pcl::PointCloud containing the cloud
       * \param _box std::vector of pcl::PointXYZ containg the 3D vertices of the bay
       * \return a point cloud containing the single bay
       */
      pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cropBox(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr _cloud, const std::vector<pcl::PointXYZ>& box);
      /**
       * \brief Check if a 3D point is inside a 3D rectangle
       * \param _box std::vector of pcl::PointXYZ containing the vertices of the 3D rectangle
       * \param _point pcl::PointXYZRGBNormal containing the point to check
       * \return true if the points is inside the rectagle, false otherwise
       */
      bool isInRect(const std::vector<pcl::PointXYZ>& _box, const pcl::PointXYZRGBNormal& _point);
      /**
       * \brief Reduce the number of points by applying a fixed voxel grid of 5cm x 5cm x 5cm
       * \param _cloud pcl::PointCloud containing the cloud to reduce
       * \return a pcl::PointCloud of pcl::PointXYZRGBNormal points
       */
      pcl::PointCloud< pcl::PointXYZRGBNormal >::Ptr filtering(const pcl::PointCloud< pcl::PointXYZRGBNormal >::Ptr _cloud) const;
      
  };
}

#endif