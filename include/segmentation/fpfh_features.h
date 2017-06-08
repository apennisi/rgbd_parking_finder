/**
 * \file fpfh_features.h
 *
 * \class FPFHFeatures
 *
 * \brief Class that extract Fast Point Feature Histograms features
 *
 **/


#ifndef _FPFH_FEATURES_H_
#define _FPFH_FEATURES_H_

#include <pcl/features/fpfh.h>
#include "bayfeatures.h"


namespace segmentation
{
  class FPFHFeatures : public BayFeatures
  {
    public:
      /**
       * \brief Create a new FPFHFeatures object
       */
      FPFHFeatures();
      /**
       * \brief Extract the features given a point cloud
       * \param _bay pcl::PointCloud that contains the points related to a single bay
       * \return a std::vector of float containing a set of features
       */
      std::vector<float> featureExtraction(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& _bay);
    private:
      typedef pcl::FPFHSignature33 FPFH33;
      constexpr static uint num_descriptors = 33;
      constexpr static int dictionarySize=100;
  };
}

#endif