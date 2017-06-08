/**
 * \file pfh_features.h
 *
 * \class PFHFeatures
 *
 * \brief Class that extract Point Feature Histograms features
 *
 **/

#ifndef _PFH_FEATURES_H_
#define _PFH_FEATURES_H_

#include <pcl/features/pfh.h>
#include "bayfeatures.h"

namespace segmentation
{
  class PFHFeatures : public BayFeatures
  {
    public:
      /**
       * \brief Create a new FPFHFeatures object
       */
      PFHFeatures();
      /**
       * \brief Extract the features given a point cloud
       * \param _bay pcl::PointCloud that contains the points related to a single bay
       * \return a std::vector of float containing a set of features
       */
      std::vector<float> featureExtraction(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& _bay);
    private:
      typedef pcl::PFHSignature125 PFH125;
      constexpr static uint num_descriptors = 125;
      constexpr static int dictionarySize=100;
  };
}

#endif