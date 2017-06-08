/**
 * \file vfh_features.h
 *
 * \class VFHFeatures
 *
 * \brief Class that extract View Point Feature histograms features
 *
 **/


#ifndef _VFH_FEATURES_H_
#define _VFH_FEATURES_H_

#include "bayfeatures.h"
#include <pcl/features/vfh.h>

namespace segmentation
{
  class VFHFeatures : public BayFeatures
  {
    public:
      /**
       * \brief Create a new VFHFeatures object
       */
      VFHFeatures();
      /**
       * \brief Extract the features given a point cloud
       * \param _bay pcl::PointCloud that contains the points related to a single bay
       * \return a std::vector of float containing a set of features
       */
      std::vector<float> featureExtraction(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& _bay);
    private:
  };
}

#endif