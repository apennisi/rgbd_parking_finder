/**
 * \file esf_features.h
 *
 * \class ESFFeatures
 *
 * \brief Class that extract Ensemble of Shape Functions features
 *
 **/

#ifndef _ESF_FEATURES_H_
#define _ESF_FEATURES_H_

#include <pcl/features/esf.h>
#include "bayfeatures.h"

namespace segmentation
{
  class ESFFeatures : public BayFeatures
  {
    public:
      /**
       * \brief Create a new ESFFeatures object
       */
      ESFFeatures();
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