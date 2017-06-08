/**
 * \file combined_features.h
 *
 * \class CombinedFeatures
 *
 * \brief Class that combines two types of features
 *
 **/

#ifndef _COMBINED_FEATURES_H_
#define _COMBINED_FEATURES_H_

#include "bayfeatures.h"
#include "esf_features.h"
#include "fpfh_features.h"
#include "pfh_features.h"


namespace segmentation
{
  class CombinedFeatures : public BayFeatures
  {
    public:
      /**
       * \brief Create a new CombinedFeatures object
       * \param combined std::vector of string containing the names of the features
       */
      CombinedFeatures(const std::vector<std::string>& combined);
    private:
      std::shared_ptr<BayFeatures> features_1, features_2;
      bool esf, other;
    private:
      /**
       * \brief Extract the features given a point cloud
       * \param _bay pcl::PointCloud that contains the points related to a single bay
       * \return a std::vector of float containing a set of features
       */
      std::vector<float> featureExtraction(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& _bay);
  };
}

#endif