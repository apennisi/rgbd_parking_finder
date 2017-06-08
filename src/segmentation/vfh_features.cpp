#include "vfh_features.h"

using namespace segmentation;

VFHFeatures::VFHFeatures() : BayFeatures()
{

}

std::vector< float > VFHFeatures::featureExtraction(const pcl::PointCloud< pcl::PointXYZRGBNormal >::Ptr& _bay)
{
  pcl::VFHEstimation<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal, pcl::VFHSignature308> vfh;
  vfh.setInputCloud (_bay);
  vfh.setInputNormals (_bay);
  
  pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBNormal> ());
  vfh.setSearchMethod (tree);

  // Output datasets
  pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs (new pcl::PointCloud<pcl::VFHSignature308> ());
  // Compute the features
  vfh.compute (*vfhs);
  std::vector<float> features;
  for(const auto& feature : vfhs->points)
  {
    for(const auto& hist : feature.histogram)
      features.push_back(hist);
  }
  return features;
}

