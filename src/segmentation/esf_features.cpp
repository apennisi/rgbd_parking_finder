#include "esf_features.h"

using namespace segmentation;

ESFFeatures::ESFFeatures() : BayFeatures()
{

}

std::vector< float > ESFFeatures::featureExtraction(const pcl::PointCloud< pcl::PointXYZRGBNormal >::Ptr& _bay)
{
  pcl::PointCloud<pcl::ESFSignature640>::Ptr descriptor(new pcl::PointCloud<pcl::ESFSignature640>);

  // ESF estimation object.
  pcl::ESFEstimation<pcl::PointXYZRGBNormal, pcl::ESFSignature640> esf;
  esf.setInputCloud(_bay);

  esf.compute(*descriptor);
  
  std::vector<float> features;
  for(const auto& feature : descriptor->points)
  {
    for(const auto& hist : feature.histogram)
      features.push_back(hist);
  }
  return features;
}
