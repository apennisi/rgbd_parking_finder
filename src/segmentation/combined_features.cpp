#include "combined_features.h"

using namespace segmentation;

CombinedFeatures::CombinedFeatures(const std::vector< std::string >& combined)
{
  if(combined.size() != 2)
  {
    std::cerr << "Features must be of two types!" << std::endl;
    exit(-1);
  }
  features_1 = features_2 = nullptr;
  esf = other = false;
  
  for(const auto& feature : combined)
  {
    if(feature.compare("ESF") == 0 && !esf)
    {
      if(!features_1)
      {
	features_1 = std::shared_ptr<ESFFeatures>(new ESFFeatures);
	esf = true;
      }
    }
    else if(feature.compare("FPFH") == 0 && !other)
    {
      if(!features_2)
      {
	features_2 = std::shared_ptr<FPFHFeatures>(new FPFHFeatures);
	other = true;
      }
    }
    else if(feature.compare("PFH") == 0 && !other)
    {
      if(!features_2)
      {
	features_2 = std::shared_ptr<PFHFeatures>(new PFHFeatures);
	other = true;
      }
    }
    else
    {
      std::cerr << "FEATURE NOT IMPLEMENTED YET!" << std::endl;
      exit(-1);
    }
  }
  
}

std::vector< float > CombinedFeatures::featureExtraction(const pcl::PointCloud< pcl::PointXYZRGBNormal >::Ptr& _bay)
{
  std::vector<float> featureVec_1 =  features_1->featureExtraction(_bay);
  const std::vector<float>& featureVec_2 =  features_2->featureExtraction(_bay);
  
  featureVec_1.insert(featureVec_1.end(), featureVec_2.begin(), featureVec_2.end());
  
  return featureVec_1;
}
