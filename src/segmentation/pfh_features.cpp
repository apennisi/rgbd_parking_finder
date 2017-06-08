#include "pfh_features.h"

using namespace segmentation;

PFHFeatures::PFHFeatures() : BayFeatures()
{
}

std::vector< float > PFHFeatures::featureExtraction(const pcl::PointCloud< pcl::PointXYZRGBNormal >::Ptr& _bay)
{
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr filtered = filtering(_bay);
  pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
  // PFH estimation object.
  pcl::PFHEstimation<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal, PFH125> pfh;
  pfh.setInputCloud(filtered);
  pfh.setInputNormals(filtered);
  pfh.setSearchMethod(kdtree);
  pfh.setRadiusSearch(0.20);

  pcl::PointCloud<PFH125>::Ptr descriptors(new pcl::PointCloud<PFH125>());
  pfh.compute(*descriptors);
std::cout << descriptors->points.size() << std::endl;
  std::vector<float> features;
  cv::Mat featureUnclustered(cv::Size(num_descriptors, descriptors->points.size()), CV_32FC1);
  uint j = 0;
  for(const auto& rift : descriptors->points)
  {
    const PFH125& descriptor = rift;

    for(uint i = 0; i < num_descriptors; ++i) 
    {
      featureUnclustered.at<float>(j, i) = descriptor.histogram[i];
    }
    ++j;
  }

  
  
  //define Term Criteria
  cv::TermCriteria tc(CV_TERMCRIT_ITER,100,0.001);
  //retries number
  int retries=1;
  //necessary flags
  int flags=cv::KMEANS_PP_CENTERS;
  //Create the BoW (or BoF) trainer
  cv::BOWKMeansTrainer bowTrainer(dictionarySize,tc,retries,flags);
  //cluster the feature vectors
  cv::Mat dictionary=bowTrainer.cluster(featureUnclustered);
  for(uint i = 0; i < dictionary.rows; ++i)
  {
    for(uint j = 0; j < dictionary.cols; ++j)
    {
      features.push_back(dictionary.at<float>(i, j));
    }
  }
  return features;
  
  
}
