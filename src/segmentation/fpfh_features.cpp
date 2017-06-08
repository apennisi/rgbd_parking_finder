#include "fpfh_features.h"

using namespace segmentation;

FPFHFeatures::FPFHFeatures() : BayFeatures()
{
}

std::vector< float > FPFHFeatures::featureExtraction(const pcl::PointCloud< pcl::PointXYZRGBNormal >::Ptr& _bay)
{
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr filtered = filtering(_bay);
  pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
  // FPFH estimation object.
  pcl::FPFHEstimation<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal, pcl::FPFHSignature33> fpfh;
  fpfh.setInputCloud(filtered);
  fpfh.setInputNormals(filtered);
  fpfh.setSearchMethod(kdtree);

  fpfh.setRadiusSearch(0.20);
  pcl::PointCloud<FPFH33>::Ptr descriptors(new pcl::PointCloud<FPFH33>());
  fpfh.compute(*descriptors);

  std::vector<float> features;
  cv::Mat featureUnclustered(cv::Size(num_descriptors, descriptors->points.size()), CV_32FC1);
  uint j = 0;
  for(const auto& rift : descriptors->points)
  {
    const FPFH33& descriptor = rift;

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
