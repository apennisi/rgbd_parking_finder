#include <iostream>
#include <pcl/point_types.h>
#include <pcl/pcl_base.h>


#include "plyfilereader.h"
#include "homography.h"
#include "planimetry_manager.h"
#include "bayfeatures.h"
#include "fpfh_features.h"
#include "pfh_features.h"
#include "vfh_features.h"
#include "combined_features.h"
#include "esf_features.h"
#include "configmanager.h"
#include "classifier.h"
#include "svm.h"
#include "adaboost.h"

//VARIABLES
std::shared_ptr<segmentation::BayFeatures> parkingFeatures = nullptr;
std::vector<std::pair<float, std::vector<float> > > features;
std::shared_ptr<ml::Classifier> classifier = nullptr;
pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud = nullptr;
utils::Options options;


void setFeatures()
{
  if(options.feature_type.size() == 2)
  {
    parkingFeatures = std::shared_ptr<segmentation::CombinedFeatures>(new segmentation::CombinedFeatures(options.feature_type));
  }
  else if(options.feature_type.size() == 1)
  {
    const std::string& feature_type = options.feature_type[0];
    if(feature_type.compare("ESF") == 0)
    {
      parkingFeatures = std::shared_ptr<segmentation::ESFFeatures>(new segmentation::ESFFeatures);
    }
    else if(feature_type.compare("FPFH") == 0)
    {
      parkingFeatures = std::shared_ptr<segmentation::FPFHFeatures>(new segmentation::FPFHFeatures);
    }
    else if(feature_type.compare("PFH") == 0)
    {
      parkingFeatures = std::shared_ptr<segmentation::PFHFeatures>(new segmentation::PFHFeatures);
    }
    else if(feature_type.compare("VFH") == 0)
    {
      parkingFeatures = std::shared_ptr<segmentation::VFHFeatures>(new segmentation::VFHFeatures);
    }
    else
    {
      std::cerr << "FEATURE NOT IMPLEMENTED YET!" << std::endl;
      exit(-1);
    }
      
  }
  else
  {
    std::cerr << "NUMBER OF FEATURES NOT IMPLEMENTED YET!" << std::endl;
    exit(-1);
  }
}

int main(int argc, char **argv)
{  
  if(argc != 2)
  {
    std::cerr << "Usage: \t" << argv[0] << " config.cfg" << std::endl;
    exit(-1);
  }
  
  try
  {
    options = utils::ConfigManager::instance()->read(std::string(argv[1]));
  }
  catch(...)
  {
    std::cerr << "Cannot read " << argv[1] << "!" << std::endl;
    exit(-1);
  }
  
  cloud = pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  utils::PlyFileReader::instance()->process<pcl::PointXYZRGBNormal>(options.cloud_file, cloud);
  std::cout << "NUMBER OF POINTS" << std::endl;
  std::cout << cloud->points.size() << std::endl;
  
  cv::Mat plane_view;
  try
  {
    plane_view = cv::imread(std::string(options.planimetry_image));
  }
  catch(...)
  {
    std::cerr << "Cannot read " << options.planimetry_image << "!" << std::endl;
    exit(-1);
  }
   
  std::shared_ptr<utils::Homography> homography(new utils::Homography);
  homography->readFromFile(std::string(options.geometry_file));
  
  
  
  std::shared_ptr<utils::PlanimetryManager> planimetryManager(new utils::PlanimetryManager);
  planimetryManager->readFromFile(std::string(options.planimetry_file));
  std::vector<utils::Rectangle> rects = planimetryManager->getRects();
  
  setFeatures();
  features = parkingFeatures->process(cloud, rects, homography);
  
  if(options.classifier.compare("adaboost") == 0)
  {
    classifier = std::shared_ptr<ml::AdaBoost>(new ml::AdaBoost(options.classifier_file));
  }
  else if(options.classifier.compare("svm") == 0)
  {
    classifier = std::shared_ptr<ml::SupportVectorMachine>(new ml::SupportVectorMachine(options.classifier_file));
  }
  else
  {
    std::cerr << "Classifier " << options.classifier << " not implemented yet!" << std::endl;
    exit(-1);
  }
  
  const std::vector<float>& predictions = classifier->predict(features);
  std::cout << "Done" << std::endl;
  planimetryManager->classificationViewer(plane_view, predictions);
  
  cv::namedWindow("Prediction Results");
  cv::imshow("Prediction Results", plane_view);
  cv::waitKey(0);
  cv::destroyAllWindows();
  cloud.reset();
  std::cout << "Done" << std::endl;
  return 0;
  
}