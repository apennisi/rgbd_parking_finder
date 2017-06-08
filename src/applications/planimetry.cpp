#include <iostream>
#include <pcl/point_types.h>
#include <pcl/pcl_base.h>

#include "plyfilereader.h"
#include "filewriter.h"
#include "plane_viewer.h"
#include "parking_selection.h"


//VARIABLES
std::shared_ptr<segmentation::PlanViewer> planeViewer = nullptr;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = nullptr;

int main(int argc, char **argv)
{  
  if(argc != 2)
  {
    std::cerr << "Usage: \t" << argv[0] << " ply_filename" << std::endl;
    exit(-1);
  }
  
  cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
  utils::PlyFileReader::instance()->process<pcl::PointXYZRGB>(std::string(argv[1]), cloud);
  std::cout << "NUMBER OF POINTS" << std::endl;
  std::cout << cloud->points.size() << std::endl;
 
  planeViewer = std::shared_ptr<segmentation::PlanViewer>(new segmentation::PlanViewer);
  planeViewer->process(cloud);
  cv::Mat plane = planeViewer->plan_view();

  std::vector<utils::Rectangle> planimetry;
  utils::ParkingSelection::instance()->process(plane, planimetry);
  
  utils::FileWriter::instance()->write("planimetry.txt", planimetry);
  utils::FileWriter::instance()->write("matrices.txt", planeViewer->homographyMatrix(), planeViewer->invRotationMatrix());
  
  cv::imwrite("plane_view.png", plane);
  cloud.reset();
  return 0; 
}
