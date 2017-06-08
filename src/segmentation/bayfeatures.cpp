#include "bayfeatures.h"

using namespace segmentation;

bool BayFeatures::isInRect(const std::vector< pcl::PointXYZ >& _box, const pcl::PointXYZRGBNormal& _point)
{
  Eigen::Vector2f p1(_box[0].x, _box[0].y);
  Eigen::Vector2f p2(_box[1].x, _box[1].y);
  Eigen::Vector2f p3(_box[2].x, _box[2].y);
  Eigen::Vector2f p4(_box[3].x, _box[3].y);
  Eigen::Vector2f p(_point.x, _point.y);
 
  Eigen::Vector2f p1_p4 = p1 - p4;
  Eigen::Vector2f p3_p4 = p3 - p4;
  Eigen::Vector2f two_p_c = 2.0*p - p1 - p3; //TWO_P_C=2P-C, C=Center of rectangle
 
  return (p3_p4.dot(two_p_c - p3_p4) <= 0 && p3_p4.dot(two_p_c + p3_p4) >= 0) &&
         (p1_p4.dot(two_p_c - p1_p4) <= 0 && p1_p4.dot(two_p_c + p1_p4) >= 0);
}


pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr BayFeatures::cropBox(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr _cloud, const std::vector<pcl::PointXYZ>& box)
{
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    
    
    for(uint i = 0; i < _cloud->points.size(); ++i)
    {
      pcl::PointXYZRGBNormal p = _cloud->points.at(i);
      if(isInRect(box, p) && p.z <= maxHeight)
      {
	output->points.push_back(p);
      }
    }
    
    return output;    
}

std::vector<std::pair<float, std::vector<float> > > BayFeatures::process(const pcl::PointCloud< pcl::PointXYZRGBNormal >::Ptr _cloud, std::vector< utils::Rectangle >& _rectangles, 
		       std::shared_ptr<utils::Homography>& _homography)
{
  std::cout << "FEATURE COMPUTATION" << std::endl;

  for(auto& rect : _rectangles)
  {
    const std::vector<cv::Point2f>& vertices = rect.getPoints();
    const std::vector<cv::Point2f>& rotatedVertices = _homography->rotatedPoints(vertices);
    
    std::vector<pcl::PointXYZ> points3D;
    _homography->convertImage2PCL(rotatedVertices, points3D);
    
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr bay;
    bay = cropBox(_cloud, points3D);
    
    m_features.push_back(std::make_pair<float, std::vector<float> >(rect.getLabel(), featureExtraction(bay)));
    

  }
  
  std::cout << "END FEATURE COMPUTATION" << std::endl;
  
  return m_features;
}

pcl::PointCloud< pcl::PointXYZRGBNormal >::Ptr BayFeatures::filtering(const pcl::PointCloud< pcl::PointXYZRGBNormal >::Ptr _cloud) const
{
  //Creating a voxel grid in order to reduce the number of points
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGBNormal>());

  // Create the filtering object
  pcl::VoxelGrid<pcl::PointXYZRGBNormal> sor;
  sor.setInputCloud (_cloud);
  sor.setLeafSize (0.05f, 0.05f, 0.05f);
  sor.filter (*cloud_filtered);
  
  return cloud_filtered;
}

void BayFeatures::write(const std::string& _filename)
{
  utils::FeatureReaderWriter::instance()->write(m_features, _filename);
}

std::vector<std::pair<float, std::vector<float> > > BayFeatures::read(const std::string& _filename)
{
  utils::FeatureReaderWriter::instance()->read(_filename, m_features);
  return m_features;
}
