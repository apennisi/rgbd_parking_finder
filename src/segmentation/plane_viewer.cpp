#include "plane_viewer.h"

using namespace segmentation;

PlanViewer::PlanViewer()
{
  homography = std::shared_ptr<utils::Homography>(new utils::Homography);
}


void PlanViewer::planeDetection(const pcl::PointCloud< pcl::PointXYZRGB >::Ptr _cloud)
{
  std::cout << "\t - Cloud Filtering" << std::endl;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_plane(new pcl::PointCloud<pcl::PointXYZRGB>);
  m_plane = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
  
  //Reducing the cloud by considering only the points under m_plane_threshold = 180cm
  for(uint i = 0; i < _cloud->points.size(); ++i)
  {
    const pcl::PointXYZRGB& point = _cloud->points.at(i);
    if(point.z < m_plane_threshold)
    {
      temp_plane->points.push_back(point);
    }
  }
  
  //Reducing the 3D points to voxels in order to increase the process
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered = filtering(temp_plane);
  std::cout << "\t - Plane Projetion" << std::endl;
  // Create a set of planar coefficients with X=Y=0,Z=1
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  coefficients->values.resize (4);
  coefficients->values[0] = coefficients->values[1] = 0;
  coefficients->values[2] = 1.0;
  coefficients->values[3] = 0;
  
  // Create the filtering object
  pcl::ProjectInliers<pcl::PointXYZRGB> proj;
  proj.setModelType (pcl::SACMODEL_PLANE);
  proj.setInputCloud (filtered);
  proj.setModelCoefficients (coefficients);
  proj.filter (*m_plane);
  
}

pcl::PointCloud< pcl::PointXYZRGB >::Ptr PlanViewer::filtering(const pcl::PointCloud< pcl::PointXYZRGB >::Ptr _cloud) const
{
  //Creating a voxel grid in order to reduce the number of points
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>());

  // Create the filtering object
  pcl::VoxelGrid<pcl::PointXYZRGB> sor;
  sor.setInputCloud (_cloud);
  sor.setLeafSize (0.05f, 0.05f, 0.05f);
  sor.filter (*cloud_filtered);
  
  return cloud_filtered;
}

std::vector< cv::Point2f > PlanViewer::minMaxPoint3D(const pcl::PointCloud< pcl::PointXYZRGB >::Ptr _cloud) 
{
  pcl::PointXYZRGB minP, maxP;
  pcl::getMinMax3D(*_cloud, minP, maxP);
  int w = maxP.x - minP.x;
  int h = maxP.y - minP.y;
  
  std::vector<cv::Point2f> cloudPoints;
  cloudPoints.push_back(cv::Point2f(minP.x, minP.y));
  cloudPoints.push_back(cv::Point2f(maxP.x, minP.y));
  cloudPoints.push_back(cv::Point2f(maxP.x / 2, maxP.y / 2));
  cloudPoints.push_back(cv::Point2f(maxP.x, maxP.y));
  cloudPoints.push_back(cv::Point2f(minP.x, maxP.y));
  
  plane_width = maxP.x - minP.x;
  plane_height = maxP.y - minP.y;
  
  return cloudPoints;
}

void PlanViewer::getRotation(const cv::Mat& _plane_view)
{
  cv::Mat gray;
  cv::cvtColor(_plane_view, gray, CV_BGR2GRAY);
  
  //Thresholding
  cv::Mat t = gray > 0;
  
  //Morphological Operations
  cv::dilate(t, t, cv::Mat(), cv::Point(-1, -1), 2);
  
  //Contours computation
  std::vector<std::vector<cv::Point> > contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours( t, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0)  );
  
  //Finding the countour with the maximum area
  double area = -FLT_MAX;
  uint i = 0, j = -1;
  for(const auto& contour : contours)
  {
    const double& temp_area = cv::contourArea(contour);
    if(temp_area > area)
    {
      area = temp_area;
      j = i;
    }
    ++i;
  }
  
  //Finding the corresponding rotated rectangle with respect to the maximum contour
  cv::RotatedRect minRect = minAreaRect(cv::Mat(contours[j]));
 
  //Finding the rotation matrix
  if(minRect.angle != 0)
  {
    const double& angle = (minRect.angle < 0) ? minRect.angle : -minRect.angle;
    m_rotation = cv::getRotationMatrix2D(minRect.center, angle, 1);
    m_invRotation = cv::getRotationMatrix2D(minRect.center, -angle, 1);
  }
  
}


void PlanViewer::process(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloud)
{
  std::cout << "Plane Detection: " << std::endl;
  planeDetection(_cloud); 
  const std::vector<cv::Point2f>& planePoints = minMaxPoint3D(m_plane);
  const cv::Size& rectSize = cv::Size(cvRound(plane_width * 20), cvRound(plane_height * 20));
  m_plan_view = cv::Mat::zeros(rectSize, CV_8UC3);
  
  //Image Points
  std::vector<cv::Point2f> imgPoints;
  imgPoints.push_back(cv::Point2f(0, 0));
  imgPoints.push_back(cv::Point2f(rectSize.width, 0));
  imgPoints.push_back(cv::Point2f(rectSize.width / 2, rectSize.height / 2));
  imgPoints.push_back(cv::Point2f(rectSize.width, rectSize.height));
  imgPoints.push_back(cv::Point2f(0, rectSize.height));
  
  //Compute Homography
  homography->addPoints(planePoints, imgPoints);
  homography->calcHomography();
  homography->convertPCL2Image(m_plane, m_plan_view);
  
  //Compute the rotation
  getRotation(m_plan_view);
  
  //If the view is rotated: apply the rotation
  if(!m_rotation.empty())
    cv::warpAffine(m_plan_view, m_plan_view, m_rotation, m_plan_view.size());
}
