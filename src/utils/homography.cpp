#include "homography.h"

using namespace utils;

Homography::Homography(const std::vector<cv::Point2f>& _srcPoints, const std::vector<cv::Point2f>& _dstPoints) 
 : m_srcPoints(_srcPoints), m_dstPoints(_dstPoints) {}

void Homography::addPoints(const std::vector< cv::Point2f >& _srcPoints, const std::vector< cv::Point2f >& _dstPoints)
{
  m_srcPoints = _srcPoints;
  m_dstPoints = _dstPoints;
}


void Homography::calcHomography() 
{
  H = cv::findHomography(
  cv::Mat(m_srcPoints), cv::Mat(m_dstPoints),  // corresponding points
    //inliers,	// outputed inliers matches
    CV_RANSAC, //force all corrispondences to be used
    1e-1); // max distance to reprojection point
  std::cout << H << std::endl;
  H_inv = H.inv();
}

void Homography::readFromFile(const std::string& _filename)
{
  if(_filename.find(".txt") == std::string::npos)
  {
    std::cerr << "Error: " << _filename << " does not exist!" << std::endl;
    exit(-1);
  }
  
  std::ifstream file;
  file.open(_filename);
  if(!file.is_open())
  {
    std::cerr << "File " << _filename << " does not exist!" << std::endl;
    file.close();
    exit(-1);
  }
  
  if(H.empty())
  {
    H = cv::Mat::zeros(3, 3, CV_64FC1);
  }
  
  if(R_inv.empty())
  {
    R_inv = cv::Mat::zeros(cv::Size(3, 2), CV_64FC1);
  }
  
  std::string line;
  while(std::getline(file, line))
  {
    if(line.find("[HOMOGRAPHY]") != std::string::npos)
    {
	double v1, v2, v3;
	uint i = 0;
	while(i < 3 && std::getline(file, line))
	{
	  std::istringstream iss(line);
	  if (!(iss >> v1 >> v2 >> v3)) { break; } // error
	  H.at<double>(i, 0) = v1;
	  H.at<double>(i, 1) = v2;
	  H.at<double>(i++, 2) = v3;
	}
	H_inv = H.inv();
    }
    else if(line.find("[ROTATION]") != std::string::npos)
    {
	double v1, v2, v3;
	uint i = 0;
	while(i < 2 && std::getline(file, line))
	{
	  std::cout << line << std::endl;
	  std::istringstream iss(line);
	  if (!(iss >> v1 >> v2 >> v3)) { break; } // error
	  R_inv.at<double>(i, 0) = v1;
	  R_inv.at<double>(i, 1) = v2;
	  R_inv.at<double>(i++, 2) = v3;
	}
    }
    /*else
    {
      std::cout << "Error: no transformation matrix found" << std::endl;
      exit(-1);
    }*/
  }
  file.close();
}

std::vector< cv::Point2f > Homography::rotatedPoints(const std::vector< cv::Point2f >& _input)
{
  std::vector<cv::Point2f> rotated;
  cv::Point2f result;
  for(const auto& point : _input)
  {
    result.x = R_inv.at<double>(0,0)*point.x + R_inv.at<double>(0,1)*point.y + R_inv.at<double>(0,2);
    result.y = R_inv.at<double>(1,0)*point.x + R_inv.at<double>(1,1)*point.y + R_inv.at<double>(1,2);
    rotated.push_back(result);
  }
  return rotated;
}


void Homography::convertImage2PCL(const std::vector<cv::Point2f>& _points, std::vector<pcl::PointXYZ>& _converted)
{
  double x, y;
  for(const auto& point : _points)
  {
    calcInvProjection(point, x, y);
    _converted.push_back(pcl::PointXYZ(x, y, 0.0));
  }
}

void Homography::convertPCL2Image(const pcl::PointCloud< pcl::PointXYZRGB >::Ptr _cloud, cv::Mat& _img)
{
  double x, y;
  for(const auto& point : _cloud->points)
  {
    calcProjection(cv::Point2f(point.x, point.y), x, y);
    x = cvRound(x);
    y = cvRound(y);
    if(x > 0 && y > 0 && x < _img.cols && y < _img.rows)
    { 
      _img.at<cv::Vec3b>(cv::Point(x, y)) = cv::Vec3b(point.b, point.g, point.r);
    }
  }
}


void Homography::calcProjection(const cv::Point2f& point, double& x, double& y) 
{
    x =	(point.x*H.at<double>(0,0)+point.y*H.at<double>(0,1)+H.at<double>(0,2))/
	  (point.x*H.at<double>(2,0)+point.y*H.at<double>(2,1)+H.at<double>(2,2));
    y =	(point.x*H.at<double>(1,0)+point.y*H.at<double>(1,1)+H.at<double>(1,2))/
	  (point.x*H.at<double>(2,0)+point.y*H.at<double>(2,1)+H.at<double>(2,2));
}

void Homography::calcInvProjection(const cv::Point2f &point, double& x, double& y) 
{
    x =	(point.x*H_inv.at<double>(0,0)+point.y*H_inv.at<double>(0,1)+H_inv.at<double>(0,2)) / 
	  (point.x*H_inv.at<double>(2,0)+point.y*H_inv.at<double>(2,1)+H_inv.at<double>(2,2));
    y =	(point.x*H_inv.at<double>(1,0)+point.y*H_inv.at<double>(1,1)+H_inv.at<double>(1,2)) /
	  (point.x*H_inv.at<double>(2,0)+point.y*H_inv.at<double>(2,1)+H_inv.at<double>(2,2));
}




