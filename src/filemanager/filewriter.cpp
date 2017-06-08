#include "filewriter.h"

using namespace utils;


std::shared_ptr<FileWriter> FileWriter::m_instance = nullptr;

std::shared_ptr< FileWriter > FileWriter::instance()
{
  if(!m_instance)
  {
    m_instance = std::shared_ptr<FileWriter>(new FileWriter);
  }
  
  return m_instance;
}

void FileWriter::write(const std::string& _name, const std::vector< Rectangle >& _planimetry)
{
  if(_planimetry.size() == 0) 
  {
    std::cerr << "No planimetry saved" << std::endl;
    return;
  }
  if(_name.find(".txt") == std::string::npos)
  {
    std::cerr << "Error: " << _name << " does not exist!" << std::endl;
    exit(-1);
  }
  
  std::ofstream file;
  file.open(_name);
  
  if(!file.is_open())
  {
    std::cerr << "File " << _name << " does not exist!" << std::endl;
    file.close();
    exit(-1);
  }
  
  for(auto rect : _planimetry)
  {
    file << rect.rectToString();
  }
  
  file.close();
}

void FileWriter::write(const std::string& _name, const cv::Mat& _homography, const cv::Mat& _rotation)
{
  if(_name.find(".txt") == std::string::npos)
  {
    std::cerr << "Error: " << _name << " does not exist!" << std::endl;
    exit(-1);
  }
  
  std::ofstream file;
  file.open(_name);
  
  if(!file.is_open())
  {
    std::cerr << "File " << _name << " does not exist!" << std::endl;
    file.close();
    exit(-1);
  }
  
  file << "[HOMOGRAPHY]\n";
  for(uint i = 0; i < _homography.rows; ++i)
  {
    for(uint j = 0; j < _homography.cols; ++j)
    {
      file << std::fixed << std::setprecision(15) << _homography.at<double>(i, j) << " ";
    }
    file << "\n";
  }
  
  if(!_rotation.empty())
  {
    file << "[ROTATION]\n";
    for(uint i = 0; i < _rotation.rows; ++i)
    {
      for(uint j = 0; j < _rotation.cols; ++j)
      {
	file << std::fixed << std::setprecision(15) << _rotation.at<double>(i, j) << " ";
      }
      file << "\n";
    }
  }
  
  file.close();
}


