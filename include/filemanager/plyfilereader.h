/**
 * \file plyfilereader.h
 *
 * \class PlyFileReader
 *
 * \brief Class for reading the ply file
 *
 **/


#ifndef _FILEREADER_H_
#define _FILEREADER_H_

#include <iostream>
#include <memory>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types_conversion.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Eigen>
#include <functional>


namespace utils
{
  class PlyFileReader 
  {
    public:
      /**
       * \brief create a new instance of the object PlyFileReader
       * \return a shared_ptr of PlyFileReader type
       */
      static std::shared_ptr<PlyFileReader> instance();
      /**
       * \brief process the file containing the point cloud
       * \param _filename std::string containing the path to the point cloud
       * \param cloud pcl::PointCloud of type T that will containt the point cloud and T can be: pcl::PointXYZ, pcl::PointXYZRGB, pcl::PointXYZRGBNormal
       */
      template<typename T>
      void process(const std::string& _filename, typename pcl::PointCloud<T>::Ptr cloud)
      {
	//CHECK FILE NAME
	if(_filename.find(".ply") == std::string::npos)
	{
	  std::cerr << "Invalid filename: " << _filename << std::endl;
	  std::cerr << "The file has to be a ply file!" << std::endl;
	  exit(-1);
	}
		
	//READING THE POINT CLOUD
	if (pcl::io::loadPLYFile(_filename, *cloud) == -1) //* load the file
	{
	  PCL_ERROR ("Couldn't read PLY file\n");
	  exit (-1);
	}
	std::cout << "Point Cloud Loaded!" << std::endl;
      }
    private:
      static std::shared_ptr<PlyFileReader> m_instance;
    private:
      /**
       * \brief Create a new PlyFileReader object
       */
      PlyFileReader() {;}
  };
}

#endif //_FILEREADER_H_