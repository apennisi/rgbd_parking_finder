/**
 * \file filewriter.h
 *
 * \class FileWriter
 *
 * \brief Class for writing the planimetry and the matrices for rotation and homography
 *
 **/

#ifndef _PLY_FILE_WRITER_H_
#define _PLY_FILE_WRITER_H_

#include <iostream>
#include <fstream>
#include <memory>
#include <iomanip>

#include "rectangle.h"

namespace utils
{
  class FileWriter
  {
    public:
      /**
       * \brief create a new instance of the object FileWriter
       * \return a shared_ptr of FileWriter type
       */
      static std::shared_ptr<FileWriter> instance();
      /**
       * \brief write the planimetry to a file
       * \param _name std::string containing the file name
       * \param _planimetry std::vector of utils::Rectangle containing the planimetry
       */
      void write(const std::string& _name, const std::vector<utils::Rectangle>& _planimetry);
      /**
       * \brief write the rotation and the homography matrix to a file
       * \param _name std::string containing the file name
       * \param _homography cv::Mat containing the homography matrix
       * \param _rotation cv::Mat containing the rotation matrix
       */
      void write(const std::string& _name, const cv::Mat& _homography, const cv::Mat& _rotation);
    private:
      static std::shared_ptr<FileWriter> m_instance;
    private:
      /**
       * \brief Create a new FileWriter object
       */
      FileWriter() { ; }
      
  };
}

#endif