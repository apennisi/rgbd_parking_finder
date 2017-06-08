/**
 * \file featurefilemanager.h
 *
 * \class FeatureReaderWriter
 *
 * \brief Class for reading and writing the _features
 *
 **/

#ifndef _FEATURE_FILE_MANAGER_H_
#define _FEATURE_FILE_MANAGER_H_

#include <iostream>
#include <memory>
#include <fstream>
#include <iomanip>
#include <vector>
#include <sstream>

namespace utils
{
  class FeatureReaderWriter
  {
    public:
      /**
       * \brief create a new instance of the object FeatureReaderWriter
       * \return a shared_ptr of FeatureReaderWriter type
       */
      static std::shared_ptr<FeatureReaderWriter> instance();
      /**
       * \brief write the features into a file
       * \param _features std::vector containing the features and the labels
       * \param _name std::string containing the name of the file to write
       */
      void write(const std::vector<std::pair<float, std::vector<float> > >& _features, const std::string& _name);
      /**
       * \brief read the features from a file
       * \param _name std::string with the path to the file containing the features and the labels
       * \param _features  an empty std::vector where the features will be inserted
       */ 
      void read(const std::string& _name, std::vector<std::pair<float, std::vector<float> > >& _features);
    private:
      static std::shared_ptr<FeatureReaderWriter> m_instance;
    private:
      /**
       * \brief split a string in a vector of strings
       * \param s std::string to split 
       * \param delim char that contains the delimiter
       * \param elems std::vector of std::string that will contains the splitted string
       */
      void split(const std::string &s, char delim, std::vector<std::string> &elems);
      /**
       * \brief split a string in a vector of strings
       * \param s std::string to split
       * \param delim char that contains the delimiter
       * \return a vector of std::string
       */
      std::vector<std::string> split(const std::string &s, char delim);
    private:
      /**
       * \brief Create a new FeatureReaderWriter object
       */
      FeatureReaderWriter() { ; }
  };
}

#endif