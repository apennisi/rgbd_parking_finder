/**
 * \file configmanager.h
 *
 * \class ConfigManager
 *
 * \brief Class for managing the config files
 *
 **/

#ifndef _CONFIG_MANAGER_H_
#define _CONFIG_MANAGER_H_

#include <iostream>
#include <memory>
#include <fstream>
#include <algorithm>
#include <boost/algorithm/string.hpp>

namespace utils
{
  struct Options
  {
    std::string cloud_file;
    std::string planimetry_file;
    std::string geometry_file;
    std::string planimetry_image;
    std::string feature_file;
    std::string mode;
    std::string classifier;
    std::string classifier_file;
    std::vector<std::string> feature_type;
    float validation_percentage;
    Options() 
    {
      validation_percentage = -1;
    }
    Options& operator=(const Options& op)
    {
      this->cloud_file = op.cloud_file;
      this->planimetry_file = op.planimetry_file;
      this->geometry_file = op.geometry_file;
      this->planimetry_image = op.planimetry_image;
      this->feature_file = op.feature_file;
      this->mode = op.mode;
      this->classifier = op.classifier;
      this->classifier_file = op.classifier_file;
      this->feature_type = op.feature_type;
      this->validation_percentage = op.validation_percentage;
      return *this;
    }
  };
  
  class ConfigManager
  {
    public:
      /**
       * \brief create a single instance of the ConfigManager object
       * \return a std::shared_ptr of ConfigManager type
       */
      static std::shared_ptr<ConfigManager> instance();
      /**
       * \brief read the options from a confin file
       * \param _filename std::string containing the path to the config file
       * \return a struct of Options containing the options
       */
      Options read(const std::string& _filename);
    private:
      static std::shared_ptr<ConfigManager> m_instance;
      Options options;
    private:
      /**
       * \brief create a new ConfigManager object
       */
      ConfigManager() { ; }
      /**
       * \brief print the parameters of the confing file
       */
      void print();
  };
}

#endif