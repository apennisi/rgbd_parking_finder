#include "configmanager.h"

using namespace utils;

std::shared_ptr<ConfigManager> ConfigManager::m_instance = nullptr;

std::shared_ptr< ConfigManager > ConfigManager::instance()
{
  if(!m_instance)
  {
    m_instance = std::shared_ptr<ConfigManager>(new ConfigManager);
  }
  return m_instance;
}

std::string cloud_file;
    std::string planimetry_file;
    std::string geometry_file;
    std::string planimetry_image;
    std::string feature_file;
    std::string mode;
    std::string classifier;
    std::string classifier_file;
    float validation_percentage;

void ConfigManager::print()
{

  if(!options.mode.empty())
    std::cout << "MODE: " << options.mode << std::endl;
  
  if(!options.classifier.empty())
    std::cout << "CLASSIFIER: "  << options.classifier << std::endl;
  
  if(!options.validation_percentage != -1.)
    std::cout << "VALIDATION PERCENTAGE: "  << options.validation_percentage << std::endl;
  
  if(!options.classifier_file.empty())
    std::cout << "CLASSIFIER FILE: " << options.classifier_file << std::endl;
  
  if(!options.feature_file.empty())
    std::cout << "FEATURE FILE: " << options.feature_file << std::endl;
  
  if(options.feature_type.size() != 0)
  {
    std::cout << "FEATURE TYPE: ";
    for(const auto& feature : options.feature_type)
      std::cout << feature << " ";
    
    std::cout << std::endl;
  }
  
  if(!options.cloud_file.empty())
    std::cout << "POINT CLOUD FILE: " << options.cloud_file << std::endl;
  
  if(!options.planimetry_file.empty())
    std::cout << "PLANIMETRY FILE: " << options.planimetry_file << std::endl;
  
  if(!options.planimetry_image.empty())
    std::cout << "PLANIMETRY IMAGE: " << options.planimetry_image << std::endl;
  
  if(!options.geometry_file.empty())
    std::cout << "TRANSFORMATION FILE: " << options.geometry_file << std::endl;
}

Options ConfigManager::read(const std::string& _filename)
{
  if(_filename.find(".cfg") == std::string::npos)
  {
    std::cerr << "Error: " << _filename << " does not exist!" << std::endl;
    exit(-1);
  }
  
  std::ifstream file;
  
  try
  {
    file.open(_filename);
  }
  catch(...)
  {
    std::cerr << "Cannot open " << _filename << std::endl;
    file.close();
    exit(-1);
  }
  
  if(!file.is_open())
  {
    std::cerr << "File " << _filename << " does not exist!" << std::endl;
    file.close();
    exit(-1);
  }
  
  std::string line;
  while(std::getline(file, line))
  {
      std::remove_if(line.begin(), line.end(), isspace);
      if(line.empty())
      {
	continue;
      }
      else if(line.find("[MODE]") != std::string::npos)
      {
	std::getline(file, line);
	if(line.find("read") != std::string::npos || line.find("compute") != std::string::npos)
	{
	  options.mode = line;
	}
      }
      else if(line.find("[CLASSIFIER]") != std::string::npos)
      {
	std::getline(file, line);
	if(line.find("adaboost") != std::string::npos || line.find("svm") != std::string::npos)
	{
	  options.classifier = line;
	}
      }
      else if(line.find("[VALIDATION_PERCENTAGE]") != std::string::npos)
      {
	std::getline(file, line);
	try
	{
	  options.validation_percentage = atof(line.c_str());
	}
	catch(...)
	{
	  std::cerr << "Error in converting the validation percentage: " << line << std::endl;
	  exit(-1);
	}
      }
      else if(line.find("[CLASSIFIER_FILE_SAVE]") != std::string::npos)
      {
	std::getline(file, line);
	options.classifier_file = line;
      }
      else if(line.find("[FEATURES_FILE]") != std::string::npos)
      {
	std::getline(file, line);
	options.feature_file = line;
      }
      else if(line.find("[FEATURES_TYPE]") != std::string::npos)
      {
	std::getline(file, line);
	try
	{
	  boost::split(options.feature_type, line, boost::is_any_of("+"));
	}
	catch(...)
	{
	  std::cerr << "Cannot split the feature string!" << std::endl;
	  exit(-1);
	}
      }
      else if(line.find("[POINT_CLOUD]") != std::string::npos)
      {
	std::getline(file, line);
	options.cloud_file = line;
      }
      else if(line.find("[PLANIMETRY_FILE]") != std::string::npos)
      {
	std::getline(file, line);
	options.planimetry_file = line;
      }
      else if(line.find("[PLANIMETRY_IMAGE]") != std::string::npos)
      {
	std::getline(file, line);
	options.planimetry_image = line;
      }
      else if(line.find("[TRANSFORMATION_FILE]") != std::string::npos)
      {
	std::getline(file, line);
	options.geometry_file = line;
      }
      else
      {
	std::cerr << "Option: " << line << " does not exist!" << std::endl;
	exit(-1);
      }
    
  }
 
  file.close();
  print();
  
  return options;
}
