#include "featurefilemanager.h"

using namespace utils;

std::shared_ptr<FeatureReaderWriter> FeatureReaderWriter::m_instance = nullptr;

std::shared_ptr<FeatureReaderWriter> FeatureReaderWriter::instance()
{
    if(!m_instance)
    {
        m_instance = std::shared_ptr<FeatureReaderWriter>(new FeatureReaderWriter);
    }
    return m_instance;
}
void FeatureReaderWriter::split(const std::string &s, char delim, std::vector<std::string> &elems) 
{
    std::stringstream ss;
    ss.str(s);
    std::string item;
    while (std::getline(ss, item, delim)) 
    {
        elems.push_back(item);
    }
}


std::vector<std::string> FeatureReaderWriter::split(const std::string &s, char delim) 
{
    std::vector<std::string> elems;
    split(s, delim, elems);
    return elems;
}

void FeatureReaderWriter::read(const std::string& _name, std::vector<std::pair<float, std::vector<float> > >& _features)
{
  if(_name.find(".txt") == std::string::npos)
  {
    std::cerr << "Error: " << _name << " does not exist!" << std::endl;
    exit(-1);
  }
  
  std::ifstream file;
  try
  {
    file.open(_name);
  }
  catch(...)
  {
    std::cerr << "Cannot open " << _name << std::endl;
    file.close();
    exit(-1);
  }
  
  if(!file.is_open())
  {
    std::cerr << "File " << _name << " does not exist!" << std::endl;
    file.close();
    exit(-1);
  }
  
  uint f_number;
  std::string line;
  std::getline(file, line);
  f_number = atoi(line.c_str());
  
  
  
  while(std::getline(file, line))
  {
    std::vector<float> featureVals;
    const std::vector<std::string>& str = split(line, ' ');
    for(uint i = 0; i < f_number; ++i)
    {
      featureVals.push_back(atof(str.at(i).c_str()));
    }
    float label = atof(str.at(f_number).c_str());
    _features.push_back(std::make_pair(label, featureVals));
  }
  
  file.close();
}

void FeatureReaderWriter::write(const std::vector< std::pair<float, std::vector<float> > >& _features, const std::string& _name)
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
  
  file << _features[0].second.size() << std::endl;
  for(const auto& feature : _features)
  {
    for(const auto& value : feature.second)
    {
      file << std::fixed << std::setprecision(5) << value << " ";
    }
    
    file << feature.first << std::endl;
  }
  std::cout << "Writing Features Done!" << std::endl;
  file.close();
}