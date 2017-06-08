#include "planimetry_manager.h"

using namespace utils;

static int left_pressed = 0;

PlanimetryManager::~PlanimetryManager()
{
  rects.clear();
}


void PlanimetryManager::readFromFile(const std::string& _filename)
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
  
  rects.clear();
  
  std::string line;
  while(std::getline(file, line))
  {
    utils::Rectangle rect;
    rect.rectFromString(line);
    rects.push_back(rect);
  }
  
  file.close();
}

void PlanimetryManager::classificationViewer(cv::Mat& _img, const std::vector< float >& predictions)
{
  if(rects.size() != predictions.size())
  {
    std::cerr << "Predictions are different from detections!" << std::endl;
    exit(-1);
  }
  
  uint i = 0;
  for(const auto& prediction : predictions)
  {
    auto& rect = rects.at(i++);
    rect.draw(_img, prediction);
  }
}


void PlanimetryManager::mouse_callback(int event, int x, int y, int, void* data)
{
  DataStruct* d = static_cast< DataStruct* >(data);
  
  if(event == cv::EVENT_LBUTTONDOWN)
  {
    for(uint i = 0; i < d->rects.size(); ++i)
    {
      if(d->rects.at(i).isSelected(x, y))
      {
	if(d->selected_index != -1)
	{
	  d->rects.at(d->selected_index).deselect();
	  d->rects.at(d->selected_index).draw(d->img);
	  cv::imshow("Planimetry", d->img);
	}
	d->selected_index = i;
	break;
      }
    }
    if(d->selected_index != -1)
      d->rects.at(d->selected_index).draw(d->img);
    
    cv::imshow("Planimetry", d->img);
  }
  
}

void PlanimetryManager::text(cv::Mat& _img)
{
  cv::putText(_img, "c - for car", cv::Point(15, 25), cv::FONT_HERSHEY_SIMPLEX, 1.2 , cv::Scalar(0,255,0));
  cv::putText(_img, "f - for free bay", cv::Point(15, 55), cv::FONT_HERSHEY_SIMPLEX, 1.2 , cv::Scalar(0,255,0));
  cv::putText(_img, "q - for quitting and continuing the process", cv::Point(15, 85), cv::FONT_HERSHEY_SIMPLEX, 1.2 , cv::Scalar(0,255,0));
}


void PlanimetryManager::managePlaneView(const cv::Mat& _img)
{
  cv::Mat img;
  char key = ' ';
  DataStruct data;
  data.selected_index = -1;
  data.rects = rects;
  cv::namedWindow("Planimetry");
  cv::setMouseCallback("Planimetry", mouse_callback, &data);
  
  while(key != 'q')
  {
    img = _img.clone();
    text(img);
    for(auto& rect : data.rects)
    {
      rect.draw(img);
    }
    data.img = img.clone();
    
    cv::imshow("Planimetry", img);
    key = cv::waitKey(0);
    switch(key)
    {
      case 'f':
      {
	if(data.selected_index >= 0)
	{
	  auto& r = data.rects.at(data.selected_index);
	  std::cout << "Set as FREE" << std::endl;
	  r.setLabel(-1.);
	  r.deselect();
	}
	data.selected_index = -1;
	break;
      }
      case 'c':
      {
	if(data.selected_index >= 0)
	{
	  auto& r = data.rects.at(data.selected_index);
	  std::cout << "Set as CAR" << std::endl;
	  r.setLabel(1.);
	  r.deselect();
	}
	data.selected_index = -1;
	break;
      }
      default:
	break;
    }
  }
  
  cv::destroyAllWindows();
  rects = data.rects;
  
}

