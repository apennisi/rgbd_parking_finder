#include "rectangle.h"

using namespace utils;

Rectangle::Rectangle()
{
    start_point = true;
    m_selected = false;
    red = cv::Scalar(0, 0, 255);
    green = cv::Scalar(0, 255, 0);
    label = -100;
    m_empty = true;
}


void Rectangle::rectangle(const int& _x, const int &_y)
{
    if(start_point)
    {
        vertices.resize(4, Point(_x, _y));
        start_point = false;
	m_empty = false;
    }
    else
    {
        vertices[1].move(_x, vertices[0]().y);
        vertices[2].move(_x, _y);
        vertices[3].move(vertices[0]().x, _y);
    }
}

void Rectangle::move(const int &_x, const int &_y)
{
    if(!point_selected)
    {
        pointIdx = -1;
        for(auto &vertex : vertices)
        {
            pointIdx++;
            if(vertex.isSelected(_x, _y))
            {
                vertex.move(_x, _y);
                point_selected = true;
                break;
            }

        }
    }
    else
    {
        vertices[pointIdx].move(_x, _y);
    }
}

const void Rectangle::draw(cv::Mat &_image)
{
  if(!m_empty)
  {
    const cv::Scalar& color = (m_selected) ? green : red;
    vertices[0].draw(_image, color);
    for(const auto &vertex : vertices)
    {
        vertex.draw(_image, color);
    }
    
    
    cv::line(_image, vertices[0](), vertices[1](), color, 2);
    cv::line(_image, vertices[1](), vertices[2](), color, 2);
    cv::line(_image, vertices[2](), vertices[3](), color, 2);
    cv::line(_image, vertices[3](), vertices[0](), color, 2);

    if(label != -100)
      putText(_image);
  }
}

const void Rectangle::draw(cv::Mat& _image, const float& _prediction)
{
  if(!m_empty)
  {
    const cv::Scalar& color = (_prediction == -1) ? green : red;
    vertices[0].draw(_image, color);
    for(const auto &vertex : vertices)
    {
        vertex.draw(_image, color);
    }
    
    cv::line(_image, vertices[0](), vertices[1](), color, 2);
    cv::line(_image, vertices[1](), vertices[2](), color, 2);
    cv::line(_image, vertices[2](), vertices[3](), color, 2);
    cv::line(_image, vertices[3](), vertices[0](), color, 2);
    label = _prediction;
    putText(_image);
  }
}


const void Rectangle::releasePoint()
{
    point_selected = false;
    pointIdx = -1;
}
 
const bool Rectangle::isSelected(const int& _x, const int& _y)
{
  if(_x >= vertices[0]().x && _x >= vertices[3]().x && _x <= vertices[1]().x && _x <= vertices[2]().x
      && _y >= vertices[0]().y && _y >= vertices[1]().y && _y <= vertices[2]().y && _y <= vertices[3]().y)
  {
    m_selected = true;
  }
  else
  {
    m_selected = false;
  }
  return m_selected;
}

bool Rectangle::rectFromString(const std::string& _rect)
{
  uint x1, y1, x2, y2, x3, y3, x4, y4;
  std::istringstream iss(_rect);
  if (!(iss >> x1 >> y1 >> x2 >> y2 >> x3 >> y3 >> x4 >> y4)) { return false; } // error
  vertices.push_back(utils::Point(x1, y1));
  vertices.push_back(utils::Point(x2, y2));
  vertices.push_back(utils::Point(x3, y3));
  vertices.push_back(utils::Point(x4, y4));
  m_empty = false;
  return true;
}

const void Rectangle::putText(cv::Mat& _img)
{
  std::string l;
  if(label == -1.)
  {
    l = "F";
  }
  else if(label == 1.)
  {
    l = "C";
  }
  else
  {
    std::cerr << "Error: label does not exist!" << std::endl;
    exit(-1);
  }
  cv::Point center((vertices[1]().x + vertices[0]().x) / 2 - 5, (vertices[2]().y + vertices[1]().y) / 2);
  cv::putText(_img, l, center, cv::FONT_HERSHEY_DUPLEX, 0.8 , cv::Scalar(0,255,0));
}


