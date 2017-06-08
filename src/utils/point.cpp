#include "point.h"

using namespace utils;



Point::Point(const int &_x, const int &_y)
    : p(cv::Point(_x, _y))
{
    selected = false;
}

bool Point::isSelected(const int &_x, const int &_y)
{
    if(_x >= p.x - offset && _x <= p.x + offset && _y >= p.y - offset && _y <= p.y + offset)
    {
        return true;
    }

    return false;
}

void Point::move(const int &_x, const int &_y)
{
    p.x = _x;
    p.y = _y;
}

void Point::draw(cv::Mat &_image) const
{
    cv::rectangle(_image, cv::Rect(p.x - offset, p.y - offset, 2*offset, 2*offset),
                  cv::Scalar(0, 0, 255), -1);
}

void Point::draw(cv::Mat& _image, const cv::Scalar& _color) const
{
  cv::rectangle(_image, cv::Rect(p.x - offset, p.y - offset, 2*offset, 2*offset),
                  _color, -1);
}



