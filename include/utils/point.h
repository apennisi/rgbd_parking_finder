/**
 * \file point.h
 *
 * \class Point
 *
 * \brief Class that represents a point
 *
 **/

#ifndef _POINT_H_
#define _POINT_H_

#include <opencv2/opencv.hpp>

namespace utils
{
    class Point
    {
        public:
	    /**
	    * \brief Create a Point object
	    */
            Point() {;}
            /**
	    * \brief Create a Point object
	    * \param _x int containing the x coordinate of the point
	    * \param _y int containing the y coordinate of the point
	    */
            Point(const int& _x, const int& _y);
	    /**
	     * \brief Check if a point is selected
	     * \param _x int containing the x coordinate of the current selected point
	     * \param _y int containing the y coordinate of the current selected point
	     */
            bool isSelected(const int& _x, const int& _y);
	    /**
	     * \brief Move the point coordinates
	     * \param _x int containing the new x coordinate
	     * \param _y int containing the new y coordinate
	     */
            void move(const int& _x, const int& _y);
	    /**
	     * \brief Draw the point
	     * \param _image cv::Mat where to draw the point
	     */
            void draw(cv::Mat &_image) const;
	    /**
	     * \brief Draw the point
	     * \param _image cv::Mat where to draw the point
	     * \param _color cv::Scalar containing the color
	     */
	    void draw(cv::Mat &_image, const cv::Scalar& _color) const;
	    
	    Point& operator=(const Point& copyRect)
	    {
	      this->p = copyRect.p;
	      this->selected = copyRect.selected;
	      return *this;
	    }
	    
            cv::Point operator()()
            {
                return p;
            }

        private:
            cv::Point p;
            bool selected;
            static constexpr int offset = 4;
    };
}

#endif
