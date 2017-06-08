/**
 * \file rectangle.h
 *
 * \class Rectangle
 *
 * \brief Class that represents a rectangle
 *
 **/

#ifndef _RECTANGLE_H_
#define _RECTANGLE_H_

#include "point.h"

namespace utils
{
    class Rectangle
    {
        public:
	    /**
	     * \brief Create a Rectangle object
	     */
            Rectangle();
	    /**
	     * \brief Move the vertices of the rectangle
	     * \param _x int containing the current x coordinate pointed by the mouse
	     * \param _y int containing the current y coordinate pointed by the mouse
	     */
            void move(const int& _x, const int &_y);
	    /**
	     * \brief Draw the rectangle
	     * \param _x int containing the current x coordinate pointed by the mouse
	     * \param _y int containing the current y coordinate pointed by the mouse
	     */
            void rectangle(const int& _x, const int &_y);
	    /**
	     * \brief Transform a string containing points into a rectangle
	     * \param _rect a std::string containing the points
	     * \return true if the conversion succed, false otherwise
	     */
	    bool rectFromString(const std::string& _rect);
	    /**
	     * \brief Draw the rectangle 
	     * \brief _image cv::Mat containing the image
	     */
            const void draw(cv::Mat& _image);
	    /**
	     * \brief Draw the rectangle given a prediction
	     * \brief _image cv::Mat containing the image
	     * \brief prediction float containing the prediction
	     */
	    const void draw(cv::Mat& _image, const float& _prediction);
	    /**
	     * \brief Release the selected points
	     */
            const void releasePoint();
	    /**
	     * \brief Check if a Rectangle has been selected
	     * \param _x int containing the current x coordinate pointed by the mouse
	     * \param _y int containing the current y coordinate pointed by the mouse
	     */
	    const bool isSelected(const int& _x, const int& _y);

	
	public:
	    /**
	     * \brief Convert a rectangle to a string
	     * \return a std::string containing the vertices of the rectangle
	     */
	    const std::string rectToString() 
	    {
	      std::stringstream ss;
	      ss << vertices[0]().x << " " << vertices[0]().y << " " << vertices[1]().x << " "
	      << vertices[1]().y << " "  << vertices[2]().x << " " << vertices[2]().y << " "
	      << vertices[3]().x << " " << vertices[3]().y << "\n";
	      return ss.str();
	    }
	    /**
	     * \brief Return the vertices of the rectangle
	     * \return a std::vector of cv::Point2f containing the vertices
	     */
            const std::vector<cv::Point2f> getPoints()
            {
                std::vector<cv::Point2f> points;
                for(auto &v : vertices)
                {
                    points.push_back(v());
                }
                return points;
            }
            Rectangle& operator=(const Rectangle& copyRect)
	    {
	      this->point_selected = copyRect.point_selected;
	      this->start_point = copyRect.start_point;
	      this->m_empty = false;
	      return *this;
	    }
	    /**
	     * \brief Set the label of the rectangle
	     * \param _label float containing the label
	     */
	    const inline void setLabel(const float& _label)
	    {
	      label = _label;
	    }
	    /**
	     * \brief Return the label of the rectangle
	     * \return a float containing the lable
	     */
	    const inline float getLabel()
	    {
	      return label;
	    }
	    /**
	     * \brief Return a boolean indicating if the rectangle has been selected
	     * \return true if the rectangle has been selected, false otherwise
	     */
	    const inline bool selected() const
	    {
	      return m_selected;
	    }
	    /**
	     * \brief Deselect the rectangle
	     */
	    const void deselect()
	    {
	      m_selected = false;
	    }
	    /**
	     * \brief Return if a rectangle is empty
	     */
	    const inline bool empty() const
	    {
	      return m_empty;
	    }
	   
        private:
            std::vector<Point> vertices;
            static constexpr int offset = 100;
            bool point_selected;
            bool start_point;
	    bool m_empty;
	    bool m_selected;
            int pointIdx;
	    float label;
	    cv::Scalar green;
	    cv::Scalar red;
	private:
	    /**
	      * \brief Put the text inside the image
	      * \param _image cv::Mat where applying the text
	      */
	    const void putText(cv::Mat& _img);


    };
}

#endif
