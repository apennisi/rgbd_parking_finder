/**
 * \file parking_selection.h
 *
 * \class ParkingSelection
 *
 * \brief Abstract class that implements a feature extractor
 *
 **/

#ifndef _PARKING_SELECTION_H_
#define _PARKING_SELECTION_H_

#include <iostream>
#include <memory>
#include "rectangle.h"

namespace utils
{
    
    class ParkingSelection
    {
      public:
	/**
	  * \brief create a single instance of the ParkingSelection object
	  * \return a std::shared_ptr of ParkingSelection type
	  */
	static std::shared_ptr<ParkingSelection> instance();
	/**
	 * \brief Select the parking bays by using the mouse
	 * \param _img cv::Mat containing the plan view
	 * \param _planimetry std::vector of utils::Rectangle that will contain the coordinates of the bays
	 */
	void process(const cv::Mat& _img, std::vector<utils::Rectangle>& _planimetry);
	struct DataStruct
	{
	  cv::Mat img;
	  bool left_pressed;
	  bool draw_rect;
	  std::vector<utils::Rectangle> rects;
	  std::shared_ptr<utils::Rectangle> rect;
	};
      private:
	/**
       * \brief Create a new ParkingSelection object
       */
	ParkingSelection() { ; }
	/**
	 * \brief Mouse Callaback for selecting the bays
	 * \param event int containing the mouse event type
	 * \param x int containing the x coordinate
	 * \param y int containing the y coordinate
	 * \param data a void pointer containing the date to manipulate insiede the mouse callback
	 */
	static void mouse_callback(int event, int x, int y, int, void *data);
	/**
	 * \brief Put the text inside the image
	 * \param _image cv::Mat where applying the text
	 */
	static void text(cv::Mat& _img);
      private:
	static std::shared_ptr<ParkingSelection> m_instance;
	std::vector<utils::Rectangle> rects;
	bool left_pressed, draw_rect;
	cv::Mat selectionImage;
	
    };
}

#endif