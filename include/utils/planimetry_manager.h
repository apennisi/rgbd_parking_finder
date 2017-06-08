/**
 * \file planimetry_manager.h
 *
 * \class PlanimetryManager
 *
 * \brief Class that manage the planimetry 
 *
 **/

#ifndef _PLANIMETRY_MANAGER_H_
#define _PLANIMETRY_MANAGER_H_

#include <iostream>
#include <fstream>
#include "rectangle.h"

namespace utils
{
  class PlanimetryManager
  {
    public:
      /**
       * \brief Create a PlanimetryManager object
       */
      PlanimetryManager() { ; }
      /**
       * \brief Read the planimetry from a file
       * \param _filename std::string containing the path to the file
       */
      void readFromFile(const std::string& _filename);
      /**
       * \brief Label the plan view
       * \param _img cv::Mat containing the plane view
       */
      void managePlaneView(const cv::Mat& _img);
      /**
       * \brief Show the plan view containing the labels provided by a classifier
       * \brief _img cv::Mat containing the plan view
       * \brief _predictions std::vector of float containing the labels
       */
      void classificationViewer(cv::Mat& _img, const std::vector<float>& predictions);
      /**
       * \brief Return the planimetry rectangles
       * \return a std::vector of utils::Rectangle containing the planimetry rectangles
       */
      const inline std::vector<utils::Rectangle> getRects() { return rects; } 
      ~PlanimetryManager();
    private:  
      std::vector<utils::Rectangle> rects;
      struct DataStruct
      {
	std::vector<utils::Rectangle> rects;
	int selected_index;
	cv::Mat img;
      };
    private:
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
      void text(cv::Mat& _img);
  };
}

#endif