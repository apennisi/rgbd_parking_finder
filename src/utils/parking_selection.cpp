#include "parking_selection.h"

using namespace utils;

std::shared_ptr<ParkingSelection> ParkingSelection::m_instance = nullptr;

std::shared_ptr< ParkingSelection > ParkingSelection::instance()
{
  if(!m_instance)
  {
    m_instance = std::shared_ptr<ParkingSelection>(new ParkingSelection);
  }
  
  return m_instance;
}

// Callback function for mouse events
void ParkingSelection::mouse_callback(int event, int x, int y, int, void *data)
{
    DataStruct *d = static_cast<DataStruct*>(data);
    // When the left mouse button is pressed
    if(event == cv::EVENT_LBUTTONDOWN)
    {
        d->left_pressed = true;
    }
    // When the left mouse button is released
    else if(event == cv::EVENT_RBUTTONDOWN)
    {

    }

    else if ( event == cv::EVENT_MOUSEMOVE )
    {
        if(d->left_pressed)
        {
            if(d->draw_rect)
            {
                d->rect->rectangle(x, y);
            }
            else
            {
                d->rect->move(x, y);
            }
            cv::Mat img = d->img.clone();
            d->rect->draw(img);
            cv::imshow("Select Parking", img);
        }
    }
    else if (event == cv::EVENT_LBUTTONUP)
    {
        d->left_pressed = false;
        if(d->draw_rect) d->draw_rect = false;
        d->rect->releasePoint();
    }
}

void ParkingSelection::text(cv::Mat& _img)
{
  cv::putText(_img, "a - adding rectangle", cv::Point(15, 25), cv::FONT_HERSHEY_SIMPLEX, 1.2 , cv::Scalar(0,255,0));
  cv::putText(_img, "q - quitting and writing plan view", cv::Point(15, 55), cv::FONT_HERSHEY_SIMPLEX, 1.2 , cv::Scalar(0,255,0));
}


void ParkingSelection::process(const cv::Mat& _img, std::vector<utils::Rectangle>& _planimetry)
{
  char key = ' ';

  DataStruct data;
  cv::namedWindow("Select Parking");
  cv::setMouseCallback("Select Parking", mouse_callback, (void*)&data);
  while(key != 'q')
  {
    selectionImage = _img.clone();
    for(auto rect : data.rects)
    {
      rect.draw(selectionImage);
    }
    text(selectionImage);
    data.img = selectionImage.clone();
    data.left_pressed = false;
    data.draw_rect = true;
    data.rect = std::shared_ptr<utils::Rectangle>(new utils::Rectangle);
  
    cv::imshow("Select Parking", selectionImage);
    key = cv::waitKey(0);
    switch(key)
    {
      case 'a':
	if(!data.rect->empty())
	{
	  data.rects.push_back(*(data.rect));
	  std::cout << "Added" << std::endl;
	}
	break;
      case 'q':
      default:
	break;
    }
    data.rect.reset();
  }
  _planimetry = data.rects;
}


