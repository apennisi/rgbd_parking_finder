/**
 * \file adaboost.h
 *
 * \class AdaBoost
 *
 * \brief Class that implements the AdaBoost classifier
 *
 **/

#ifndef _ADABOOST_H_
#define _ADABOOST_H_

#include "classifier.h"

namespace ml
{
  class AdaBoost : public Classifier
  {
    public:
      /**
       * \brief Create a new AdaBoost object
       */
      AdaBoost();
      /**
       * \brief Create a new AdaBoost object given a pre-trained model
       * \param _name std::string containing the path to the pre-trained adaboost model
       */
      AdaBoost(const std::string& _name);
      /**
       * \brief Train the classifier
       * \param _features std::vector of features and labels
       * \param _val float containing the percentage of the data set that has to be used for validating the classifier
       */
      void train(const std::vector<std::pair<float, std::vector<float> > >& _features, const float& _val);
      /**
       * \brief Save the trained classifier
       * \param _name std::string containing the path for saving the classifer
       */
      const inline void save(const std::string& _name)
      {
	if(boost->isTrained())
	  boost->save(_name);
      }
      /**
       * \brief Predict the current set of features
       * \param _features std::vector containing the features and the labels
       * \return a std::vector of float containing the predictions
       */
      std::vector<float> predict(const std::vector<std::pair<float, std::vector<float> > >& _features);
    private:
      cv::Ptr<cv::ml::Boost> boost;
    private:
      /**
       * \brief Validate the trained classifer
       * \param _labels cv::Mat containing the labels
       * \param _features cv::Mat containing the features
       */
      void validate(const cv::Mat& _labels, const cv::Mat& _features);
  };
}

#endif