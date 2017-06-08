/**
 * \file classifier.h
 *
 * \class Classifier
 *
 * \brief Abstract class that implements a clasifier
 *
 **/

#ifndef _CLASSIFIER_H_
#define _CLASSIFIER_H_

#include <opencv2/ml.hpp>
#include <opencv2/opencv.hpp>

namespace ml
{
  class Classifier
  {
    public:
      /**
       * \brief Create a new Classifier object
       */
      Classifier() { ; }
      /**
       * \brief Train the classifier
       * \param _features std::vector of features and labels
       * \param _val float containing the percentage of the data set that has to be used for validating the classifier
       */
      virtual void train(const std::vector<std::pair<float, std::vector<float> > >& _features, const float& _val) = 0;
      /**
       * \brief Save the trained classifier
       * \param _name std::string containing the path for saving the classifer
       */
      virtual const inline void save(const std::string& _name) = 0;
      /**
       * \brief Predict the current set of features
       * \param _features std::vector containing the features and the labels
       * \return a std::vector of float containing the predictions
       */
      virtual std::vector<float> predict(const std::vector<std::pair<float, std::vector<float> > >& _features) = 0;
    protected:
      /**
       * \brief Validate the trained classifer
       * \param _labels cv::Mat containing the labels
       * \param _features cv::Mat containing the features
       */
      virtual void validate(const cv::Mat& _labels, const cv::Mat& _features) = 0;
  };
}

#endif