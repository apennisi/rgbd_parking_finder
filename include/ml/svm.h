/**
 * \file svm.h
 *
 * \class SupportVectorMachine
 *
 * \brief Class that implements the SupportVectorMachine classifier
 *
 **/


#ifndef _SVM_H_
#define _SVM_H_

#include "classifier.h"

namespace ml
{
  class SupportVectorMachine : public Classifier
  {
    public:
      /**
       * \brief Create a new SupportVectorMachine object
       */
      SupportVectorMachine();
      /**
       * \brief Create a new SupportVectorMachine object given a pre-trained model
       * \param _name std::string containing the path to the pre-trained adaboost model
       */
      SupportVectorMachine(const std::string& _name);
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
	if(svm->isTrained())
	  svm->save(_name);
      }
      /**
       * \brief Predict the current set of features
       * \param _features std::vector containing the features and the labels
       * \return a std::vector of float containing the predictions
       */
      std::vector<float> predict(const std::vector<std::pair<float, std::vector<float> > >& _features);
    private:
      cv::Ptr<cv::ml::SVM> svm;
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