#include "svm.h"

using namespace ml;

SupportVectorMachine::SupportVectorMachine() : Classifier()
{
  //Creating SVM
  svm = cv::ml::SVM::create();
  //Setting parameters
  svm->setType(cv::ml::SVM::C_SVC);
  svm->setKernel(cv::ml::SVM::RBF);
  svm->setTermCriteria(cv::TermCriteria(cv::TermCriteria::MAX_ITER, 100, 1e-6));
  svm->setGamma(1.0); // for poly/rbf/sigmoid
  svm->setCoef0(0.); // for poly/sigmoid
  svm->setC(0.5);
  svm->setNu(.5);
}

SupportVectorMachine::SupportVectorMachine(const std::string& _name) : Classifier()
{
  svm = cv::ml::SVM::create();
  svm = cv::ml::SVM::load(_name);
  //svm = cv::ml::SVM::load<cv::ml::SVM>(_name);
}

std::vector< float > SupportVectorMachine::predict(const std::vector< std::pair< float, std::vector< float > > >& _features)
{
  std::vector<float> predictions;
  for(auto& feature : _features)
  {
    cv::Mat featureMat(feature.second, false);
    featureMat = featureMat.t();
    predictions.push_back(svm->predict(featureMat.row(0)));
  }
  return predictions;
}

void SupportVectorMachine::validate(const cv::Mat& _labels, const cv::Mat& _features)
{
  if(_features.rows == 0)
  {
    std::cerr << "The size of the validation features is equal to 0" << std::endl;
    exit(-1);
  }
  std::cout << "Validating SVM with " << _features.rows << " samples!" << std::endl;
  float err = 0.;
  for(uint i = 0; i < _features.rows; ++i)
  {
    const float& resp = svm->predict(_features.row(i));
    err += (_labels.at<float>(i) != resp);
  }
  
  std::cout << "Errors = " << err << "/" << _features.rows << " - Accuracy = " << (1.0f-(float)err/_features.rows)*100 << " %" << std::endl;
}

void SupportVectorMachine::train(const std::vector< std::pair< float, std::vector< float > > >& _features, const float& _val)
{
  if(_features.size() == 0) 
  {
    std::cerr << "The size of the feature vector is equal to 0" << std::endl;
    exit(-1);
  }
  
  
  std::vector< std::pair< float, std::vector< float > > > features = _features;
  const uint& n_samplesVal = cvRound(_features.size() * _val);
  const uint& n_samples = _features.size() - n_samplesVal;
  const uint& feature_size = _features[0].second.size();
  std::cout << "Training SVM with " << n_samples << " samples!" << std::endl;

  //Mixing _features
  std::random_shuffle ( features.begin(), features.end() );
  cv::Mat trainLabels(cv::Size(1, n_samples), CV_32F);
  cv::Mat trainFeatures(cv::Size(feature_size, n_samples), CV_32F);
  
  cv::Mat validationLabels(cv::Size(1, n_samplesVal), CV_32F);
  cv::Mat validationFeatures(cv::Size(feature_size, n_samplesVal), CV_32F);

  for(uint i = 0; i < features.size(); ++i)
  {
    const std::pair< int, std::vector< float > >& feature = features.at(i);
    if(i < n_samples)
    {
      trainLabels.at<float>(i) = feature.first;
      
      for(uint j = 0; j < feature_size; ++j)
      {
	trainFeatures.at<float>(i, j) = feature.second.at(j);
	++j;
      }
    }
    else
    {
      validationLabels.at<float>(i - n_samples) = feature.first;
      
      for(uint j = 0; j < feature_size; ++j)
      {
	validationFeatures.at<float>(i - n_samples, j) = feature.second.at(j);
	++j;
      }
    }
  }
  trainLabels.convertTo(trainLabels, CV_32SC1);
  svm->train(trainFeatures, cv::ml::ROW_SAMPLE, trainLabels);
  
  validate(validationLabels, validationFeatures);
  
  std::cout << "Done." << std::endl;
  
}



