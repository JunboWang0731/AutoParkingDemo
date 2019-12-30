#include <iostream>
#include <algorithm>
#include <iterator>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>

#include "/home/junbo/dev/ROS_Tutorials/src/movingdetect/src/algorithms/algorithms.h"

/* Background Subtraction Methods */
auto algorithmsName = BGS_Factory::Instance()->GetRegisteredAlgorithmsName();

int main(int argc, char** argv)
{
  std::cout << "Using OpenCV " << CV_MAJOR_VERSION << "." << CV_MINOR_VERSION << "." << CV_SUBMINOR_VERSION << std::endl;
  
  std::cout << "Number of available algorithms: " << algorithmsName.size() << std::endl;
  std::cout << "List of available algorithms:" << std::endl;
  std::copy(algorithmsName.begin(), algorithmsName.end(), std::ostream_iterator<std::string>(std::cout, "\n"));

  /*
   List of all algorithms:
   (Note that some of these algorithms are available only for a specific version of OpenCV, see algorithms.h)
   AdaptiveBackgroundLearning,AdaptiveSelectiveBackgroundLearning,CodeBook,DPAdaptiveMedian,DPEigenbackground,
   DPGrimsonGMM,DPMean,DPPratiMediod,DPTexture,DPWrenGA,DPZivkovicAGMM,FrameDifference,FuzzyChoquetIntegral,
   FuzzySugenoIntegral,GMG,IndependentMultimodal,KDE,KNN,LBAdaptiveSOM,LBFuzzyAdaptiveSOM,LBFuzzyGaussian,
   LBMixtureOfGaussians,LBP_MRF,LBSimpleGaussian,LOBSTER,MixtureOfGaussianV2,MixtureOfGaussianV1,MultiCue,
   MultiLayer,PAWCS,PixelBasedAdaptiveSegmenter,SigmaDelta,StaticFrameDifference,SuBSENSE,T2FGMM_UM,T2FGMM_UV,
   T2FMRF_UM,T2FMRF_UV,TwoPoints,ViBe,VuMeter,WeightedMovingMean,WeightedMovingVariance
  */
  std::string algorithmName = "FrameDifference";
  int cameraIndex = 0;
  if (argc > 1) algorithmName = argv[1];
  if (argc > 2) cameraIndex = std::stoi(argv[2]);
  
  cv::VideoCapture capture;
  capture.open(cameraIndex);
  
  if (!capture.isOpened()) {
    std::cerr << "Cannot initialize web camera!" << std::endl;
    return -1;
  }
  
  std::cout << "Running " << algorithmName << std::endl;
  auto bgs = BGS_Factory::Instance()->Create(algorithmName);
  
  cv::Mat img_input;
  auto key = 0;
  std::cout << "Press 's' to stop:" << std::endl;
  while (key != 's') {
    // Capture frame-by-frame
    capture >> img_input;
    
    if (img_input.empty()) break;
    
    // Resize input frame for better visualization
    cv::resize(img_input, img_input, cv::Size(380, 240), 0, 0, CV_INTER_LINEAR);
    cv::imshow("input", img_input);
    
    cv::Mat img_mask;
    cv::Mat img_bkgmodel;
    try {
      // by default, bgs->process(...) shows automatically the foreground mask image
      // or you can disable it by: bgs->setShowOutput(false);
      bgs->process(img_input, img_mask, img_bkgmodel);
      
      //if(!img_mask.empty())
      //  cv::imshow("Foreground", img_mask);
      //  ....do something else...
    }
    catch (std::exception& e) {
      std::cout << "Exception occurred" << std::endl;
      std::cout << e.what() << std::endl;
    }
    
    key = cv::waitKey(33);
  }
  
  cv::destroyAllWindows();
  capture.release();

  return 0;
}
