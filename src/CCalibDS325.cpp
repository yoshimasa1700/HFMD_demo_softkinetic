#include "CCalibDS325.h"

int CCalibDS325::loadParameters(std::string inName, std::string exName){
  // save intrinsic parameters
  cv::FileStorage fs(inName.c_str() , CV_STORAGE_READ);
  if( fs.isOpened() )
    {
      fs["M1"] >> cameraMatrix[0];
      fs["D1"] >> distCoeffs[0];
      fs["M2"] >> cameraMatrix[1];
      fs["D2"] >> distCoeffs[1];
      
      fs.release();
    }
  else
    {
      std::cout << "Error: can not save the intrinsic parameters\n";
      return -1;
    }

  fs.open(exName.c_str() , CV_STORAGE_READ);
  if( fs.isOpened() )
    {
      fs["R"] >> R;
      fs["T"] >> T;
      fs["R1"] >> R1;
      fs["R2"] >> R2;
      fs["P1"] >> P1;
      fs["P2"] >> P2;
      fs["Q"] >> Q;
      cv::Mat_<int> vroiMat;
      fs["vroi"] >> vroiMat;

      for(int i = 0; i < 2; ++i){
	validRoi[i].x = vroiMat.at<int>(i,0);
	validRoi[i].y = vroiMat.at<int>(i,1);
	validRoi[i].width = vroiMat.at<int>(i,2);
	validRoi[i].height = vroiMat.at<int>(i,3);
      }

      fs.release();
    }
  else
    {
      std::cout << "Error: can not save the extrinsic parameters\n";
      return -1;
    }
  return 0;
}

int CCalibDS325::calib(cv::Mat &colorSrc, cv::Mat &depthSrc, cv::Mat &colorDest, cv::Mat &depthDest){
  cv::Mat scaledDepth;
  cv::Mat maxDist = cv::Mat::ones(depthSrc.rows * 2, depthSrc.cols * 2, CV_16U) * MAX_DEPTH;
  cv::Mat minDist = cv::Mat::ones(depthSrc.rows * 2, depthSrc.cols * 2, CV_16U) * MIN_DEPTH;

  cv::resize(depthSrc, scaledDepth, cv::Size(640,480));
  cv::min(scaledDepth, maxDist, scaledDepth);

  scaledDepth -= minDist;


   cv::initUndistortRectifyMap(
			      cameraMatrix[0], 
			      distCoeffs[0], 
			      R1, 
			      P1, 
			      colorSrc.size(), 
			      CV_16SC2, 
			      rmap[0][0], rmap[0][1]
			      );
  cv::initUndistortRectifyMap(
			      cameraMatrix[1], 
			      distCoeffs[1], 
			      R2, 
			      P2, 
			      colorSrc.size(), 
			      CV_16SC2, 
			      rmap[1][0], 
			      rmap[1][1]
			      );
  {
    cv::Mat rimg;
    cv::remap(colorSrc, rimg, rmap[0][0], rmap[0][1], CV_INTER_LINEAR);
    cv::resize(rimg(validRoi[0]), colorDest, cv::Size(640,480));
  }
  {
    cv::Mat rimg;

    cv::Mat depthPart = scaledDepth(cv::Rect(40, 43,498,498 / 4 * 3));
    cv::resize(depthPart, depthPart, cv::Size(640, 480));
    cv::remap(depthPart, rimg, rmap[1][0], rmap[1][1], CV_INTER_LINEAR);
    
    cv::resize(rimg(validRoi[1]), depthDest, cv::Size(640,480));
  }

  return 0;
}
