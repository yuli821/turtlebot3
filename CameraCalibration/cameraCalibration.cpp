#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <iostream>

// Defining the dimensions of checkerboard
int CHECKERBOARD[2]{6,9}; 

int main()
{
  // Creating vector to store vectors of 3D points for each checkerboard image
  std::vector<std::vector<cv::Point3f> > objpoints;

  // Creating vector to store vectors of 2D points for each checkerboard image
  std::vector<std::vector<cv::Point2f> > imgpoints;

  // Defining the world coordinates for 3D points
  std::vector<cv::Point3f> objp;
  for(int i{0}; i<CHECKERBOARD[1]; i++)
  {
    for(int j{0}; j<CHECKERBOARD[0]; j++)
      objp.push_back(cv::Point3f(j,i,0));
  }


  // Extracting path of individual image stored in a given directory
  std::vector<cv::String> images;
  // Path of the folder containing checkerboard images
  std::string path = "./images3/*.jpg";

  cv::glob(path, images);

  cv::Mat frame, gray;
  // vector to store the pixel coordinates of detected checker board corners 
  std::vector<cv::Point2f> corner_pts;
  bool success;

  // Looping over all the images in the directory
  for(int i{0}; i<images.size(); i++)
  {
    frame = cv::imread(images[i]);
    cv::cvtColor(frame,gray,cv::COLOR_BGR2GRAY);

    // Finding checker board corners
    // If desired number of corners are found in the image then success = true  
    success = cv::findChessboardCorners(gray,cv::Size(CHECKERBOARD[0],CHECKERBOARD[1]), corner_pts, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);

    /*
     * If desired number of corner are detected,
     * we refine the pixel coordinates and display 
     * them on the images of checker board
    */
    if(success)
    {
      cv::TermCriteria criteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.001);

      // refining pixel coordinates for given 2d points.
      cv::cornerSubPix(gray,corner_pts,cv::Size(11,11), cv::Size(-1,-1),criteria);

      // Displaying the detected corner points on the checker board
      cv::drawChessboardCorners(frame, cv::Size(CHECKERBOARD[0],CHECKERBOARD[1]), corner_pts,success);

      objpoints.push_back(objp);
      imgpoints.push_back(corner_pts);
    }
    cv::Mat frame_resize;
    cv::resize(frame,frame_resize,frame.size()/4);
    cv::imshow("Image",frame_resize);
    cv::waitKey(0);
  }

  cv::destroyAllWindows();

  cv::Mat cameraMatrix,distCoeffs,R,T;

  /*
   * Performing camera calibration by 
   * passing the value of known 3D points (objpoints)
   * and corresponding pixel coordinates of the 
   * detected corners (imgpoints)
  */
  cv::calibrateCamera(objpoints, imgpoints,cv::Size(gray.rows,gray.cols),cameraMatrix,distCoeffs,R,T);
  //distCoeffs.at<double>(4) = 0.0;

  std::cout << "cameraMatrix : " << cameraMatrix << std::endl;
  std::cout << "distCoeffs : " << distCoeffs << std::endl;
  std::cout << "Rotation vector : " << R << std::endl;
  std::cout << "Translation vector : " << T << std::endl;

  //undistort the 05.jpg
  cv::Mat map1, map2;
  const double alpha = 1;
  cv::Mat NewCameraMatrix = getOptimalNewCameraMatrix(cameraMatrix,distCoeffs,frame.size(),alpha,frame.size(),0);
  initUndistortRectifyMap(cameraMatrix,distCoeffs,cv::Mat(),NewCameraMatrix,frame.size(),CV_16SC2,map1,map2);

  const std::string str = "./images3/";
  for(int i = 0 ; i < 5; i++){
    std::string inputPath = str + "0" + std::to_string(i) + ".jpg";
    cv::Mat rawImage = cv::imread(inputPath);
    cv::Mat image_resize;
    cv::resize(rawImage,image_resize,rawImage.size()/4);
    cv::imshow("RawImage",image_resize);

    cv::Mat UndistortImage;
    remap(rawImage,UndistortImage,map1,map2,cv::INTER_LINEAR);
    cv::Mat image_resize_un;
    cv::resize(UndistortImage,image_resize_un,UndistortImage.size()/4);
    cv::imshow("UndistortImage",image_resize_un);

    std::string Outputpath = str + std::to_string(i) + "_un" + ".jpg";
    cv::imwrite(Outputpath,UndistortImage);
    cv::waitKey(0);
  }
  return 0;
}