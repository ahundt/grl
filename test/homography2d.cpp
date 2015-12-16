#include <stdio.h>
#include <iostream>
#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/opencv.hpp"

#include <opencv2/line_descriptor.hpp>

#include <opencv2/imgproc.hpp>

using namespace cv;
using namespace cv::xfeatures2d;

void readme();

/** @function main */
int main( int argc, char** argv )
{
    
  if( argc != 3 )
  { readme(); return -1; }
    
    
  Mat img_object = imread( argv[1], IMREAD_GRAYSCALE );
  if( !img_object.data )
    { std::cout<< " --(!) Error reading object image " << std::endl; return -1; }
    
  VideoCapture cap(0); // open the default camera
  if(!cap.isOpened())  // check if we succeeded
    return -1;
  
//  Mat edges;
//  namedWindow("edges",1);
  for(;;)
  {
    Mat frame;
    cap >> frame; // get a new frame from camera
    cvtColor(frame, img_scene, COLOR_BGR2GRAY);
    if( !img_object.data || !img_scene.data )
      { std::cout<< " --(!) Error reading scene image " << std::endl; return -1; }
      //-- Step 1: Detect and calculate the keypoints and descriptors using SURF Detector
      std::vector<KeyPoint> keypoints_object, keypoints_scene;
      int surfNFeatures = 1000;
      //-- Note: need OpenCV3 and opencv_contrib to use SurfFeatureDetector
      auto extractor = cv::xfeatures2d::SurfFeatureDetector::create(surfNFeatures, 5, 2);
      
      Mat descriptors_object, descriptors_scene;
      
      extractor->detectAndCompute( img_object, noArray(), keypoints_object, descriptors_object );
      extractor->detectAndCompute( img_scene, noArray(), keypoints_scene, descriptors_scene );
      
      //-- Step 2: Matching descriptor vectors using FLANN matcher
      FlannBasedMatcher matcher;
      std::vector< DMatch > matches;
      matcher.match( descriptors_object, descriptors_scene, matches );
      
      double max_dist = 0; double min_dist = 100;
      
      //-- Quick calculation of max and min distances between keypoints
      for( int i = 0; i < descriptors_object.rows; i++ )
      { double dist = matches[i].distance;
          if( dist < min_dist ) min_dist = dist;
          if( dist > max_dist ) max_dist = dist;
      }
      
      printf("-- Max dist : %f \n", max_dist );
      printf("-- Min dist : %f \n", min_dist );
      
      //-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
      std::vector< DMatch > good_matches;
      
      for( int i = 0; i < descriptors_object.rows; i++ )
      { if( matches[i].distance < 3*min_dist )
          { good_matches.push_back( matches[i]); }
      }
      
      Mat img_matches;
      drawMatches( img_object, keypoints_object, img_scene, keypoints_scene,
                  good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
                  std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
      
      //-- Localize the object
      std::vector<Point2f> obj;
      std::vector<Point2f> scene;
      
      for( int i = 0; i < good_matches.size(); i++ )
      {
          //-- Get the keypoints from the good matches
          obj.push_back( keypoints_object[ good_matches[i].queryIdx ].pt );
          scene.push_back( keypoints_scene[ good_matches[i].trainIdx ].pt );
      }
      
      Mat H = findHomography( obj, scene, RANSAC );
      
      //-- Get the corners from the image_1 ( the object to be "detected" )
      std::vector<Point2f> obj_corners(4);
      obj_corners[0] = cvPoint(0,0); obj_corners[1] = cvPoint( img_object.cols, 0 );
      obj_corners[2] = cvPoint( img_object.cols, img_object.rows ); obj_corners[3] = cvPoint( 0, img_object.rows );
      std::vector<Point2f> scene_corners(4);
      
      perspectiveTransform( obj_corners, scene_corners, H);
      
      //-- Draw lines between the corners (the mapped object in the scene - image_2 )
      line( img_matches, scene_corners[0] + Point2f( img_object.cols, 0), scene_corners[1] + Point2f( img_object.cols, 0), Scalar(0, 255, 0), 4 );
      line( img_matches, scene_corners[1] + Point2f( img_object.cols, 0), scene_corners[2] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );
      line( img_matches, scene_corners[2] + Point2f( img_object.cols, 0), scene_corners[3] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );
      line( img_matches, scene_corners[3] + Point2f( img_object.cols, 0), scene_corners[0] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );
      
      //-- Show detected matches
      imshow( "Good Matches & Object detection", img_matches );
      if(waitKey(30) >= 0) break;
      
//    cvtColor(frame, edges, COLOR_BGR2GRAY);
//    GaussianBlur(edges, edges, Size(7,7), 1.5, 1.5);
//    Canny(edges, edges, 0, 30, 3);
//    imshow("edges", edges);
//    if(waitKey(30) >= 0) break;
  }
  // the camera will be deinitialized automatically in VideoCapture destructor
  return 0;
    
//  Mat img_object = imread( argv[1], IMREAD_GRAYSCALE );
  // Mat img_scene = imread( argv[2], IMREAD_GRAYSCALE );

//  if( !img_object.data || !img_scene.data )
//  { std::cout<< " --(!) Error reading images " << std::endl; return -1; }

//  //-- Step 1: Detect and calculate the keypoints and descriptors using SURF Detector
//  std::vector<KeyPoint> keypoints_object, keypoints_scene;
//  int surfNFeatures = 1000;
//  //-- Note: need OpenCV3 and opencv_contrib to use SurfFeatureDetector
//  auto extractor = cv::xfeatures2d::SurfFeatureDetector::create(surfNFeatures, 5, 2);
//
//  Mat descriptors_object, descriptors_scene;
//
//  extractor->detectAndCompute( img_object, noArray(), keypoints_object, descriptors_object );
//  extractor->detectAndCompute( img_scene, noArray(), keypoints_scene, descriptors_scene );
//
//  //-- Step 2: Matching descriptor vectors using FLANN matcher
//  FlannBasedMatcher matcher;
//  std::vector< DMatch > matches;
//  matcher.match( descriptors_object, descriptors_scene, matches );
//
//  double max_dist = 0; double min_dist = 100;
//
//  //-- Quick calculation of max and min distances between keypoints
//  for( int i = 0; i < descriptors_object.rows; i++ )
//  { double dist = matches[i].distance;
//    if( dist < min_dist ) min_dist = dist;
//    if( dist > max_dist ) max_dist = dist;
//  }
//
//  printf("-- Max dist : %f \n", max_dist );
//  printf("-- Min dist : %f \n", min_dist );
//
//  //-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
//  std::vector< DMatch > good_matches;
//
//  for( int i = 0; i < descriptors_object.rows; i++ )
//  { if( matches[i].distance < 3*min_dist )
//     { good_matches.push_back( matches[i]); }
//  }
//
//  Mat img_matches;
//  drawMatches( img_object, keypoints_object, img_scene, keypoints_scene,
//               good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
//               std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
//
//  //-- Localize the object
//  std::vector<Point2f> obj;
//  std::vector<Point2f> scene;
//
//  for( int i = 0; i < good_matches.size(); i++ )
//  {
//    //-- Get the keypoints from the good matches
//    obj.push_back( keypoints_object[ good_matches[i].queryIdx ].pt );
//    scene.push_back( keypoints_scene[ good_matches[i].trainIdx ].pt );
//  }
//
//  Mat H = findHomography( obj, scene, RANSAC );
//
//  //-- Get the corners from the image_1 ( the object to be "detected" )
//  std::vector<Point2f> obj_corners(4);
//  obj_corners[0] = cvPoint(0,0); obj_corners[1] = cvPoint( img_object.cols, 0 );
//  obj_corners[2] = cvPoint( img_object.cols, img_object.rows ); obj_corners[3] = cvPoint( 0, img_object.rows );
//  std::vector<Point2f> scene_corners(4);
//
//  perspectiveTransform( obj_corners, scene_corners, H);
//
//  //-- Draw lines between the corners (the mapped object in the scene - image_2 )
//  line( img_matches, scene_corners[0] + Point2f( img_object.cols, 0), scene_corners[1] + Point2f( img_object.cols, 0), Scalar(0, 255, 0), 4 );
//  line( img_matches, scene_corners[1] + Point2f( img_object.cols, 0), scene_corners[2] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );
//  line( img_matches, scene_corners[2] + Point2f( img_object.cols, 0), scene_corners[3] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );
//  line( img_matches, scene_corners[3] + Point2f( img_object.cols, 0), scene_corners[0] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );
//
//  //-- Show detected matches
//  imshow( "Good Matches & Object detection", img_matches );
//
//  waitKey(0);
//  return 0;
  }

  /** @function readme */
  void readme()
  { std::cout << " Usage: ./SURF_descriptor <img1> <img2>" << std::endl; }
