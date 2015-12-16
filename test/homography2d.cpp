#include <stdio.h>
#include <iostream>
#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/xfeatures2d.hpp"

#include <opencv2/line_descriptor.hpp>

#include <opencv2/imgproc.hpp>

using namespace cv;
using namespace cv::xfeatures2d;

void readme();

class ImageToPose{
    
    int _surfNFeatures = 1000;
    cv::Mat _img_object;
    cv::Mat _img_scene;
    cv::Mat _descriptors_object;
    cv::Mat _descriptors_scene;
    std::vector<KeyPoint> _keypoints_object;
    std::vector<KeyPoint> _keypoints_scene;
    cv::Mat _img_matches;
    cv::Mat _homography_transform;
    cv::Mat _pose;
    
    public:
    
    cv::Mat videoToFrame(cv::VideoCapture cap){
        cv::Mat video_img;
        cv::Mat frame;
        cap >> video_img; // get a new frame from camera
        cv::cvtColor(video_img, frame, COLOR_BGR2GRAY);
        return frame;
    }
    
    void getObjectDescriptors(cv::Mat img_object){
        //-- Note: need OpenCV3 and opencv_contrib to use SurfFeatureDetector
        img_object.copyTo(_img_object);
        auto extractor = cv::xfeatures2d::SurfFeatureDetector::create(_surfNFeatures, 5, 2);
        extractor->detectAndCompute( _img_object, noArray(), _keypoints_object, _descriptors_object );
    }
    
    void getSceneDescriptors(cv::Mat scene_object){
        scene_object.copyTo(_img_scene);
        auto extractor = cv::xfeatures2d::SurfFeatureDetector::create(_surfNFeatures, 5, 2);
        extractor->detectAndCompute( _img_scene, noArray(), _keypoints_scene, _descriptors_scene );
    }
    
    cv::Mat getHomography(){
        cv::Mat H;
        //-- Step 1: Detect and calculate the keypoints and descriptors using SURF Detector
        
        //-- Step 2: Matching descriptor vectors using FLANN matcher
        FlannBasedMatcher matcher;
        std::vector< DMatch > matches;
        matcher.match( _descriptors_object, _descriptors_scene, matches );
        
        double max_dist = 0; double min_dist = 100;
        
        //-- Quick calculation of max and min distances between keypoints
        for( int i = 0; i < _descriptors_object.rows; i++ )
            {
            double dist = matches[i].distance;
            if( dist < min_dist ) min_dist = dist;
            if( dist > max_dist ) max_dist = dist;
            }
        
        printf("-- Max dist : %f \n", max_dist );
        printf("-- Min dist : %f \n", min_dist );
        
        //-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
        std::vector< DMatch > good_matches;
        
        for( int i = 0; i < _descriptors_object.rows; i++ )
        {
            if( matches[i].distance < 3*min_dist )
                {
                    good_matches.push_back( matches[i]);
                }
        }
        
        drawMatches( _img_object, _keypoints_object, _img_scene, _keypoints_scene,
                    good_matches, _img_matches, Scalar::all(-1), Scalar::all(-1),
                    std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
        
        //-- Localize the object
        std::vector<Point2f> obj;
        std::vector<Point2f> scene;
        
        for( int i = 0; i < good_matches.size(); i++ )
        {
            //-- Get the keypoints from the good matches
            obj.push_back( _keypoints_object[ good_matches[i].queryIdx ].pt );
            scene.push_back( _keypoints_scene[ good_matches[i].trainIdx ].pt );
        }

        _homography_transform = findHomography( obj, scene, RANSAC );
        _homography_transform.copyTo(H);
        return H;
    }
    
    cv::Mat cameraPoseFromHomography(cv::Mat H)
    {
        //std::cout << "H = "<< std::endl << " "  << H << std::endl << std::endl;
    
        _pose = Mat::eye(3, 4, CV_32FC1);      // 3x4 matrix, the camera pose
        float norm1 = (float)norm(H.col(0));
        float norm2 = (float)norm(H.col(1));
        float tnorm = (norm1 + norm2) / 2.0f; // Normalization value
    
        Mat p1 = H.col(0);       // Pointer to first column of H
                             //std::cout << "p1 = "<< std::endl << " "  << p1 << std::endl << std::endl;
        Mat p2 = _pose.col(0);    // Pointer to first column of pose (empty)
                             //std::cout << "p2 = "<< std::endl << " "  << p2 << std::endl << std::endl;
    
        cv::normalize(p1, p2);   // Normalize the rotation, and copies the column to pose
        p2.copyTo(_pose.col(0));
        //std::cout << "p1 = "<< std::endl << " "  << p1 << std::endl << std::endl;
        //std::cout << "pose = "<< std::endl << " "  << pose << std::endl << std::endl;
    
        p1 = H.col(1);           // Pointer to second column of H
        p2 = _pose.col(1);        // Pointer to second column of pose (empty)
    
        cv::normalize(p1, p2);   // Normalize the rotation and copies the column to pose
        p2.copyTo(_pose.col(1));
    
        p1 = _pose.col(0);
        p2 = _pose.col(1);
    
        Mat p3 = p1.cross(p2);   // Computes the cross-product of p1 and p2
                             //Mat c2 = pose.col(2);    // Pointer to third column of pose
        p3.copyTo(_pose.col(2));       // Third column is the crossproduct of columns one and two
    
        std::cout << "H.col(2) = "<< std::endl << " "  << H.col(2) << std::endl << std::endl;
        std::cout << "tnorm = "<< std::endl << " "  << tnorm << std::endl << std::endl;
        p3 = H.col(2) / tnorm;  //vector t [R|t] is the last column of pose
        (p3).copyTo(_pose.col(3));
        std::cout << "pose = "<< std::endl << " "  << _pose << std::endl << std::endl;
    
        cv::Mat pose;
        _pose.copyTo(pose);
        return pose;
    }
    
    cv::Mat draw2dMatchesToImage(){
        //-- Get the corners from the image_1 ( the object to be "detected" )
        std::vector<Point2f> obj_corners(4);
        obj_corners[0] = cvPoint(0,0); obj_corners[1] = cvPoint( _img_object.cols, 0 );
        obj_corners[2] = cvPoint( _img_object.cols, _img_object.rows ); obj_corners[3] = cvPoint( 0, _img_object.rows );
        std::vector<Point2f> scene_corners(4);
        
        perspectiveTransform( obj_corners, scene_corners, _homography_transform);
    
        //-- Draw lines between the corners (the mapped object in the scene - image_2 )
        line( _img_matches, scene_corners[0] + Point2f( _img_object.cols, 0), scene_corners[1] + Point2f( _img_object.cols, 0), Scalar(0, 255, 0), 4 );
        line( _img_matches, scene_corners[1] + Point2f( _img_object.cols, 0), scene_corners[2] + Point2f( _img_object.cols, 0), Scalar( 0, 255, 0), 4 );
        line( _img_matches, scene_corners[2] + Point2f( _img_object.cols, 0), scene_corners[3] + Point2f( _img_object.cols, 0), Scalar( 0, 255, 0), 4 );
        line( _img_matches, scene_corners[3] + Point2f( _img_object.cols, 0), scene_corners[0] + Point2f( _img_object.cols, 0), Scalar( 0, 255, 0), 4 );
        cv::Mat img_matches;
        _img_matches.copyTo(img_matches);
        return img_matches;
    }
    
    };
    
    
    /** @function main */
    int main( int argc, char** argv )
    {
    
    if( argc != 2 )
        { readme(); return -1; }
    
    // Create ImageToPose object
    ImageToPose ImageToPoseObject;
    
    // Read in command line for file path for the object image to be found in the scene
    Mat img_object = imread( argv[1], IMREAD_GRAYSCALE );
    if( !img_object.data )
        { std::cout<< " --(!) Error reading object image " << std::endl; return -1; }
    // Get the descriptors using SIFT/SURF in the object image
    ImageToPoseObject.getObjectDescriptors(img_object);
    
    // Gets an opencv video capture using a standard webcam. You can get the image scene from somewhere else
    Mat img_scene;
    cv::VideoCapture cap(0); // open the default camera
    if(!cap.isOpened())  // check if we succeeded
        return -1;

    // Loop to continually get scene images
    for(;;)
    {
        // Function to convert opencv video capture to scene image
        cv::Mat scene_object = ImageToPoseObject.videoToFrame(cap);
    
        // Get the descriptors using SIFT/SURF in the scene image
        ImageToPoseObject.getSceneDescriptors(scene_object);
        // Get the homography between the scene and object image
        cv::Mat H = ImageToPoseObject.getHomography();
        if(!H.cols || !H.rows) continue; //Could not find match
        // Get the pose from the homography
        cv::Mat pose = ImageToPoseObject.cameraPoseFromHomography(H);
        // Get the image matches between the object and scene
        cv::Mat img_matches = ImageToPoseObject.draw2dMatchesToImage();
    
        //-- Show detected matches
        imshow( "Good Matches & Object detection", img_matches );
        if(waitKey(30) >= 0) continue;
    }
    
    }
    
    /** @function readme */
    void readme()
    { std::cout << " Usage: ./SURF_descriptor <img1> <img2>" << std::endl; }
