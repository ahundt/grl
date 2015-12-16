#include <stdio.h>
#include <iostream>
#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/xfeatures2d.hpp"

#include <opencv2/line_descriptor.hpp>

#include <opencv2/imgproc.hpp>


// extra headers for writing out ply file
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/conversions.h>
#include <pcl/common/io.h>
#include <pcl/io/ply_io.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/recognition/ransac_based/obj_rec_ransac.h>
#include <pcl/registration/transforms.h>
#include <pcl/features/integral_image_normal.h>

#include "k2g.h"


void readme();

class ImageToPose{

    enum ParamsIndex {
       nFeaturesSURF,
       nOctavesSURF,
       nOctaveLayersSURF,
       minKeypointDist,
       maxKeypointDist
    };
    
    typedef std::tuple<
       int,
       int,
       int
    > Params;
    
    Params  _params;
    cv::Mat _img_object;
    cv::Mat _img_scene;
    cv::Mat _descriptors_object;
    cv::Mat _descriptors_scene;
    std::vector<cv::KeyPoint> _keypoints_object;
    std::vector<cv::KeyPoint> _keypoints_scene;
    cv::Mat _img_matches;
    cv::Mat _homography_transform;
    cv::Mat _pose;
    cv::Ptr<cv::Feature2D> _detector;
public:
    static Params defaultParams(){
       return std::make_tuple(
           1000,   // nFeaturesSURF
           5,      // nOctavesSURF
           2       // nOctaveLayersSURF
       );
    }
    
    ImageToPose(Params params = defaultParams()):
        _params(params)
        ,_detector(cv::xfeatures2d::SurfFeatureDetector::create(
            std::get<nFeaturesSURF>(params)
            ,std::get<nOctavesSURF>(params)
            ,std::get<nOctaveLayersSURF>(params)
         ))
    {
    }
    
    
    void getObjectDescriptors(cv::Mat img_object){
        //-- Note: need OpenCV3 and opencv_contrib to use SurfFeatureDetector
        img_object.copyTo(_img_object);
        _detector->detectAndCompute( _img_object, cv::noArray(), _keypoints_object, _descriptors_object );
    }
    
    void getSceneDescriptors(cv::Mat scene_object){
        scene_object.copyTo(_img_scene);
        _detector->detectAndCompute( _img_scene, cv::noArray(), _keypoints_scene, _descriptors_scene );
    }
    
    cv::Mat getHomography(bool debug = true){
        cv::Mat H;
        //-- Step 1: Detect and calculate the keypoints and descriptors using SURF Detector
        
        //-- Step 2: Matching descriptor vectors using FLANN matcher
        cv::FlannBasedMatcher matcher;
        std::vector< cv::DMatch > matches;
        matcher.match( _descriptors_object, _descriptors_scene, matches );
        
        // these are updated in the next loop
        double max_dist = 0, min_dist = 100;
        
        //-- Quick calculation of max and min distances between keypoints
        for( int i = 0; i < _descriptors_object.rows; i++ )
        {
            double dist = matches[i].distance;
            if( dist < min_dist ) min_dist = dist;
            if( dist > max_dist ) max_dist = dist;
        }
        
        if (debug) {
            printf("-- Max dist : %f \n", max_dist );
            printf("-- Min dist : %f \n", min_dist );
        }
        
        //-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
        std::vector< cv::DMatch > good_matches;
        
        for( int i = 0; i < _descriptors_object.rows; i++ )
        {
            if( matches[i].distance < 3*min_dist )
                {
                    good_matches.push_back( matches[i]);
                }
        }
        
        if (debug) {
          drawMatches( _img_object, _keypoints_object, _img_scene, _keypoints_scene,
                      good_matches, _img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
                      std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
        }
        
        //-- Localize the object
        std::vector<cv::Point2f> obj;
        std::vector<cv::Point2f> scene;
        
        for( std::size_t i = 0; i < good_matches.size(); i++ )
        {
            //-- Get the keypoints from the good matches
            obj.push_back( _keypoints_object[ good_matches[i].queryIdx ].pt );
            scene.push_back( _keypoints_scene[ good_matches[i].trainIdx ].pt );
        }

        _homography_transform = findHomography( obj, scene, cv::RANSAC );
        _homography_transform.copyTo(H);
        return H;
    }
    
    cv::Mat cameraPoseFromHomography(cv::Mat H)
    {
        //std::cout << "H = "<< std::endl << " "  << H << std::endl << std::endl;
    
        _pose = cv::Mat::eye(3, 4, CV_32FC1);      // 3x4 matrix, the camera pose
        float norm1 = (float)norm(H.col(0));
        float norm2 = (float)norm(H.col(1));
        float tnorm = (norm1 + norm2) / 2.0f; // Normalization value
    
        cv::Mat p1 = H.col(0);       // Pointer to first column of H
                             //std::cout << "p1 = "<< std::endl << " "  << p1 << std::endl << std::endl;
        cv::Mat p2 = _pose.col(0);    // Pointer to first column of pose (empty)
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
    
        cv::Mat p3 = p1.cross(p2);   // Computes the cross-product of p1 and p2
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
        std::vector<cv::Point2f> obj_corners(4);
        obj_corners[0] = cvPoint(0,0); obj_corners[1] = cvPoint( _img_object.cols, 0 );
        obj_corners[2] = cvPoint( _img_object.cols, _img_object.rows ); obj_corners[3] = cvPoint( 0, _img_object.rows );
        std::vector<cv::Point2f> scene_corners(4);
        
        perspectiveTransform( obj_corners, scene_corners, _homography_transform);
    
        //-- Draw lines between the corners (the mapped object in the scene - image_2 )
        line( _img_matches, scene_corners[0] + cv::Point2f( _img_object.cols, 0), scene_corners[1] + cv::Point2f( _img_object.cols, 0), cv::Scalar(0, 255, 0), 4 );
        line( _img_matches, scene_corners[1] + cv::Point2f( _img_object.cols, 0), scene_corners[2] + cv::Point2f( _img_object.cols, 0), cv::Scalar( 0, 255, 0), 4 );
        line( _img_matches, scene_corners[2] + cv::Point2f( _img_object.cols, 0), scene_corners[3] + cv::Point2f( _img_object.cols, 0), cv::Scalar( 0, 255, 0), 4 );
        line( _img_matches, scene_corners[3] + cv::Point2f( _img_object.cols, 0), scene_corners[0] + cv::Point2f( _img_object.cols, 0), cv::Scalar( 0, 255, 0), 4 );
        cv::Mat img_matches;
        _img_matches.copyTo(img_matches);
        return img_matches;
    }
    
    };

    
    
    cv::Mat videoToFrame(cv::VideoCapture cap){
        cv::Mat video_img;
        cv::Mat frame;
        cap >> video_img; // get a new frame from camera
        cv::cvtColor(video_img, frame, cv::COLOR_BGR2GRAY);
        return frame;
    }
    
    /** @function main */
    int main( int argc, char** argv )
    {
      std::cout << "Syntax is: " << argv[0] << " [--processor 0|1|2] [model.ply] [--showmodels]\n --processor options 0,1,2 correspond to CPU, OPENCL, and OPENGL respectively\n";
      processor freenectprocessor = OPENGL;
      std::vector<int> ply_file_indices;
      
      /// http://docs.pointclouds.org/trunk/classpcl_1_1recognition_1_1_obj_rec_r_a_n_s_a_c.html#ae1a4249f8278de41a34f74b950996986
      float pair_width = 0.15;
      float voxel_size = 0.01;
      bool showmodels = false;
        
      
      if(argc>1){
          int fnpInt;
          ply_file_indices = pcl::console::parse_file_extension_argument (argc, argv, ".ply");
          pcl::console::parse_argument (argc, argv, "--processor", fnpInt);
          pcl::console::parse_argument (argc, argv, "--pair_width", pair_width);
          pcl::console::parse_argument (argc, argv, "--voxel_size", voxel_size);
          showmodels = pcl::console::find_switch(argc,argv,"--showmodels");
          freenectprocessor = static_cast<processor>(fnpInt);
          
      }
    
    if( argc != 2 )
        { readme(); return -1; }
    
    // Create ImageToPose object
    ImageToPose ImageToPoseObject;
    
    // Read in command line for file path for the object image to be found in the scene
    cv::Mat img_object = cv::imread( argv[1], cv::IMREAD_GRAYSCALE );
    if( !img_object.data )
        { std::cout<< " --(!) Error reading object image " << std::endl; return -1; }
    // Get the descriptors using SIFT/SURF in the object image
    ImageToPoseObject.getObjectDescriptors(img_object);
    
    // Gets an opencv video capture using a standard webcam. You can get the image scene from somewhere else
    cv::Mat img_scene;
    cv::VideoCapture cap(0); // open the default camera
    if(!cap.isOpened())  // check if we succeeded
        return -1;

    // Loop to continually get scene images
    for(;;)
    {
        // Function to convert opencv video capture to scene image
        cv::Mat scene_object = videoToFrame(cap);
    
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
        cv::imshow( "Good Matches & Object detection", img_matches );
        if(cv::waitKey(30) >= 0) continue;
    }
    
    }
    
    /** @function readme */
    void readme()
    { std::cout << " Usage: ./SURF_descriptor <img1> <img2>" << std::endl; }
