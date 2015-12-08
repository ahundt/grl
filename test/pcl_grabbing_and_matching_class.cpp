// #include "Kinect2Grabber.hpp" 
#include <chrono>
#include "k2g.h" // Might throw an error, as it's in src and not include. Placed it there though because it has several function definitions, while declarations are placed in include.
#include <pcl/visualization/cloud_viewer.h>

// extra headers for writing out ply file
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/conversions.h>
#include <pcl/io/ply_io.h>
#include <pcl/PCLPointCloud2.h>

#include "opencv2/surface_matching.hpp"
#include <iostream>
#include "opencv2/surface_matching/ppf_helpers.hpp"
#include "opencv2/core/utility.hpp"

using namespace std;
using namespace cv;
using namespace ppf_match_3d;

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;   
    
class SurfaceMatching
{
public:

    template<typename Cloud>
    void
    saveCloud (const std::string &filename, const Cloud &cloud, bool binary, bool use_camera)
    {
      TicToc tt;
      tt.tic ();

      print_highlight ("Saving "); print_value ("%s ", filename.c_str ());
      
      pcl::PLYWriter writer;
      writer.write (filename, cloud, Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), binary, use_camera);
      
      print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", cloud.width * cloud.height); print_info (" points]\n");
    }


    cv::Mat pc, pcTest, pct;
    ppf_match_3d::PPF3DDetector detector;
    vector<Pose3DPtr> results;
    Pose3DPtr result;
    Pose3DPtr result_0;
    enum ParamIndex {
        modelFileName,
        processor,
        outputFileName,        
        sampling_step_relative,
        distance_step_relative
    };
    typedef std::tuple<
        std::string,
        enum processor, // processor freenectprocessor = OPENGL; //enumerated object in k2g.h
        std::string,
        float,
        float
        > Params;

    Params params;
    
    SurfaceMatching() // Is this default
    {
        params = std::make_tuple(
            "parasaurolophus_low_normals2",  //modelFileName
            OPENGL,
            "output.ply",
            0.025,                            //sampling_step_relative
            0.05                             //distance_step_relative
        );
    }
    SurfaceMatching(std::string modelFilename, enum processor freenectprocessor, std::string outputFilename, float sampling_step_relative, float distance_step_relative)
    {   
        params = std::make_tuple(
            modelFileName,
            freenectprocessor,
            outputFileName,
            sampling_step_relative,
            distance_step_relative
        );
    }

// Skipping the error message static void help(const string& errorMessage)
    
    void train_model()
    {
//        string modelFileName = (string)argv[1]; // Might need to typecast
        pc = loadPLYSimple(std::get<modelFileName>(params).c_str(), 1); //c_str necessary?
        // Now train the model
        cout << "Training..." << endl;
        int64 tick1 = cv::getTickCount();
        ppf_match_3d::PPF3DDetector detector_in_function(std::get<sampling_step_relative>(params), std::get<distance_step_relative>(params)); //////////////////
        detector_in_function.trainModel(pc);
        detector = detector_in_function; // Very hacky. TODO.
        int64 tick2 = cv::getTickCount();
        cout << endl << "Training complete in "
             << (double)(tick2-tick1)/ cv::getTickFrequency()
             << " sec" << endl << "Loading model..." << endl;
    }
    
    void save_model()
    {
        // TODO SAVE IT TO FILE!! - Not implemented yet
    }

    void matching(enum processor freenectprocessor)
    {
       // The below code reads in the scene model from the kinect and performs matching.
        boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud;
        K2G k2g(freenectprocessor);
        std::cout << "getting cloud" << std::endl;
        cloud = k2g.getCloud();

        cloud->sensor_orientation_.w() = 0.0;
        cloud->sensor_orientation_.x() = 1.0;
        cloud->sensor_orientation_.y() = 0.0;
        cloud->sensor_orientation_.z() = 0.0;

        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
        viewer->setBackgroundColor (0, 0, 0);
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
        viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud"); 

        bool done = false;
        while ((!viewer->wasStopped()) && (!done)) {
            viewer->spinOnce ();
            using namespace std::chrono;
            static high_resolution_clock::time_point last;

            auto tnow = high_resolution_clock::now();
            cloud = k2g.updateCloud(cloud);
            auto tpost = high_resolution_clock::now();
            std::cout << "delta " << duration_cast<duration<double>>(tpost-tnow).count()*1000 << std::endl;
            pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
            viewer->updatePointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");

//            if(ply_file_indices.size() > 1 ){ //changed //Removing this check. Can't access it, we're assuming we have valid input. Can put in a check later.
                pcl::PCLPointCloud2 cloud2;
                pcl::toPCLPointCloud2(*cloud,cloud2);
                saveCloud(std::get<outputFileName>(params).c_str(),cloud2,false,false); //changed // Might not want to save scene to disk to speed it up.
                done = true;
//            }
        }
        k2g.shutDown();

        // http://stackoverflow.com/questions/32521043/how-to-convert-cvmat-to-pclpointcloud  
        cv::Mat pcTest_in_function(3, cloud->points.size(), CV_64FC1);
        for(int i=0; i < cloud->points.size();i++){
            pcTest_in_function.at<double>(0,i) = cloud->points.at(i).x; // look at getFullCloud and createFullCloud for structure. Think this is valid. TODO Not messing with rgb data yet!
            pcTest_in_function.at<double>(1,i) = cloud->points.at(i).y;
            pcTest_in_function.at<double>(2,i) = cloud->points.at(i).z;
        }
        pcTest = pcTest_in_function; // Hacky! TODO

       // Match the model to the scene and get the pose
        cout << endl << "Starting matching..." << endl;
        int tick1 = cv::getTickCount();
        detector.match(pcTest, results, std::get<sampling_step_relative>(params), std::get<distance_step_relative>(params));
        int tick2 = cv::getTickCount();
        cout << endl << "PPF Elapsed Time " <<
             (tick2-tick1)/cv::getTickFrequency() << " sec" << endl;

        //check results size from match call above
        size_t results_size = results.size();
        cout << "Number of matching poses: " << results_size;
        if (results_size == 0) {
            cout << endl << "No matching poses found. Exiting." << endl;
            exit(0);
        }

        // Get only first N results - but adjust to results size if num of results are less than that specified by N
        size_t N = 2;
        if (results_size < N) {
            cout << endl << "Reducing matching poses to be reported (as specified in code): "
                 << N << " to the number of matches found: " << results_size << endl;
            N = results_size;
        }
        vector<Pose3DPtr> resultsSub(results.begin(),results.begin()+N);

        // Create an instance of ICP
        ICP icp(100, 0.005f, 2.5f, 8);
        int64 t1 = cv::getTickCount();

        // Register for all selected poses
        cout << endl << "Performing ICP on " << N << " poses..." << endl;
        icp.registerModelToScene(pc, pcTest, resultsSub);
        int64 t2 = cv::getTickCount();

        cout << endl << "ICP Elapsed Time " <<
             (t2-t1)/cv::getTickFrequency() << " sec" << endl;
         
    cout << "Poses: " << endl;
    // debug first five poses
    for (size_t i=0; i<resultsSub.size(); i++)
    {
        result = resultsSub[i];
        cout << "Pose Result " << i << endl;
        result->printPose();
        if (i==0)
        {
            result_0 = resultsSub[0]; // to make it accessible
            pct = transformPCPose(pc, result->pose);
//          TODO Display it live
/*
            pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
            for(int i=0;i<pct.cols;i++)
            {
                pcl::PointXYZ point;
                point.x = pct.at<float>(0,i);
                point.y = pct.at<float>(1,i);
                point.z = pct.at<float>(2,i);
                // TODO when color needs to be added: found this starter code
                //uint32_t rgb = (static_cast<uint32_t>(pr) << 16 | static_cast<uint32_t>(pg) << 8 | static_cast<uint32_t>(pb));
                //point.rgb = *reinterpret_cast<float*>(&rgb);
                point_cloud_ptr -> points.push_back(point);
            }
            point_cloud_ptr->width = (int)point_cloud_ptr->points.size();
            point_cloud_ptr->height = 1;
*/
            writePLY(pct, "para6700PCTrans.ply"); // Might want to not write out, as writing to disk is forever. Or so I hear :).
        }
    }

    }
                
};


