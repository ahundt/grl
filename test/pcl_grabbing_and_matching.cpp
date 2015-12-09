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


bool stop = false;
void sigint_handler(int s)
{
	stop = true;
}


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

//static void help(const string& errorMessage)
//{
//    cout << "Program init error : "<< errorMessage << endl;
//    cout << "\nUsage : ppf_matching [input model file] "<< endl; // modified it to remove input scene file
//    cout << "\nPlease start again with new parameters"<< endl;
//}

int main(int argc, char** argv)
{
    std::cout << "Syntax is: " << argv[0] << " [inputObjectModel.ply] [-processor 0|1|2] [output.ply] [-sampling_step_relative (0.025 default)] [-distance_step_relative (0.05 default)] \n -processor options 0,1,2 correspond to CPU, OPENCL, and OPENGL respectively\n"; // because I changed this, I had to change indices being accessed in places below.
    processor freenectprocessor = OPENGL; // enumerated object defined in k2g.h
    float sampling_step_relative, distance_step_relative;
    std::vector<int> ply_file_indices;
    if(argc>1){
        int fnpInt;
        float argv_param;
        ply_file_indices = parse_file_extension_argument (argc, argv, ".ply");
        parse_argument (argc, argv, "-processor", fnpInt);
        freenectprocessor = static_cast<processor>(fnpInt);
        parse_argument (argc, argv, "-sampling_step_relative", argv_param);
        sampling_step_relative = static_cast<float>(argv_param); // Not actually using these two here
        parse_argument (argc, argv, "-distance_step_relative", argv_param);
        distance_step_relative = static_cast<float>(argv_param);                                
    }
//    if (argc < 2) // Changed from 3 to 2
//    {
//        help("Not enough input arguments");
//        exit(1);
//    }
//#if (defined __x86_64__ || defined _M_X64)
//    cout << "Running on 64 bits" << endl;
//#else
//    cout << "Running on 32 bits" << endl;
//#endif
//    
//#ifdef _OPENMP
//    cout << "Running with OpenMP" << endl;
//#else
//    cout << "Running without OpenMP and without TBB" << endl;
//#endif

    string modelFileName = std::string(argv[ply_file_indices[0]]);
    Mat pc = loadPLYSimple(modelFileName.c_str(), 1);

    // Now train the model
    cout << "Training..." << endl;
    int64 tick1 = cv::getTickCount();
    ppf_match_3d::PPF3DDetector detector(0.025, 0.05);
    detector.trainModel(pc);
    int64 tick2 = cv::getTickCount();
    cout << endl << "Training complete in "
         << (double)(tick2-tick1)/ cv::getTickFrequency()
         << " sec" << endl << "Loading model..." << endl;

    // TODO SAVE IT TO FILE!! - Not implemented yet

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
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud"); // If it outputs a sample cloud, need to suppress it. Not sure which lines of code correspond specifically to it, so leaving it in.

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

        if(ply_file_indices.size() > 1 ){ //changed
            pcl::PCLPointCloud2 cloud2;
            pcl::toPCLPointCloud2(*cloud,cloud2);
            saveCloud(std::string(argv[ply_file_indices[1]]),cloud2,false,false); //changed
            done = true;
        } //TODO
    }
    k2g.shutDown();

    // http://stackoverflow.com/questions/32521043/how-to-convert-cvmat-to-pclpointcloud  
    cv::Mat pcTest(3, cloud->points.size(), CV_64FC1);
    for(int i=0; i < cloud->points.size();i++){
        pcTest.at<double>(0,i) = cloud->points.at(i).x; // look at getFullCloud and createFullCloud for structure. Think this is valid. TODO Not messing with rgb data yet!
        pcTest.at<double>(1,i) = cloud->points.at(i).y;
        pcTest.at<double>(2,i) = cloud->points.at(i).z;
    }

   // Match the model to the scene and get the pose
    cout << endl << "Starting matching..." << endl;
    vector<Pose3DPtr> results;
    tick1 = cv::getTickCount();
    detector.match(pcTest, results, 1.0/40.0, 0.05);
    tick2 = cv::getTickCount();
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
        Pose3DPtr result = resultsSub[i];
        cout << "Pose Result " << i << endl;
        result->printPose();
        if (i==0)
        {
            Mat pct = transformPCPose(pc, result->pose);
//          TODO Display it live
/*
            pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
            for(int i=0;i<pct.cols;i++)
            {
                pcl::PointXYZ point;
                point.x = pct.at<float>(0,i);
                point.y = pct.at<float>(1,i);
                point.z = pct.at<float>(2,i);
                // TODO when color needs to be added:
                //uint32_t rgb = (static_cast<uint32_t>(pr) << 16 | static_cast<uint32_t>(pg) << 8 | static_cast<uint32_t>(pb));
                //point.rgb = *reinterpret_cast<float*>(&rgb);
                point_cloud_ptr -> points.push_back(point);
            }
            point_cloud_ptr->width = (int)point_cloud_ptr->points.size();
            point_cloud_ptr->height = 1;
*/
            writePLY(pct, "para6700PCTrans.ply");
        }
    }
    
    return 0;
}
