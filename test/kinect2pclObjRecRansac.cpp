/*
Copyright 2015, Giacomo Dabisias"


Dual licensed with permission under GPLv3 or later and the 2 clause simplified BSD.
https://github.com/giacomodabisias/libfreenect2pclgrabber/issues/10

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
@Author 
Giacomo. Dabisias, PhD Student
PERCRO, (Laboratory of Perceptual Robotics)
Scuola Superiore Sant'Anna
via Luigi Alamanni 13D, San Giuliano Terme 56010 (PI), Italy
*/
#include "k2g.h"
#include <pcl/visualization/cloud_viewer.h>
#include <chrono>

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


#include "opencv2/surface_matching.hpp"
#include <iostream>
#include "opencv2/core/utility.hpp"

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

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



  /**
   * \breif convert an opencv collection of points to a pcl::PoinCloud, your opencv mat should have NAN's for invalid points.
   * @param points3d opencv matrix of nx1 3 channel points
   * @param cloud output cloud
   * @param rgb the rgb, required, will color points
   * @param mask the mask, required, must be same size as rgb
   */
  inline void
  cvToCloudXYZRGBNormal(const cv::Mat& cvcloud, pcl::PointCloud<pcl::PointXYZRGBNormal>& cloud)
  {
    std::size_t rows = cvcloud.rows;
    std::size_t cols = cvcloud.cols;
    cloud.clear();
  
    for (std::size_t i = 0; i<rows; ++i) {
      
      pcl::PointXYZRGBNormal cp;
      cp.x = cvcloud.at<float>(i,0);
      cp.x = cvcloud.at<float>(i,1);
      cp.x = cvcloud.at<float>(i,2);
      if (cols > 3)
      {
        cp.normal_x = cvcloud.at<float>(i,3);
        cp.normal_y = cvcloud.at<float>(i,4);
        cp.normal_z = cvcloud.at<float>(i,5);
      }
      if (cols > 6)
      {
        cp.r = cvcloud.at<float>(i,6);
        cp.g = cvcloud.at<float>(i,7);
        cp.b = cvcloud.at<float>(i,8);
      }
      cloud.push_back(cp);
    }
  }


std::vector<std::string> split(const std::string &text, char sep);

std::vector<std::string> split(const std::string &text, char sep) {
  std::vector<std::string> tokens;
  std::size_t start = 0, end = 0;
  while ((end = text.find(sep, start)) != std::string::npos) {
    tokens.push_back(text.substr(start, end - start));
    start = end + 1;
  }
  tokens.push_back(text.substr(start));
  return tokens;
}

cv::Mat loadPLYSimple(const char* fileName, int withNormals = 0)
{
  cv::Mat cloud;
  int numVertices=0;
  int numCols=3;
  bool with_color = false;
  bool with_alpha = false; // alpha transparency

  std::ifstream ifs(fileName);

  if (!ifs.is_open())
  {
    printf("Cannot open file...\n");
    return cv::Mat();
  }

  std::string str;
  while (str.substr(0, 10) !="end_header")
  {
    std::vector<std::string> tokens = split(str,' ');
    if (tokens.size() == 3 && tokens[0] == "element" && tokens[1] == "vertex")
    {
      numVertices = atoi(tokens[2].c_str());
    }
    if (tokens.size() ==3 && tokens[0] == "property")
    {
      if(tokens[3]=="nx" || tokens[3]=="normal_x" )
      {
        withNormals = true;
        numCols+=3;
      }
      else if(tokens[3]=="r" || tokens[3]=="red" )
      {
        with_color = true;
        numCols+=3;
      }
      else if(tokens[3]=="a" || tokens[3]=="alpha" )
      {
        with_alpha = true;
        numCols+=1;
      }
    }
    else if (tokens.size() > 1 && tokens[0] == "format")
    {
      if (tokens[1]!="ascii"){
        printf("Cannot read file, only ascii ply format is currently supported...\n");
        // uncomment below when CV_StsBadArg can be located
        //OPENCV_ERROR (CV_StsBadArg, "loadPLYSimple", "Cannot read file, only ascii ply format is currently supported...");
        return cv::Mat();
      }
    }
    std::getline(ifs, str);
  }

  cloud=cv::Mat(numVertices, numCols, CV_32FC1);

  for (int i = 0; i < numVertices; i++)
  {
    float* data = (float*)(&cloud.data[i*cloud.step[0]]);
    for (int col = 0; col < numCols; ++col)
    {
      ifs >> data[col];
    }
  
    if (withNormals)
    {

      // normalize to unit norm
      double norm = sqrt(data[3]*data[3] + data[4]*data[4] + data[5]*data[5]);
      if (norm>0.00001)
      {
        data[3]/=static_cast<float>(norm);
        data[4]/=static_cast<float>(norm);
        data[5]/=static_cast<float>(norm);
      }
    }
  }

  //cloud *= 5.0f;
  return cloud;
}

pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr loadPLYSimpleCloud(const char* fileName, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr model)
{
   cv::Mat cvcloud = loadPLYSimple(fileName);
   cvToCloudXYZRGBNormal(cvcloud,*model);
   return model;
}


bool parse (const std::string& filename)
{
  std::ifstream istream (filename.c_str (), std::ios::in | std::ios::binary);

  std::string line;
  std::size_t line_number_ = 0;

  std::size_t number_of_format_statements = 0; 
  std::size_t number_of_element_statements = 0; 
  std::size_t number_of_property_statements = 0; 
  std::size_t number_of_obj_info_statements = 0; 
  std::size_t number_of_comment_statements = 0;

  pcl::io::ply::format_type format = pcl::io::ply::unknown;
  //std::vector< boost::shared_ptr<pcl::io::ply::element> > elements;

  char line_delim = '\n';
  int char_ignore_count = 0;

  // magic
  char magic[4];
  istream.read (magic, 4);

  // Check if CR/LF, setup delim and char skip
  if (magic[3] == '\r')
  {
    istream.ignore (1);
    line_delim = '\r';
    char_ignore_count = 1;
  }

  ++line_number_;
  if (!istream)
  {
    //if (error_callback_)
    //  error_callback_ (line_number_, "parse error: couldn't read the magic string");
    return false;
  }

  if ((magic[0] != 'p') || (magic[1] != 'l') || (magic[2] != 'y'))
  {
    //if (error_callback_)
    //  error_callback_ (line_number_, "parse error: wrong magic string");
    return false;
  }
  return true;
}

int main(int argc, char *argv[])
{
  std::cout << "Syntax is: " << argv[0] << " [--processor 0|1|2] [model.ply]\n --processor options 0,1,2 correspond to CPU, OPENCL, and OPENGL respectively\n";
  processor freenectprocessor = OPENGL;
  std::vector<int> ply_file_indices;
  
  /// http://docs.pointclouds.org/trunk/classpcl_1_1recognition_1_1_obj_rec_r_a_n_s_a_c.html#ae1a4249f8278de41a34f74b950996986
  float pair_width = 0.15;
  float voxel_size = 0.01;
  
  if(argc>1){
      int fnpInt;
      ply_file_indices = parse_file_extension_argument (argc, argv, ".ply");
      parse_argument (argc, argv, "--processor", fnpInt);
      parse_argument (argc, argv, "--pair_width", pair_width);
      parse_argument (argc, argv, "--voxel_size", voxel_size);
      freenectprocessor = static_cast<processor>(fnpInt);
      
  }
  
  
  // setup ransac
  pcl::recognition::ObjRecRANSAC orransac(pair_width,voxel_size);
  pcl::PLYReader reader;

  //Map from the file name to the point cloud
  std::map<std::string, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> pclMap;

  for (int idx : ply_file_indices) {
      pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr model(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
      // these are the point clouds of the ply files read in
      pcl::PointCloud<Normal>::Ptr modelnormal(new pcl::PointCloud<pcl::Normal>());
      pcl::PointCloud<pcl::PointXYZ>::Ptr modelxyz(new pcl::PointCloud<pcl::PointXYZ>());
      
      std::string modelFile(argv[idx]);

      bool success = parse(modelFile);
      std::cout << "success: " << success << "\n";
      reader.read(modelFile,*model);
      //loadPLYSimpleCloud(modelFile.c_str(),model);
      pcl::copyPointCloud(*model,*modelnormal);
      pcl::copyPointCloud(*model,*modelxyz);
      
      /// @todo should const_cast really be used?
      orransac.addModel(*modelxyz,*modelnormal,modelFile);
      

      //we need to map these strings, which are the names, to the ply files
      pclMap[modelFile] = model;
  }
  
    // estimate normals http://pointclouds.org/documentation/tutorials/normal_estimation_using_integral_images.php#normal-estimation-using-integral-images
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>());

  // setup kinect
  //changed cloud type to ::Ptr
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr kinect_cloud;
  K2G k2g(freenectprocessor);
  std::cout << "getting cloud" << std::endl;
  kinect_cloud = k2g.getCloud();

  kinect_cloud->sensor_orientation_.w() = 0.0;
  kinect_cloud->sensor_orientation_.x() = 1.0;
  kinect_cloud->sensor_orientation_.y() = 0.0;
  kinect_cloud->sensor_orientation_.z() = 0.0;

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
    kinect_cloud = k2g.updateCloud(kinect_cloud);
    auto tpost = high_resolution_clock::now();
    std::cout << "delta " << duration_cast<duration<double>>(tpost-tnow).count()*1000 << std::endl;
    
    

    pcl::IntegralImageNormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
    /// @todo make magic numbers into params
    ne.setMaxDepthChangeFactor(1.0f);
    ne.setNormalSmoothingSize(10.0f);
    ne.setInputCloud(kinect_cloud);
    ne.compute(*normals);
    
    pcl::copyPointCloud(*kinect_cloud,*cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudxyz(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::copyPointCloud(*kinect_cloud,*cloudxyz);
    std::list< pcl::recognition::ObjRecRANSAC::Output > recognized_objects;
    /// @todo make magic numbers into params
    double success_probability=0.99;
    orransac.recognize(*cloudxyz,*normals,recognized_objects,success_probability);
    
    // iterate through the recognized objects, obtain names and get pcls to transform onto cloud
    // the final concatenated point cloud to visualize
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclCloudTransform(new pcl::PointCloud<pcl::PointXYZRGB>());

    for (const pcl::recognition::ObjRecRANSAC::Output & object : recognized_objects) {
        //obtain the object string, use it to find the initial point cloud to transpose on cloud scene
        std::string objstring = object.object_name_; //:: ?

        //get the point cloud with normal, need to copy to pcl without normal and add to cloud
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pclCloud = pclMap[objstring];
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclCloudNoNorm(new pcl::PointCloud<pcl::PointXYZRGB>());

        // copy to point cloud without normal, now add to scene cloud
        pcl::copyPointCloud(*pclCloud, *pclCloudNoNorm);

        // need to create an eigenmatrix from the transform parameters
        // no idea if this is correct, need to create an eigenMatrix for transformation
        Eigen::Affine3f eMatrix = Eigen::Affine3f::Identity();
        for (std::size_t i = 0; i < 4; i++) {
            for (std::size_t j = 0; j < 4; j++) {
                eMatrix.matrix()(i,j) = object.rigid_transform_[i*4 + j];
            }
        }
        //outputs pclCloudTransform, now correctly transformed to add to cloud
        pcl::transformPointCloud(*pclCloudNoNorm, *pclCloudTransform, eMatrix);
        //now concatenate with existing point cloud to get new visualization
        *cloud += *pclCloudTransform;
    }

    // now visualize with the template objects superimposed on
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->updatePointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");

    /// @todo support writing to file again
//    if(ply_file_indices.size() > 0 ){
//        pcl::PCLPointCloud2 cloud2;
//        pcl::toPCLPointCloud2(*cloud,cloud2);
//        saveCloud("cloudname.ply",cloud2,false,false);
//        done = true;
//    }
  }

  k2g.shutDown();
  return 0;
}

