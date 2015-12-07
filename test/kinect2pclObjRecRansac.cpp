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

int main(int argc, char *argv[])
{
  std::cout << "Syntax is: " << argv[0] << " [-processor 0|1|2] [model.ply]\n -processor options 0,1,2 correspond to CPU, OPENCL, and OPENGL respectively\n";
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
      pcl::copyPointCloud(*model,*modelnormal);
      pcl::PointCloud<pcl::PointXYZ>::Ptr modelxyz(new pcl::PointCloud<pcl::PointXYZ>());
      pcl::copyPointCloud(*model,*modelxyz);
      
      std::string modelFile(argv[ply_file_indices[idx]]);

      //we need to map these strings, which are the names, to the ply files
      pclMap[modelFile] = model;

      reader.read(modelFile,*model);
      /// @todo should const_cast really be used?
      orransac.addModel(*modelxyz,*modelnormal,modelFile);
      
  }
  
    // estimate normals http://pointclouds.org/documentation/tutorials/normal_estimation_using_integral_images.php#normal-estimation-using-integral-images
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);

  // setup kinect
  //changed cloud type to ::Ptr
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
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
    
    

    pcl::IntegralImageNormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
    /// @todo make magic numbers into params
    ne.setMaxDepthChangeFactor(1.0f);
    ne.setNormalSmoothingSize(10.0f);
    ne.setInputCloud(cloud);
    ne.compute(*normals);
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudxyz(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::copyPointCloud(*cloud,*cloudxyz);
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

    if(ply_file_indices.size() > 0 ){
        pcl::PCLPointCloud2 cloud2;
        pcl::toPCLPointCloud2(*cloud,cloud2);
        saveCloud("cloudname.ply",cloud2,false,false);
        done = true;
    }
  }

  k2g.shutDown();
  return 0;
}

