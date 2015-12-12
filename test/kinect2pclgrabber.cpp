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
#include <pcl/io/ply_io.h>
#include <pcl/PCLPointCloud2.h>
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
  std::cout << "Syntax is: " << argv[0] << " [-processor 0|1|2] [output.ply]\n -processor options 0,1,2 correspond to CPU, OPENCL, and OPENGL respectively\n";
  processor freenectprocessor = OPENGL;
  std::vector<int> ply_file_indices;
  if(argc>1){
      int fnpInt;
      ply_file_indices = parse_file_extension_argument (argc, argv, ".ply");
      parse_argument (argc, argv, "-processor", fnpInt);
      freenectprocessor = static_cast<processor>(fnpInt);
      
  }
    
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
  
  // estimate normals http://pointclouds.org/documentation/tutorials/normal_estimation_using_integral_images.php#normal-estimation-using-integral-images
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>());
  
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
    
    pcl::IntegralImageNormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
    /// @todo make magic numbers into params
    ne.setMaxDepthChangeFactor(1.0f);
    ne.setNormalSmoothingSize(10.0f);
    ne.setInputCloud(cloud);
    ne.compute(*normals);
	
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_2(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
    pcl::concatenateFields (*cloud, *normals, *cloud_2);
    	
    
    if(ply_file_indices.size() > 0 ){
        pcl::PCLPointCloud2 cloud2;
        pcl::toPCLPointCloud2(*cloud_2,cloud2);
        saveCloud(std::string(argv[ply_file_indices[0]]),cloud2,false,false);
        done = true;
    }
  }

  k2g.shutDown();
  return 0;
}

