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

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/registration.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>
#include <signal.h>
#include <string>
#include <limits>
#include <iostream>
#include <cstdlib>
#include <Eigen/Core>

//bool stop = false;

enum processor{
	CPU, OPENCL, OPENGL
};

// void sigint_handler(int s)
// {
//     stop = true;
// }

class K2G {

public:

	K2G(processor p): undistorted_(512, 424, 4), registered_(512, 424, 4), listener_(libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth),big_mat_(1920, 1082, 4),qnan_(std::numeric_limits<float>::quiet_NaN()){

		//signal(SIGINT,sigint_handler);

		if(freenect2_.enumerateDevices() == 0)
		{
			std::cout << "no kinect2 connected!" << std::endl;
			exit(-1);
		}

		serial_ = freenect2_.getDefaultDeviceSerialNumber();
		switch(p){
			case CPU:
				std::cout << "creating CPU processor" << std::endl;
				pipeline_ = new libfreenect2::CpuPacketPipeline();
				break;
			case OPENCL:
				std::cout << "creating OpenCL processor" << std::endl;
				pipeline_ = new libfreenect2::OpenCLPacketPipeline();
				break;
			case OPENGL:
				std::cout << "creating OpenGL processor" << std::endl;
				pipeline_ = new libfreenect2::OpenGLPacketPipeline();
				break;
			default:
				std::cout << "creating CPU processor" << std::endl;
				pipeline_ = new libfreenect2::CpuPacketPipeline();
				break;
		}
		
		dev_ = freenect2_.openDevice(serial_, pipeline_);
		dev_->setColorFrameListener(&listener_);
		dev_->setIrAndDepthFrameListener(&listener_);
		dev_->start();

		registration_ = new libfreenect2::Registration(dev_->getIrCameraParams(), dev_->getColorCameraParams());

		prepareMake3D(dev_->getIrCameraParams());
 	}

 	/*
	void addCallback(std::function f){
		callbacks_.push_back(f);
	}*/

	libfreenect2::Freenect2Device::IrCameraParams getIrParameters(){
		libfreenect2::Freenect2Device::IrCameraParams ir = dev_->getIrCameraParams();
		return ir;
	}

	libfreenect2::Freenect2Device::ColorCameraParams getRgbParameters(){
		libfreenect2::Freenect2Device::ColorCameraParams rgb = dev_->getColorCameraParams();
		return rgb;
	}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr getCloud(){
		const short w = undistorted_.width;
		const short h = undistorted_.height;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>(w, h));
        
		return updateCloud(cloud);
	}
    

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr updateCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud){
		
		listener_.waitForNewFrame(frames_);
		libfreenect2::Frame * rgb = frames_[libfreenect2::Frame::Color];
		libfreenect2::Frame * depth = frames_[libfreenect2::Frame::Depth];

		registration_->apply(rgb, depth, &undistorted_, &registered_, true, &big_mat_);
		const short w = undistorted_.width;
		const short h = undistorted_.height;

		const float * itD0 = (float *)undistorted_.data;
		const char * itRGB0 = (char *)registered_.data;
		pcl::PointXYZRGB * itP = &cloud->points[0];
        bool is_dense = true;
		
		for(int y = 0; y < h; ++y){

			const unsigned int offset = y * w;
			const float * itD = itD0 + offset;
			const char * itRGB = itRGB0 + offset * 4;
			const float dy = rowmap(y);

			for(size_t x = 0; x < w; ++x, ++itP, ++itD, itRGB += 4 )
			{
				const float depth_value = *itD / 1000.0f;
				
				if(!std::isnan(depth_value) && !(std::abs(depth_value) < 0.0001)){
	
					const float rx = colmap(x) * depth_value;
                	const float ry = dy * depth_value;               
					itP->z = depth_value;
					itP->x = rx;
					itP->y = ry;

					itP->b = itRGB[0];
					itP->g = itRGB[1];
					itP->r = itRGB[2];
				} else {
					itP->z = qnan_;
					itP->x = qnan_;
					itP->y = qnan_;

					itP->b = qnan_;
					itP->g = qnan_;
					itP->r = qnan_;
                    is_dense = false;
				}
			}
		}
        cloud->is_dense = is_dense;
		listener_.release(frames_);
		return cloud;
	}

	void shutDown(){
		dev_->stop();
  		dev_->close();
	}

	cv::Mat getColor(){
		listener_.waitForNewFrame(frames_);
		libfreenect2::Frame * rgb = frames_[libfreenect2::Frame::Color];
		cv::Mat tmp(rgb->height, rgb->width, CV_8UC4, rgb->data);
		cv::Mat r = tmp.clone();
		listener_.release(frames_);
		return std::move(r);
	}

	cv::Mat getDepth(){
		listener_.waitForNewFrame(frames_);
		libfreenect2::Frame * depth = frames_[libfreenect2::Frame::Depth];
		cv::Mat tmp(depth->height, depth->width, CV_8UC4, depth->data);
		cv::Mat r = tmp.clone();
		listener_.release(frames_);
		return std::move(r);
	}

	std::pair<cv::Mat, cv::Mat> getDepthRgb(){
		listener_.waitForNewFrame(frames_);
		libfreenect2::Frame * depth = frames_[libfreenect2::Frame::Depth];
		libfreenect2::Frame * rgb = frames_[libfreenect2::Frame::Color];
		registration_->apply(rgb, depth, &undistorted_, &registered_);
		cv::Mat tmp_depth(undistorted_.height, undistorted_.width, CV_8UC4, undistorted_.data);
		cv::Mat tmp_color(registered_.height, registered_.width, CV_8UC4, registered_.data);
		cv::Mat r = tmp_color.clone();
		cv::Mat d = tmp_depth.clone();
		listener_.release(frames_);
		return std::move(std::pair<cv::Mat, cv::Mat>(r,d));
	}

private:

	void prepareMake3D(const libfreenect2::Freenect2Device::IrCameraParams & depth_p)
	{
		const int w = 512;
		const int h = 424;
	    float * pm1 = colmap.data();
	    float * pm2 = rowmap.data();
	    for(int i = 0; i < w; i++)
	    {
	        *pm1++ = (i-depth_p.cx + 0.5) / depth_p.fx;
	    }
	    for (int i = 0; i < h; i++)
	    {
	        *pm2++ = (i-depth_p.cy + 0.5) / depth_p.fy;
	    }
	}

	libfreenect2::Freenect2 freenect2_;
	libfreenect2::Freenect2Device * dev_ = 0;
	libfreenect2::PacketPipeline * pipeline_ = 0;
	libfreenect2::Registration * registration_ = 0;
	libfreenect2::SyncMultiFrameListener listener_;
	libfreenect2::FrameMap frames_;
	libfreenect2::Frame undistorted_, registered_, big_mat_;
	Eigen::Matrix<float,512,1> colmap;
	Eigen::Matrix<float,424,1> rowmap;
	std::string serial_;
	int map_[512 * 424];
	float qnan_;   
};