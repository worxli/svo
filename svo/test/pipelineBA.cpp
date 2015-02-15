// This file is part of SVO - Semi-direct Visual Odometry.
//
// Copyright (C) 2014 Christian Forster <forster at ifi dot uzh dot ch>
// (Robotics and Perception Group, University of Zurich, Switzerland).
//
// SVO is free software: you can redistribute it and/or modify it under the
// terms of the GNU General Public License as published by the Free Software
// Foundation, either version 3 of the License, or any later version.
//
// SVO is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#include <svo/config.h>
#include <svo/frame_handler_mono.h>
#include <svo/map.h>
#include <svo/frame.h>
#include <vector>
#include <string>
#include <vikit/math_utils.h>
#include <vikit/vision.h>
#include <vikit/abstract_camera.h>
#include <vikit/atan_camera.h>
#include <vikit/pinhole_camera.h>
#include <opencv2/opencv.hpp>
#include <sophus/se3.h>
#include <iostream>
#include "test_utils.h"
#include <fstream>

using namespace std;

namespace svo {

class BenchmarkNode
{
  vk::AbstractCamera* cam_;
  svo::FrameHandlerMono* vo_;

public:
  BenchmarkNode();
  ~BenchmarkNode();
  void runFromFolder();
};

BenchmarkNode::BenchmarkNode()
{
  cam_ = new vk::PinholeCamera(640, 480, 584.6, 580.9, 319.5, 239);
  //5.8465239833745420e+02 0. 3.1950000000000000e+02
  //0. 5.8098398265888625e+02 2.3950000000000000e+02 0. 0. 1.
  //PinholeCamera(double width, double height,
                // double fx, double fy, double cx, double cy,
                // double d0=0.0, double d1=0.0, double d2=0.0, double d3=0.0, double d4=0.0);
  vo_ = new svo::FrameHandlerMono(cam_);
  vo_->start();
}

BenchmarkNode::~BenchmarkNode()
{
  delete vo_;
  delete cam_;
}

void BenchmarkNode::runFromFolder()
{
  ofstream outfile;
  outfile.open ("/home/worxli/Datasets/data/associate_unscaled.txt");

  for(int img_id = 0;;++img_id)
  {

    // load image
    std::stringstream ss;
    ss << "/home/worxli/Datasets/data/img/color" << img_id << ".png";
	
    std::cout << "reading image " << ss.str() << std::endl;
    cv::Mat img(cv::imread(ss.str().c_str(), 0));

    // end loop if no images left
    if(img.empty())
      break;

    assert(!img.empty());

    // process frame
    vo_->addImage(img, img_id);

    // display tracking quality
    if(vo_->lastFrame() != NULL)
    {
      int id = vo_->lastFrame()->id_;
    	std::cout << "Frame-Id: " << id << " \t"
                  << "#Features: " << vo_->lastNumObservations() << " \t"
                  << "Proc. Time: " << vo_->lastProcessingTime()*1000 << "ms \n";

    	// access the pose of the camera via vo_->lastFrame()->T_f_w_.
	std::cout << vo_->lastFrame()->T_f_w_ << endl;

	//std::count << vo_->lastFrame()->pos() << endl;
  Quaterniond quat = vo_->lastFrame()->T_f_w_.unit_quaternion();
  Vector3d trans = vo_->lastFrame()->T_f_w_.translation();

	outfile << trans.x() << " "
          << trans.y() << " "
          << trans.z() << " "
          << quat.z()  << " "  
          << quat.w()  << " " 
          << quat.x()  << " " 
          << quat.y()  << " " 
          << "depth/mapped" << img_id << ".png "
          << "img/color" << img_id << ".png\n";

    }
  }
  outfile.close();
}

} // namespace svo

int main(int argc, char** argv)
{
  {
    svo::BenchmarkNode benchmark;
    benchmark.runFromFolder();
  }
  printf("BenchmarkNode finished.\n");
  return 0;
}

