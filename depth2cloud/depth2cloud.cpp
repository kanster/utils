/**
  * convert image into point cloud
  * @author Kanzhi Wu
  */

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

#include <iostream>
#include <vector>

/// depth image to xyz point cloud
///
/// depth_img -- 16 bit depth image
/// param     -- cx, cy, fx, fy
///
pcl::PointCloud<pcl::PointXYZ>::Ptr img2cloud( cv::Mat depth_img, float * param ) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud( new pcl::PointCloud<pcl::PointXYZ>() );
  cloud->width = depth_img.cols;
  cloud->height = depth_img.rows;
  cloud->is_dense = false;
  float bad_point = std::numeric_limits<float>::quiet_NaN();

  for ( int y = 0; y < depth_img.rows; ++ y ) {
    for ( int x = 0; x < depth_img.cols; ++ x ) {
      pcl::PointXYZ pt;
      if (depth_img.at<unsigned short>(y, x) == 0) {
        pt.x = bad_point; //std::numeric_limits<float>::quiet_NaN();
        pt.y = bad_point; //std::numeric_limits<float>::quiet_NaN();
        pt.z = bad_point; //std::numeric_limits<float>::quiet_NaN();
      }
      else {
        pt.z = depth_img.at<unsigned short>(y, x)/1000.;
        pt.x = pt.z*(x-param[0])/param[3];
        pt.y = pt.z*(y-param[1])/param[4];
      }
      cloud->points.push_back(pt);
    }
  }
  return cloud;
}



/// convert images to xyzrgb point cloud
///
/// rgb_img     -- 3 channels, 8bit rgb image
/// depth_img   -- 16 bit depth image
/// param       -- cx, cy, fx, fy
///
pcl::PointCloud<pcl::PointXYZRGB>::Ptr img2cloud(cv::Mat rgb_img, cv::Mat depth_img, float * param ) {
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud( new pcl::PointCloud<pcl::PointXYZRGB>() );
  cloud->width = rgb_img.cols;
  cloud->height = rgb_img.rows;
  cloud->is_dense = false;
  float bad_point = std::numeric_limits<float>::quiet_NaN();

  for ( int y = 0; y < rgb_img.rows; ++ y ) {
    for ( int x = 0; x < rgb_img.cols; ++ x ) {
      pcl::PointXYZRGB pt;
      pt.b = rgb_img.at<cv::Vec3b>(y,x)[0];
      pt.g = rgb_img.at<cv::Vec3b>(y,x)[1];
      pt.r = rgb_img.at<cv::Vec3b>(y,x)[2];
      if (depth_img.at<unsigned short>(y, x) == 0) {
        pt.x = bad_point; //std::numeric_limits<float>::quiet_NaN();
        pt.y = bad_point; //std::numeric_limits<float>::quiet_NaN();
        pt.z = bad_point; //std::numeric_limits<float>::quiet_NaN();
      }
      else {
        pt.z = depth_img.at<unsigned short>(y, x)/1000.;
        pt.x = pt.z*(x-param[0])/param[2];
        pt.y = pt.z*(y-param[1])/param[3];
      }
      cloud->points.push_back(pt);
    }
  }
  return cloud;
}

/// print function command usage
///
/// prog_name   -- program names
///
void print_usage (const char* prog_name) {
  std::cout << "\n\nUsage: "<< prog_name <<" [options]\n\n"
            << "Options:\n"
            << "-------------------------------------------\n"
            << "-h            this help\n";
            << "-c            with rgb information or not\n"
            << "-rgb          rgb image path\n"
            << "-depth        depth image path\n"
            << "-p            provide camera parameters or not\n"
            << "-param        camera parameters\n"
            << "\n\n";
}



int main( int argc, char** argv ) {
  std::string rgbp, depthp;
  float * param[4] = {319.5, 239.5, 525.0, 525.0};

  if (pcl::console::find_argument(argc, argv, "-h") >= 0) {
    print_usage (argv[0]);
    return 0;
  }

  if ( pcl::console::find_switch( argc, argv, "-c" ) ) {
    if ( pcl::console::parse( argc, argv, "-rgb", rgbp ) >= 0 ) {
      std::cout << "Load RGB image from " << rgbp << std::endl;
    }
    else {
      std::cout << "Please specify RGB image path!\n";
      return 0;
    }
  }

  if ( pcl::console::parse( argc, argv, "-depth", depthp ) >= 0 ) {
    std::cout << "Load depth image from " << depthp << std::endl;
  }
  else {
    print_usage(argv[0]);
    std::cout << "Please specify depth image path!\n";
    return 0;
  }

  if ( pcl::console::find_switch( argc, argv, "-p" ) ) {
    std::vector<float> paramvec;
    if ( pcl::console::parse_multiple_arguments( argc, argv, "-param", paramvec ) >= 0 ) {
      param = &paramvec[0];
      std::cout << "Camera parameters:\n" <<
                   "cx = " << param[0] << "\n" <<
                   "cy = " << param[1] << "\n" <<
                   "fx = " << param[2] << "\n" <<
                   "fy = " << param[3] << "\n";
    }
    else {
      std::cout << "Please specify the camera parameters!\n";
      print_usage(argv[0]);
      return 0;
    }
  }

}

