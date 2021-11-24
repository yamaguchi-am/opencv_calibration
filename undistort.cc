#include <gflags/gflags.h>

#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

DEFINE_string(calib, "camera.xml", "camera parameter XML file name");

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0]
              << " <image filename> [-calib <camera parameter file>]"
              << std::endl;
    return -1;
  }

  // (1) Load image to be undistorted
  std::string image_filename = argv[1];
  cv::Mat src_image = cv::imread(image_filename);
  if (src_image.data == NULL) {
    std::cerr << "failed to load image: " << image_filename << std::endl;
    return -1;
  }

  // (2) Read camera parameters from file
  cv::FileStorage fs(FLAGS_calib, cv::FileStorage::READ);
  cv::Mat intrinsic;
  cv::Mat distortion;
  fs["intrinsic"] >> intrinsic;
  fs["distortion"] >> distortion;
  fs.release();

  // (3) Undistort
  cv::Mat undistorted;
  cv::undistort(src_image, undistorted, intrinsic, distortion);

  // (4) Show the result and wait for key input
  cv::imshow("Original", src_image);
  cv::imshow("Undistorted", undistorted);
  cv::waitKey(0);
  cv::destroyAllWindows();
  return 0;
}
