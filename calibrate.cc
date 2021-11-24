#include <gflags/gflags.h>
#include <stdio.h>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

const char* WINDOW_NAME = "Calibration";

DEFINE_int32(rows, 7, "number of rows in calibration chart");
DEFINE_int32(cols, 10, "number of columns in calibration chart");
DEFINE_double(size, 30.0, "the length of a square edge in the chart");
DEFINE_string(filename_template, "image%02d.png",
              "input image file name template");
DEFINE_string(output, "camera.xml", "camera parameter XML output file");
DEFINE_int32(num, 25, "number of images");
DEFINE_bool(ignore_missing, false, "continue when an image file is not found");
DEFINE_bool(display, false, "show result image window");

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  cv::Size pattern_size(FLAGS_cols, FLAGS_rows);
  fprintf(stderr, "pattern size %d x %d\n", FLAGS_cols, FLAGS_rows);

  cv::Size image_size;
  std::vector<std::vector<cv::Point2f>> image_points;

  if (FLAGS_display) {
    cv::namedWindow(WINDOW_NAME, cv::WindowFlags::WINDOW_AUTOSIZE);
  }

  // (1) Load calibration images
  std::vector<cv::Mat> src_img;
  for (int i = 0; i < FLAGS_num; i++) {
    char buf[256];
    snprintf(buf, 256, FLAGS_filename_template.c_str(), i);
    cv::Mat img = cv::imread(buf, cv::IMREAD_COLOR);
    if (img.data == NULL) {
      fprintf(stderr, "cannot load image file : %s\n", buf);
      if (!FLAGS_ignore_missing) {
        return -1;
      }
      continue;
    }
    src_img.push_back(img);
    if (src_img.size() == 1) {
      image_size = cv::Size(img.size().width, img.size().height);
    } else {
      assert(image_size.width == img.size().width &&
             image_size.height == img.size().height);
    }
  }

  // (2) Detect chessboard corners
  for (size_t i = 0; i < src_img.size(); i++) {
    auto& img = src_img.at(i);
    std::vector<cv::Point2f> corners;
    fprintf(stderr, "%02ld...", i);
    bool found = cv::findChessboardCorners(img, pattern_size, corners);
    if (found) {
      cv::Mat src_gray;
      fprintf(stderr, "ok\n");
      cv::cvtColor(img, src_gray, cv::ColorConversionCodes::COLOR_BGR2GRAY);
      cv::cornerSubPix(
          src_gray, corners, cv::Size(3, 3), cv::Size(-1, -1),
          cv::TermCriteria(cv::TermCriteria::MAX_ITER | cv::TermCriteria::EPS,
                           20, 0.03));
      cv::drawChessboardCorners(img, pattern_size, cv::Mat(corners), found);
      image_points.push_back(corners);
    } else {
      fprintf(stderr, "fail\n");
    }
    if (FLAGS_display) {
      cv::imshow(WINDOW_NAME, img);
      cv::waitKey(0);
    }
  }
  if (FLAGS_display) {
    cv::destroyWindow(WINDOW_NAME);
  }

  // (3) Define reference object geometry
  std::vector<cv::Point3f> checker_pattern;
  for (int j = 0; j < FLAGS_rows; j++) {
    for (int k = 0; k < FLAGS_cols; k++) {
      cv::Point3f p;
      p.x = j * FLAGS_size;
      p.y = k * FLAGS_size;
      p.z = 0;
      checker_pattern.push_back(p);
    }
  }
  std::vector<std::vector<cv::Point3f>> object_points;
  for (size_t i = 0; i < image_points.size(); i++) {
    object_points.push_back(checker_pattern);
  }

  // (4) Estimate intrinsic and extrinsic camera parameters
  cv::Mat intrinsic;
  cv::Mat rotation;
  cv::Mat translation;
  cv::Mat distortion;
  std::vector<cv::Mat> rvecs;
  std::vector<cv::Mat> tvecs;

  cv::calibrateCamera(object_points, image_points, image_size, intrinsic,
                      distortion, rvecs, tvecs);

  // (5) Write to XML file
  int base = 0;
  cv::FileStorage fs(FLAGS_output, cv::FileStorage::WRITE);
  fs << "intrinsic" << intrinsic;
  fs << "rotation" << rvecs.at(base);
  fs << "translation" << tvecs.at(base);
  fs << "distortion" << distortion;
  fs.release();

  return 0;
}
