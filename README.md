# OpenCV camera calibration example

Based on the example code in
http://opencv.jp/sample/camera_calibration.html
, rewritten using OpenCV2 and C++ syntax.

## calibrate.cc

Estimates camera parameters based on checkerboard patterns in image files.

Example usage:

    $ ls image*.png
    image0000.png
    image0001.png
     ...
    image0024.png

    $ ./calibrate -cols 8 -rows 5 -size 2.54 -num 25 -output my_camera.xml

## undistort.cc

Displays undistorted image using the estimated camera parameters.

Example usage:

    $ ./undistort -calib my_camera.xml image.png
