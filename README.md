Camera Calibration Example
==========================================

## ChArUco

ArUco markers were improved by interspersing them inside a checkerboard called ChArUco. Checkerboard corner intersections provide more stable corners because the edge location bias on one square is countered by the opposite edge orientation in the connecting square. By interspersing ArUco markers inside the checkerboard, each checkerboard corner gets a label which enables it to be used in complex calibration or pose scenarios where you cannot see all the corners of the checkerboard.

## charuco-intrinsic-calib

Computes camera intrinsic and distortion parameters using a ChArUco marker and outputs the corresponding CameraCalibration object as a json file.

Usage example:
```shell
./charuco-intrinsic-calib --uri amqp://edge.is:30000 --camera 0 --dictionary 0 --marker-length 0.16 --square-length 0.20 --width 4 --height 3
```

## aruco-extrinsinc-calib

Computes the extrinsic parameters, that is, the transformation that changes object pose from the camera frame to the world frame (represented by the pose of the
aruco marker). The transformation is then added to the respective CameraCalibration json file.

Usage example:
```shell
./aruco-extrinsic-calib --uri amqp://edge.is:30000 --camera 0 --dictionary 0 --marker 0 --marker-length 0.3 --axis-offset 0
```