Camera Calibration
==========================================

## *ChArUco*

ArUco markers were improved by interspersing them inside a checkerboard called ChArUco. Checkerboard corner intersections provide more stable corners because the edge location bias on one square is countered by the opposite edge orientation in the connecting square. By interspersing ArUco markers inside the checkerboard, each checkerboard corner gets a label which enables it to be used in complex calibration or pose scenarios where you cannot see all the corners of the checkerboard. [More](https://github.com/opencv/opencv_contrib/blob/master/modules/aruco/tutorials/charuco_detection/charuco_detection.markdown)

## Building (Linux only)

Run the bootstrap script to get the necessary dependencies. Then, run the build script.
```shell
./boostrap.sh
./build.sh
```

## Creating board and markers

To create markers the **create-marker** executable can be used. Configuration files to create a charuco board and a aruco marker can be found at **etc/conf/create-charuco.json** and **etc/conf/create-aruco.json** respectively.

Usage example, run the produced binary passing the desired configuration file:

```shell
./build/src/is/calibration-tools/create-aruco/create-marker ./etc/conf/create-aruco.json
```

## Calibrating Cameras

The executable **calibrate-intrinsic** computes the intrinsic and distortion parameters using a ChArUco marker and outputs the corresponding [CameraCalibration](https://github.com/labviros/is-msgs/tree/v1.1.8/docs#is.vision.CameraCalibration) object as a json file.

Usage example, run the produced binary passing the desired configuration file:

```shell
./build/src/is/calibration-tools/calibrate-aruco/calibrate-intrinsic ./etc/conf/calibrate-aruco.json
```

The executable **calibrate-extrinsic** computes the extrinsic parameters, that is, the transformation that can change poses from the camera frame to the world frame and vice-versa. The world frame of reference will coincide with the one of the aruco marker. The transformation is added to the respective [CameraCalibration](https://github.com/labviros/is-msgs/tree/v1.1.8/docs#is.vision.CameraCalibration) json file.

Usage example, run the produced binary passing the desired configuration file:

```shell
./build/src/is/calibration-tools/calibrate-aruco/calibrate-extrinsic ./etc/conf/calibrate-aruco.json
```

Example configuration file (**./etc/conf/calibrate-aruco.json**): The string "{}" is used as a wildcard for the camera_id. So in the example below the topic used will be "CameraGateway.7.Frame" for instance.
```json
{
  "camera_id": 7,
  "world_id": 1003,

  "uri": "amqp://10.10.2.11",
  "topic": "CameraGateway.{}.Frame",
  "data_dir": "bristol/{}",
 
  "save_images": true,

  "intrinsic": {
    "dictionary": "DICT_4X4_50",
    "marker_length": 0.136,
    "square_length": 0.170,
    "n_squares_x": 4,
    "n_squares_y": 3,
    "samples": 100
  },

  "extrinsic": {
    "dictionary": "DICT_4X4_50",
    "marker_id": 0,
    "marker_length": 0.660,
    "offset": -0.400
  }
}
```
