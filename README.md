# Calibration Depth Pose

Suppose you have a depth camera rigidly attached to another sensor which gives you poses (position + orientation).
This library enables you to find the relative calibration of these two sensors.

![Calibration Animation](doc/calibration_anim.gif "Calibration iterations")

![Calibration Animation 2](doc/calibration_anim_2.gif "Calibration iterations 2")

![Calibration Animation Noisy 2](doc/calibration_anim_noisy_2.gif "Calibration iterations noisy 2")

![Calibration Animation Noisy](doc/calibration_anim_noisy.gif "Calibration iterations noisy")




## Synthetic Data Generator

The synthetic data generator is an interactive tool to simulate depth map acquisition by a camera. The camera is assumed to be pinhole and without noise.

This tool generates a dataset containing the following files:
  - *dataset.txt*: this is the main file which contains the path of the poses file and all the pointlcoud files
  - *dataset.poses*: this file contains the list of the camera poses (one line per capture) (qw qx qy qz x y z)
  - *pc_.ply*: point cloud files




## Calibration Example

This example shows how to use the library.
