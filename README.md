# Calibration Depth Pose

Suppose you have a depth camera rigidly attached to another sensor which gives you poses (position + orientation).
This library enables you to find the relative calibration of these two sensors.

## Example of calibration convergence

|  |  |
:-------------------------:|:-------------------------:
![Calibration Animation 4](doc/calibration_anim_4.gif "Calibration iterations 2") | ![Calibration Animation Noisy 2](doc/calibration_anim_noisy_2.gif "Calibration iterations noisy 2")
![Calibration Animation Noisy](doc/calibration_anim_noisy.gif "Calibration iterations noisy") | ![Calibration Animation 3](doc/calibration_anim_3.gif "Calibration iterations 3")



# Synthetic Data Generator

```bash
./SyntheticDataGenerator mesh_file output_dir
```

The synthetic data generator is an interactive tool to simulate depth map acquisition by a camera. It can load a mesh in OBJ and PLY formats. The camera is assumed to be pinhole and without noise. This tool directly generates a dataset which can be used by the other tools.


## Dataset
A dataset contains the following files:
  - *dataset.txt*: this is the main file which contains the path of the poses file and all the pointlcoud files
  - *dataset.poses*: this file contains the list of the camera poses (one line per capture) (qw qx qy qz x y z)
  - *pc_.ply*: point cloud files



# Examples

## calibrate

This example can directly be used to process your own data. It just requires a dataset and a configuration file. It also saves the concatenation of all point clouds transformed to world coordinates using the estimated calibration at each iteration.

```bash
./calibrate dataset_file configuration_file

```


The configuration file contains the parameters to use for calibration.

For example:
~~~yaml
calibration_parameters:
  nb_iterations: 20
  calibration_initial_guess:
    rotation: [1.0, 0.0, 0.0, 0.0]
    translation: [0.0, 0.0, 0.0]
  distance_type: POINT_TO_PLANE
  matching_max_distance: 0.1
  matching_plane_discriminator_threshold: 0.8
  matching_required_nb_neighbours: 10
~~~

## calibrate_synthetic_data

This example shows how to use the library and synthetic data. Like the previous example, it also saves the concatenation of all point clouds transformed to world coordinates using the estimated calibration at each iteration.


```bash
./calibrate_synthetic_data dataset_file nb_iterations noise_stddev configuration_file

```


The configuration file defines the real calibration and the initial guess for the estimated calibration.
The example does the following steps:
 - The real calibration is applied to synthetic data, so that we obtain point clouds and poses with the real calibration between them
 - Estimate the calibration with the library starting with the given initial guess

~~~yaml
real_calibration:
  rotation: [0.9872283, 0.0806561, 0.1336749, -0.0317164]
  translation: [-0.2, -0.1, 0.05]

estimated_calibration:
  rotation: [1.0, 0.0, 0.0, 0.0]
  translation: [0, 0, 0]

~~~
