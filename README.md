# Unscented Kalman Filter Project
This project is the continuation of the [Extended Kalman Filter Project](https://github.com/rudi77/ExtendedKalmanFilter). 
In this project we are utilizing an Unscented Kalman Filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. 

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and intall [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. Please see [this concept in the classroom](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77) for the required version and installation scripts.

Once the install for uWebSocketIO is complete, the main program can be built and ran by doing the following from the project top directory.

## Other Important Dependencies

* cmake >= v3.5
* make >= v4.1
* gcc/g++ >= v5.4

## Basic Build Instructions
1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./UnscentedKF -f output_file.txt`.

## Basic Usage:
```
Usage: UnscentedKF [-t | -f filename| -h]
CmdLine args description:
-t             Call test methods
-f filename    Path to the output file
-h             Help description
```

## Data Visualiztion
I've implemented a notebook that I used for data visualization. It is based on the jupyter notebooks from Mercedes and can be found [here](https://github.com/rudi77/CarND-Unscented-Kalman-Filter-Project/blob/master/data/visualization.ipynb)

[//]: # (Image References)

[ekf_plot]: ./images/ekf_plot.png "EKF plot"
[ukf_plot]: ./images/ukf_plot.png "UKF plot"
[ekf_rsme]: ./images/ekf_rsme.png "EKF RSME"
[ukf_rsme]: ./images/ukf_rsme.png "UKF RSME"
[ekf_rsme_plot]: ./images/ekf_rsme_plot.png "EKF RSME"
[ukf_rsme_plot]: ./images/ukf_rsme_plot.png "UKF RSME"
[nis_lidar]: ./images/nis_lidar.png "NIS LIDAR"
[nis_radar]: ./images/nis_radar.png "NIS RADAR"
[nis_table]: ./images/nis_distribution_table.png"

## EKF vs. UKF

EKF              |  UKF
:---------------------:|:-------------------------:
![text alt][ekf_plot] |  ![text alt][ukf_plot] 

EKF RSME             |  UKF RSME
:---------------------:|:-------------------------:
![text alt][ekf_rsme_plot] |  ![text alt][ukf_rsme_plot] 

## NIS LIDAR and RADAR Plots
The following plot show the NIS (normalized innovation squared) values for the chosen longitudinal and angular acceleration noise parameters.

NIS LIDAR              |  NIS RADAR
:---------------------:|:-------------------------:
![text alt][nis_lidar] |  ![text alt][nis_radar] 

