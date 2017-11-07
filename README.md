# Extended Kalman Filter Project
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

## Overview

In this project I use a Kalman Filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. The filter makes a prediction based on the sensor measurement and then update the expected position. See files in the `src` for the primary C++ files making up this project.

## Prerequisites

* cmake >= 3.5
* make >= 4.1 
* gcc/g++ >= 5.4

[This](https://github.com/udacity/CarND-Extended-Kalman-Filter-Project) repository contains all instructions for the Project.

This project involves the Udacity's Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases).

## Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make` 
4. Run it: `./ExtendedKF`

## Results

<img src="ekf_result.png" width="800px">
