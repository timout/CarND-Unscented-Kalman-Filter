# Unscented Kalman Filter

Udacity CarND Term 2, Project 2 - Unscented Kalman Filters

## Project Basics
In this project, I used C++ to write a program taking in radar and lidar data to track position using Unscented Kalman Filters, a more advanced and more accurate method than in my previous Extended Kalman Filter project.

The code will make a prediction based on the sensor measurement and then update the expected position. See files in the 'src' folder for the primary C++ files making up this project.

## Build instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./UnscentedKF` along with the Term 2 simulator.
   
## Results
Based on the provided data set, the Unscented Kalman Filter produced the below results.  
* px - x-position 
* py - y-position
* vx - velocity in the x-direction 
* vy - velocity in the y-direction  
* MSE - Residual error, was calculated by mean squared error (MSE).

#### Both Radar and Lidar

| Input |   MSE   |
| ----- | ------- |
|  px   | 0.0691 |
|  py   | 0.0803 |
|  vx   | 0.1719 |
|  vy   | 0.2115 |

#### Radar only

| Input |   MSE   |
| ----- | ------- |
|  px   | 0.2785 |
|  py   | 0.3176 |
|  vx   | 0.6686 |
|  vy   | 0.6213 |

#### Lidar only

| Input |   MSE   |
| ----- | ------- |
|  px   | 0.1627 |
|  py   | 0.1466 |
|  vx   | 0.2561 |
|  vy   | 0.3180 |

