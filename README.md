# Extended Kalman Filter
This repository contains an implementation of an Extended Kalman Filter to track a bicycle's position around our car. This respository uses the Udacity simulator and its related project of the Self Driving Cars Nanodegree.

As a data source the simulator provides simulated lidar and radar measurements detecting a bicycle that travels around our vehicle. During simulation, the lidar measurements are red circles, radar measurements are blue circles with an arrow pointing in the direction of the observed angle, and estimation markers are green triangles. 

[Image] of the simulator

## Building the project
Firstly the installation of uWebSocketIO library is required. To perform the installation you should use one of the scripts provided, all of them already contain the required dependencies to run this project.

Once the installation of uWebSocketIO is completed, the main program can be built and run by doing the following from the project top directory.

>   1. mkdir build
>   2. cd build
>   3. cmake ..
>   4. make
>   5. ./ExtendedKF

Download the Udacity simulator in this [link](https://github.com/udacity/self-driving-car-sim/releases).

Find more information related with this project in this [Link](https://github.com/udacity/CarND-Extended-Kalman-Filter-Project). A deeper explanation of the inputs and the outputs are provided here.

## Kalman Filter
Remember this is a two-step estimation

The first step of this loop is the state prediction where we use the information we have to predict the state of the bicycle. This is done until a new sensor measurement arrives.

The second step is the measurement update where we use new observations comming from the sensors to correct our belief about the bicycle state.

For this project we have available two different sensors (Lidar and Radar) to perform our measurement update. Both of them provide their respective measurement in different ways as we will see later; nevertheless, we will be able to extract the values to perform our update. 

Let's define out state transition matrix





## Motion model
This is used to perform the prediction step
Used in the prediction state.
Our motion model wil be xxx

## Measurement model
This is used to perform the correction step where the last measurement is used to update the state estimated and its uncertainty
Measurement prediction for the Lidar and the Radar

### Lidar
 

### Radar

