# Extended Kalman Filter
This repository contains an implementation of an Extended Kalman Filter to track a bicycle's position around our car. This respository uses the Udacity simulator and its related project of the Self Driving Cars Nanodegree.

As a data source the simulator provides simulated lidar and radar measurements detecting a bicycle that travels around our vehicle. During simulation, the lidar measurements are red circles, radar measurements are blue circles with an arrow pointing in the direction of the observed angle, and estimation markers are green triangles. 

![Simulator](https://user-images.githubusercontent.com/49252525/99920559-6568b200-2cf2-11eb-9676-99be0791878d.png)


## Building the project
Firstly the installation of uWebSocketIO library is required. To perform the installation you should use one of the scripts provided (See `scripts` folder), all of them already contain the required dependencies to run this project. 

Once the installation of uWebSocketIO is completed, the main program can be built and run by doing the following from the project top directory.

```bash
mkdir build
cd build
cmake ..
make
./ExtendedKF
```
Download the Udacity simulator in this [link](https://github.com/udacity/self-driving-car-sim/releases). Keep in mind that Unity engine would be required to run the simulator.

Find more information related with this project in this [link](https://github.com/udacity/CarND-Extended-Kalman-Filter-Project). A deeper explanation of the inputs and the outputs are provided here.

## Kalman Filter
Remember this is a two-step estimation

The first step of this loop is the state prediction where we use the information we have to predict the state of the bicycle. This is done until a new sensor measurement arrives.

The second step is the measurement update where we use new observations comming from the sensors to correct our belief about the bicycle state.

For this project we have available two different sensors (Lidar and Radar) to perform our measurement update. Both of them provide their respective measurement in different ways.

## Motion model
This is used to perform the prediction step.

![motion_model](https://user-images.githubusercontent.com/49252525/99921505-c5faed80-2cf8-11eb-88b7-a722ed168fc7.gif)


State covariance matrix

![process_measurement](https://user-images.githubusercontent.com/49252525/99921499-b8ddfe80-2cf8-11eb-8841-74825a7d89eb.gif)

X = State transition matrix
Q = Process noise covariance matrix
P = Process measurement matrix
## Measurement model
This is used to perform the correction step where the last measurement is used to update the state estimated and its uncertainty

Measurement prediction for the Lidar and the Radar


