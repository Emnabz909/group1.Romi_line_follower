# Romi Line Follower Bot
Term project for ME405 by Elliott Joseph Bryniarski & Emmanuel Baez

## Project Overview
This years term project is to create a line following robot that is able to follow a given course with multiple obstacles including: dashed lines, hashed lines, rounded corners, and a wall object to go around. Once the robot has gone around the course once it must also attempt to go back to the starting point by any means. The robot must be able to compete on multiple tracks to prevent being hard coded for certain scenarios.

## Sensors Used
### Reflective Sensors
We are using one 8-array Reflectance Sensor inconjunction with two single-array Reflectance Sensors. These sensors are mostly used to detect the line that we are trying to follow. These sesnsors are connected to the front of our robot and placed to be as in the middle as we possible could. 

### IR Sensors
We are using two Infrared Sensors. These sensors are used to detect objects within the path of our robot. One sensor is at the front so that it will be able to detect the wall in front of it, while the other is to the side to be able to detect once the robot has passed the wall. 

### IMU
We are using a BNO055 IMU that is located at the front of the robot, and a bit to the left. The BNO055 is an IMU that we are using so that we are to get data like acceleration, orientation, and angular rates. The BNO055 is a combination  of an accelerometer, magnetometer, and gyrocosope. We are using the IMU to get the Euler Angles of our bot so that it could be later used to calculate our pitch and yaw.

## Pin Allocations
[Pin Table.pdf](https://github.com/Emnabz909/group1.Romi_line_follower/files/13679848/Pin.Table.pdf)

## Classes
### BNO055
This class is used to we are able to retrieve various orientation-related data from our BNO055 sensor. Most of the early code of the class are constants that define various register addresses, modes, and data types for our BNO055 sensor. The rest of the code is functions that we could possibly use. For example there are ways to change which mode the IMU is in. For our project we are using the NDOF mode. There is also a function to calibrate our sensor, so that the data we are using is as accurates as can be. The more important function that we used is the getVector function. This is how we are able to get the data from a ceratin sensor in the IMU. We use this function so that we are able to get the Euler Angles of our robot and use them the calculate our Yaw. 

### Encoder
This class is actually two classes in one. The first class is used for initiallizing our Encoders. This class is used so that we are ableto keep track of our position, change in position, and velocity. The other class that is in here is our ClosedLoop class. This class is used to intialize our closed loop controller with specified proportional, intergral, and derivative gains. Once we have these calculatedwe are then able to return a control signal for use to use in our main file. 

### smbus
This class is mostly used to ease the transition code from our BNO055 to our I2C bus. We were having trouble reading the data from our IMU, and adding this class has made it easier to do so.

## Keynotes About Main
### PID Controller

### Obstacle Detector
