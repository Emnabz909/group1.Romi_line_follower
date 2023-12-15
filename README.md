# Romi Line Follower Bot
Term project for ME405 by Elliott Joseph Bryniarski & Emmanuel Baez

## Table of Contents
[Project Overview](#project-overview)<br>
[Our Romi Bot](#our-romi-bot)<br>
[3D Print CAD](#3d-print-cad)<br>
[Tasks](#tasks)<br>
&nbsp;&nbsp;&nbsp;&nbsp;[follow_track](#follow_track)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[init](#init)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[run](#run)<br>
[Bricking the Hardware](#bricking-the-hardware)<br>



## Project Overview
The goal of our term project is to create a line following robot that is able to follow a line on a race course with multiple obstacles including: dashed lines, hashed lines, rounded corners, and a wall object to go around. Once the robot has gone around the course once it must also attempt to go back to the starting point by any means. The robot must be able to compete on multiple tracks to prevent being hard coded for certain scenarios. Our project was initially going very well until the night before our submission.

## Our Romi Bot
![Romi front](https://github.com/Emnabz909/group1.Romi_line_follower/assets/147099440/93658fca-04e8-4b75-b6b9-9817f8c916fd)

![Romi side](https://github.com/Emnabz909/group1.Romi_line_follower/assets/147099440/cb52cac2-b0c6-46d1-9a1f-1c4842be914c)

## 3D Print CAD


## Sensors Used
### Reflective Sensors
We are using one 8-array Reflectance Sensor inconjunction with two single-array Reflectance Sensors. These sensors are mostly used to detect the line that we are trying to follow. These sesnsors are connected to the front of our robot and placed to be as in the middle as we possible could. 

### IR Sensors
We are using two Infrared Sensors. These sensors are used to detect objects within the path of our robot. One sensor is at the front so that it will be able to detect the wall in front of it, while the other is to the side to be able to detect once the robot has passed the wall. 

### IMU
We are using a BNO055 IMU that is located at the front of the robot, and a bit to the left. The BNO055 is an IMU that we are using so that we are to get data like acceleration, orientation, and angular rates. The BNO055 is a combination  of an accelerometer, magnetometer, and gyrocosope. We are using the IMU to get the Euler Angles of our bot so that it could be later used to calculate our pitch and yaw.

## Pin Allocations
![pin table](https://github.com/Emnabz909/group1.Romi_line_follower/assets/147099440/fdb79fda-71a2-4c41-86b2-443f6cfe9b36)

## Demo Video
Click Thumbnail to View Video
<div align="left">
      <a href="https://www.youtube.com/watch?v=LgOl3hI2dDQ">
         <img src="https://img.youtube.com/vi/LgOl3hI2dDQ/0.jpg" style="width:100%;">
      </a>
</div>

## Classes
### BNO055
This class is used to we are able to retrieve various orientation-related data from our BNO055 sensor. Most of the early code of the class are constants that define various register addresses, modes, and data types for our BNO055 sensor. The rest of the code is functions that we could possibly use. For example there are ways to change which mode the IMU is in. For our project we are using the NDOF mode. There is also a function to calibrate our sensor, so that the data we are using is as accurates as can be. The more important function that we used is the getVector function. This is how we are able to get the data from a ceratin sensor in the IMU. We use this function so that we are able to get the Euler Angles of our robot and use them the calculate our Yaw. 

### Encoder
This class is actually two classes in one. The first class is used for initiallizing our Encoders. This class is used so that we are ableto keep track of our position, change in position, and velocity. The other class that is in here is our ClosedLoop class. This class is used to intialize our closed loop controller with specified proportional, intergral, and derivative gains. Once we have these calculatedwe are then able to return a control signal for use to use in our main file. 

### smbus
This class is mostly used to ease the transition code from our BNO055 to our I2C bus. We were having trouble reading the data from our IMU, and adding this class has made it easier to do so.

### task_share
This task is used to communicate data between our taskes without the risk of data corruption.

### cotask
This task is used to create a cooperative mulitasking system. This will allow our tasks to act as generators and will also create a scheduler to run our tasks at based of our desired needs. 

## Tasks
Our code consists of one single task which is follow_track. This is because all that we are doing is constantly checking and updating the duty cycle for our motors based on the readings from our line sensors. Our plan was to have multiple states including the states that can handle the wall obstacle and returning to to zero position, but unfortunately our Romi hardware stopped working the night before our demo when we were working on that code. This is detailed further in [Bricking the Hardware](#bricking-the-hardware).

*   ### follow_track

    This is our only task

    *   #### init

        The init state of our follow_track task is state 0. This state initializes all of the variables for our run state.

    *   #### run

        The run state is state 1 of the follow_track task. This state contains all of the code for our project. This state loops through a [set of if statements](#if-statements-for-sensor-readings) to figure out how far off of the line our romi is using the reflectance sensors. This distance is used as a measured value for a controller where the set point is 0 or no distance from the line. The output of that controller is the yaw rate the romi needs to have in order to follow the line. We then plug that yaw rate into the kinematic equations for our wheels to figure out an angular velocity for the left and right wheel. Those angular velocities are then used as a set points for another controller which measured point is the velocity of the wheel acquired from the encoders. The output of this controller is the duty cycle of each motor. Through both of these controllers the line following is achieved. With the code we made before [Bricking the Hardware](#bricking-the-hardware), we were also able to use the Front IR sensor to detect that the wall is there but not handle avoiding the wall. So before it bricked the romi just stops before the wall as seen in [the demo video](#demo-video).

## Bricking the Hardware

## Appendix A (Code Snippets)
### If Statements for Sensor Readings
<img width="516" alt="image" src="https://github.com/Emnabz909/group1.Romi_line_follower/assets/106140514/bd00e07b-27c5-414e-adcb-28b959a55dcf"><br>
[^ See in main.py ^](https://github.com/Emnabz909/group1.Romi_line_follower/blob/efa484d757a301c30c7f6e89b4be0a8eb917c530/main.py#L121C1-L146C30)

## Appendix B (Task Diagrams)
### Task Diagram
<img width="723" alt="image" src="https://github.com/Emnabz909/group1.Romi_line_follower/assets/106140514/996e0bfe-8cd4-4158-a207-ca525d3332b2">


#### follow_line
<img width="723" alt="image" src="https://github.com/Emnabz909/group1.Romi_line_follower/assets/106140514/0eeb9cce-42fb-4e67-8c8a-29b58eff41c1">

[Return to Top](#romi-line-follower-bot)
