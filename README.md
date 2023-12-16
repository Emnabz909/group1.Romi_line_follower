# Romi Line Follower Bot
Term project for ME405 by Elliott Joseph Bryniarski & Emmanuel Baez

## Table of Contents
[Project Overview](#project-overview)<br>
[Our Romi Bot](#our-romi-bot)<br>
[3D Print CAD](#3d-print)<br>
[Sensors Used](#sensors-used)<br>
&nbsp;&nbsp;&nbsp;&nbsp;[Reflectance Sensors](#reflectance-sensors)<br>
&nbsp;&nbsp;&nbsp;&nbsp;[IR Sensors](#ir-sensors)<br>
&nbsp;&nbsp;&nbsp;&nbsp;[IMU](#imu)<br>
[Pin Allocations](#pin-allocations)<br>
[Demo Video](#demo-video)<br>
[Classes](#classes)<br>
&nbsp;&nbsp;&nbsp;&nbsp;[BNO055](#bno055)<br>
&nbsp;&nbsp;&nbsp;&nbsp;[Encoder](#encoder)<br>
&nbsp;&nbsp;&nbsp;&nbsp;[smbus](#smbus)<br>
&nbsp;&nbsp;&nbsp;&nbsp;[Task-share](#task_share)<br>
&nbsp;&nbsp;&nbsp;&nbsp;[Cotask](#cotask)<br>
[Tasks](#tasks)<br>
&nbsp;&nbsp;&nbsp;&nbsp;[follow_track](#follow_track)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[init](#init)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[run](#run)<br>
[Bricking the Hardware](#bricking-the-hardware)<br>
[Appendix A (Code Snippets)](#appendix-a-code-snippets)<br>
[Appendix B (Task Related Diagrams)](#appendix-b-task-related-diagrams)<br>


## Project Overview
The goal of our term project is to create a line following robot that is able to follow a line on a race course with multiple obstacles including: dashed lines, hashed lines, rounded corners, and a wall object to go around. Once the robot has gone around the course once it must also attempt to go back to the starting point by any means. The robot must be able to compete on multiple tracks to prevent being hard coded for certain scenarios. Our project was initially going very well until the night before our submission when the hardware bricked. 

## Our Romi Bot
![Romi front](https://github.com/Emnabz909/group1.Romi_line_follower/assets/147099440/93658fca-04e8-4b75-b6b9-9817f8c916fd)

![Romi side](https://github.com/Emnabz909/group1.Romi_line_follower/assets/147099440/cb52cac2-b0c6-46d1-9a1f-1c4842be914c)

## 3D Print
The 3D print that we made to hold the sensors is depicted below. Along with the CAD.
![3D Print](https://github.com/Emnabz909/group1.Romi_line_follower/assets/106140514/ff470019-a069-4c41-aa0f-47380bf037c2)

## Sensors Used
*   ### Reflectance Sensors
    We are using one 8-array Reflectance Sensor inconjunction with two single-array Reflectance Sensors. These sensors are mostly used to detect the line that we are trying to follow. These sesnsors are connected to the front of our robot and placed to be as in the middle as we possible could.
    
    ![reflectance sensors](https://github.com/Emnabz909/group1.Romi_line_follower/assets/147099440/d47f3f6b-e494-4833-a40d-72bd890875b8)

*   ### IR Sensors
    We are using two Infrared Sensors. These sensors are used to detect objects within the path of our robot. One sensor is at the front so that it will be able to detect the wall in front of it, while the other is to the side to be able to detect once the robot has passed the wall.
     
![ir](https://github.com/Emnabz909/group1.Romi_line_follower/assets/147099440/c4d35d18-1fe0-435b-b905-36806245fb91)

*   ### IMU
    We are using a BNO055 IMU that is located at the front of the robot, and a bit to the left. The BNO055 is an IMU that we are using so that we are to get data like acceleration, orientation, and angular rates. The BNO055 is a combination  of an accelerometer, magnetometer, and gyrocosope. We are using the IMU to get the Euler Angles of our bot so that it could be later used to calculate our pitch and yaw.
    
    ![bno](https://github.com/Emnabz909/group1.Romi_line_follower/assets/147099440/6324abb7-98da-42c7-94df-30f90e19da53)

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
*   ### BNO055 
    This class is used to we are able to retrieve various orientation-related data from our BNO055 sensor. Most of the early code of the class are constants that define various register addresses, modes, and data types for our BNO055 sensor. The rest of the code is functions that we could possibly use. For example there are ways to change which mode the IMU is in. For our project we are using the NDOF mode. There is also a function to calibrate our sensor, so that the data we are using is as accurates as can be. The more important function that we used is the getVector function. This is how we are able to get the data from a ceratin sensor in the IMU. We use this function so that we are able to get the Euler Angles of our robot and use them the calculate our Yaw. 

*   ### Encoder
    This class is actually two classes in one. The first class is used for initiallizing our Encoders. This class is used so that we are ableto keep track of our position, change in position, and velocity. The other class that is in here is our ClosedLoop class. This class is used to intialize our closed loop controller with specified proportional, intergral, and derivative gains. Once we have these calculatedwe are then able to return a control signal for use to use in our main file. 

*   ### smbus
    This class is mostly used to ease the transition code from our BNO055 to our I2C bus. We were having trouble reading the data from our IMU, and adding this class has made it easier to do so.

*   ### task_share
    This task is used to communicate data between our taskes without the risk of data corruption.

*   ### cotask
    This task is used to create a cooperative mulitasking system. This will allow our tasks to act as generators and will also create a scheduler to run our tasks at based of our desired needs. 

## Tasks
Our code consists of one single task which is follow_track. This is because all that we are doing is constantly checking and updating the duty cycle for our motors based on the readings from our line sensors. Our plan was to have multiple states including the states that can handle the wall obstacle and returning to to zero position, but unfortunately our Romi hardware stopped working the night before our demo when we were working on that code. This is detailed further in [Bricking the Hardware](#bricking-the-hardware).

*   ### follow_track

    This is our only task. It consists of two states. One of the states is just the initialize state that initializes our variables. The next state is the state where our line following happens. [FSM Diagram](#follow_track-1)

    *   #### init

        The init state of our follow_track task is state 0. This state initializes all of the variables for our run state and will go to the run state if the button is pressed on the Romi.

    *   #### run

        The run state is state 1 of the follow_track task. This state contains all of the code for our project. This state loops through a [set of if statements](#if-statements-for-sensor-readings) to figure out how far off of the line our romi is using the reflectance sensors. This distance is used as a measured value for a controller where the set point is 0 or no distance from the line. The output of that controller is the yaw rate the romi needs to have in order to follow the line. We then plug that yaw rate into the kinematic equations for our wheels to figure out an angular velocity for the left and right wheel. Those angular velocities are then used as a set points for another controller which measured point is the velocity of the wheel acquired from the encoders. The output of this controller is the duty cycle of each motor. Through [both of these controllers](#controllers) the line following is achieved. One of the interesting parts of our code is that whenever the angular velocity, measured by the gyroscope, is greater than a certain amount, the [velocity is modified](# to slow down to handle curves. This was done because the Romi could go fast on every part of the track except for the slalom. With the code we made before [Bricking the Hardware](#bricking-the-hardware), we were also able to use the Front IR sensor to detect that the wall is there but not handle avoiding the wall. So before it bricked the romi just stops before the wall as seen in [the demo video](#demo-video).

## Bricking the Hardware
The night before our demonstration when we finally had time to work on the Romi during finals week, our hardware bricked. We were hoping to finish up the wall obstacle handling using IR sensors to detect if the wall is there. We attached an IR sensor to the front and to the side of the Romi. For the software, we planned to detect the wall in the front and set a wall flag to enter a state that will turn 90 degrees to the right where the side IR sensor can now detect the wall. Then the Romi will go to a state that moves forward until the side IR sensor doesn't recognize the wall anymore and goes to a state that turns turn 90 degrees to the opposite direction and go back to the state to follow the wall and repeat wall follow and turn 90 until it sees the line again. Then it will return to the follow line state. [Example of Possible Code](#wall-handling)

The other main goal of the project was to get the Romi to return to the start. In order to do this, our plan was to take the heading from the IMU and the velocity data from the encoders to create X and Y coordinates of our Romi. [Example of Possible Code](#position-tracking)


## Appendix A (Code Snippets)
### If Statements for Sensor Readings
<img width="400" alt="image" src="https://github.com/Emnabz909/group1.Romi_line_follower/assets/106140514/bd00e07b-27c5-414e-adcb-28b959a55dcf"><br>

### Controllers
<img width="700" alt="image" src="https://github.com/Emnabz909/group1.Romi_line_follower/assets/106140514/9529b368-78a0-40b8-839b-7f25c994cf26"><br>

### Variable Velocity
<img width="975" alt="image" src="https://github.com/Emnabz909/group1.Romi_line_follower/assets/106140514/241becca-4220-41b2-9adc-0c2cae5cf904"><br>

### Wall Handling
<img width="400" alt="image" src="https://github.com/Emnabz909/group1.Romi_line_follower/assets/106140514/4e8b7a10-e1af-4678-b261-56df3ca1f544"><br>
<img width="400" alt="image" src="https://github.com/Emnabz909/group1.Romi_line_follower/assets/106140514/2462e8f1-4cfa-461c-b44a-e7ee4d256597"><br>
<img width="400" alt="image" src="https://github.com/Emnabz909/group1.Romi_line_follower/assets/106140514/253b1101-540a-4ca9-a1e0-59dcebbef0cf"><br>
<img width="400" alt="image" src="https://github.com/Emnabz909/group1.Romi_line_follower/assets/106140514/0797f887-2787-44f8-a3c9-7c8cf662e731"><br>

### Position Tracking
<img width="400" alt="image" src="https://github.com/Emnabz909/group1.Romi_line_follower/assets/106140514/02d4c8ac-5f32-49b3-b893-b10ce1b790f5">

### Closed Loop Calculations
![closed loop controls](https://github.com/Emnabz909/group1.Romi_line_follower/assets/147099440/c897d6eb-8560-4227-bb09-05e9a28fcd5e)

### BNO055 Get Data Function
![get data](https://github.com/Emnabz909/group1.Romi_line_follower/assets/147099440/48b5a14d-de2a-48b5-aa3b-7e8b11b3fee8)

## Appendix B (Task Related Diagrams)
### Task Diagram
<img width="723" alt="image" src="https://github.com/Emnabz909/group1.Romi_line_follower/assets/106140514/996e0bfe-8cd4-4158-a207-ca525d3332b2">

#### follow_track
<img width="723" alt="image" src="https://github.com/Emnabz909/group1.Romi_line_follower/assets/106140514/0eeb9cce-42fb-4e67-8c8a-29b58eff41c1"><br>

## [Return to Top](#romi-line-follower-bot)
