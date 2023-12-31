# Romi Line Follower Bot
Term project for ME405 by Elliott Bryniarski & Emmanuel Baez

## Table of Contents
[Project Overview](#project-overview)<br>
[Our Romi Bot](#our-romi-bot)<br>
[3D Print](#3d-print)<br>
[Demo Video](#demo-video)<br>
[Sensors Used](#sensors-used)<br>
&nbsp;&nbsp;&nbsp;&nbsp;[Reflectance Sensors](#reflectance-sensors)<br>
&nbsp;&nbsp;&nbsp;&nbsp;[IR Sensors](#ir-sensors)<br>
&nbsp;&nbsp;&nbsp;&nbsp;[IMU](#imu)<br>
[Pin Allocations](#pin-allocations)<br>
[Classes](#classes)<br>
&nbsp;&nbsp;&nbsp;&nbsp;[BNO055](#bno055-github-ghirlekar)<br>
&nbsp;&nbsp;&nbsp;&nbsp;[Encoder](#encoder)<br>
&nbsp;&nbsp;&nbsp;&nbsp;[smbus](#smbus-github-gkluoe)<br>
&nbsp;&nbsp;&nbsp;&nbsp;[Task-share](#task_share)<br>
&nbsp;&nbsp;&nbsp;&nbsp;[Cotask](#cotask)<br>
[Tasks](#tasks)<br>
&nbsp;&nbsp;&nbsp;&nbsp;[follow_track](#follow_track)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[init](#init)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[run](#run)<br>
[Bricking the Hardware](#bricking-the-hardware)<br>
[In Conclusion](#in-conclusion)<br>
[Appendix A (Code Snippets)](#appendix-a-code-snippets)<br>
[Appendix B (Task Related Diagrams)](#appendix-b-task-related-diagrams)<br>
[Appendix C (Hand Calcs)](#appendix-c-hand-calcs)<br>


## Project Overview
The goal of our term project is to create a line following robot that is able to follow a line on a course with multiple obstacles including: dashed lines, hashed lines, rounded corners, and a wall object to go around. Once the robot has gone around the course once it must also attempt to go back to the starting point. The same code must be able to compete on multiple tracks to prevent presetting variables to know what part of the track you are on and not using feedback from sensors. Our project was initially going very well until the hardware bricked and we could not test our wall avoiding or position tracking code.

## Our Romi Bot
![Romi front](https://github.com/Emnabz909/group1.Romi_line_follower/assets/147099440/93658fca-04e8-4b75-b6b9-9817f8c916fd)

![Romi side](https://github.com/Emnabz909/group1.Romi_line_follower/assets/147099440/cb52cac2-b0c6-46d1-9a1f-1c4842be914c)

## 3D Print
The 3D print that we made to hold the sensors is depicted below. Along with the CAD.
![3D Print](https://github.com/Emnabz909/group1.Romi_line_follower/assets/106140514/ff470019-a069-4c41-aa0f-47380bf037c2)<br>
![image](https://github.com/Emnabz909/group1.Romi_line_follower/assets/106140514/f1a8eb51-ae62-40a8-9f4f-700a9e47bbce)<br>
[CAD File](https://github.com/Emnabz909/group1.Romi_line_follower/blob/main/Cad%20for%20Supports/Romi%20Sensor%20Holder.SLDPRT)

## Demo Video
Initially our project was going very well and our code worked well to follow lines. We used the line sensors to tell how far off center our Romi was from the line. The feedback from these sensors was used to essentialy tell the wheels how fast to go to follow the line. This is a very high level explanation that is explained in depth [later in this page](#tasks). Our first struggle was getting the Romi to go at a speed we want while also being able to do the slalom on the track. To do this, we modified our software to detect if the Romi itself has a high yaw, or essentially if it is turning, and if it does, the software tells the Romi to slow down in order to compensate with the turns. This was the first breakthrough since accomplishing the line following aspect of the project. Our next breakthrough was going to be getting passed the wall obstacle on the course. We had a great idea for the sensor feedback we could use which utilizes two IR sensors to detect if an object was near. One on the front and one on the side of the Romi. The planned procedure is documented below as well. But before we could test the code we wrote for the wall, somehow we messed up our hardware. Due to this we were only able to use the video we had taken prior to our Romi not functioning which is depicted below. This does not accomplish the wall obstacle or the return to start aspect of the project, but the Romi follows the track very well and slows down for turns and was definitely successful in handling the slalom part of the track.

Click Thumbnail to View Video
<div align="left">
      <a href="https://www.youtube.com/watch?v=LgOl3hI2dDQ">
         <img src="https://img.youtube.com/vi/LgOl3hI2dDQ/0.jpg" style="width:100%;">
      </a>
</div>

[Back to Top](#romi-line-follower-bot)

## Sensors Used
*   ### Reflectance Sensors
    We are using one [8-array Reflectance Sensor](https://www.pololu.com/product/4248) along with two [single-array Reflectance Sensors](https://www.pololu.com/product/4242). These sensors are used to detect if the line is below them, and based on the geometry of the robot which we know, we can tell where the robot needs to go to be centered on the line. These sesnsors are connected to the front of our robot in the center.
    
    ![reflectance sensors](https://github.com/Emnabz909/group1.Romi_line_follower/assets/147099440/d47f3f6b-e494-4833-a40d-72bd890875b8)

*   ### IR Sensors
    We are using two [Infrared Sensors](https://www.amazon.com/dp/B0B74QJV8H?psc=1&ref=ppx_yo2ov_dt_b_product_details). These sensors are used to detect objects within the path of our robot. One sensor is at the front so that it will be able to detect the wall in front of it, while the other is to the side to be able to detect once the robot has passed the wall.
     
![ir](https://github.com/Emnabz909/group1.Romi_line_follower/assets/147099440/c4d35d18-1fe0-435b-b905-36806245fb91)

*   ### IMU
    We are using a [BNO055 IMU](https://www.digikey.com/en/products/detail/adafruit-industries-llc/2472/5699182?utm_adgroup=&utm_source=google&utm_medium=cpc&utm_campaign=Pmax_Shopping_Product_Silicon%20Valley%20Category%20Awareness&utm_term=&utm_content=&utm_id=go_cmp-20773039395_adg-_ad-__dev-c_ext-_prd-5699182_sig-Cj0KCQiAj_CrBhD-ARIsAIiMxT8pJTmY1CzgpDeoNfldrNdTTDs2zvcY8C21eEwcTdVhm81UirK_qJgaAofnEALw_wcB&gad_source=1&gclid=Cj0KCQiAj_CrBhD-ARIsAIiMxT8pJTmY1CzgpDeoNfldrNdTTDs2zvcY8C21eEwcTdVhm81UirK_qJgaAofnEALw_wcB) that is located at the front of the robot, and a bit to the right. The BNO055 is an 9-DOF IMU that we are using so that we are to get data like acceleration, orientation, and angular rates. The BNO055 is a combination  of an accelerometer, magnetometer, and gyrocosope. We are using the IMU to get the Euler Angles of our bot including the heading. We use this feedback to tell if the bot is turning and also in the position tracking.
    
    ![bno](https://github.com/Emnabz909/group1.Romi_line_follower/assets/147099440/6324abb7-98da-42c7-94df-30f90e19da53)<br>

[Back to Top](#romi-line-follower-bot)

## Pin Allocations
Below is tables that depict what pins each sensor is connected to. And also a diagram of the pin layout.
![pin table](https://github.com/Emnabz909/group1.Romi_line_follower/assets/147099440/fdb79fda-71a2-4c41-86b2-443f6cfe9b36)<br>
<img width="972" alt="image" src="https://github.com/Emnabz909/group1.Romi_line_follower/assets/106140514/f23bbfa8-062d-457b-a83a-256fd5945468">

[Back to Top](#romi-line-follower-bot)


## Classes
*   ### [BNO055 (Github: ghirlekar)](https://github.com/ghirlekar/bno055-python-i2c)
    This class is used to we are able to retrieve various orientation-related data from our BNO055 sensor. Most of the early code of the class are constants that define various register addresses, modes, and data types for our BNO055 sensor. The rest of the code is functions that we could possibly use. For example there are ways to change which mode the IMU is in. For our project we are using the NDOF mode. There is also a function to calibrate our sensor, so that the data we are using is as accurates as can be. The more important function that we used is the getVector function. This is how we are able to get the data from a ceratin sensor in the IMU. We use this function so that we are able to get the Euler Angles of our robot and use them the calculate our Yaw. We used a class created by ghirlekar on github and edited to fit the needs of our project.

*   ### Encoder
    This class is actually two classes in one. The first class is used for initiallizing our Encoders. This class is used so that we are ableto keep track of our position, change in position, and velocity. The other class that is in here is our ClosedLoop class. This class is used to intialize our closed loop controller with specified proportional, intergral, and derivative gains. Once we have these calculatedwe are then able to return a control signal for use to use in our main file. 

*   ### [smbus (Github: gkluoe)](https://github.com/gkluoe/micropython-smbus)
    This is a class we are using as a wrapper so the python module smbus is available on micropython which is what our microcontrollers use. We got this wrapper from gkluoe on github.

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

        The run state is state 1 of the follow_track task. This state contains all of the code for our project. This state loops through a [set of if statements](#if-statements-for-sensor-readings) to figure out how far off of the line our romi is using the reflectance sensors. This distance is used as a measured value for a controller where the set point is 0 or no distance from the line. The output of that controller is the yaw rate the romi needs to have in order to follow the line. We then plug that yaw rate into the [kinematic equations for the angular velocity of the wheels](#appendix-c-hand-calcs) to figure out an angular velocity for the left and right wheel. Those angular velocities are then used as a set points for another controller which measured point is the velocity of the wheel acquired from the encoders. The output of this controller is the duty cycle of each motor. Through [both of these controllers](#controllers) the line following is achieved. One of the interesting parts of our code is that whenever the angular velocity, measured by the gyroscope, is greater than a certain amount, the [velocity is modified](#variable-velocity). This was done because the Romi could go fast on every part of the track except for the slalom. With the code we made before [Bricking the Hardware](#bricking-the-hardware), we were also able to use the Front IR sensor to detect that the wall is there but not handle avoiding the wall. So before it bricked the romi just stops before the wall as seen in [the demo video](#demo-video).

## Bricking the Hardware
The night before our demonstration when we finally had time to work on the Romi during finals week, our hardware bricked. We were hoping to finish up the wall obstacle handling using IR sensors to detect if the wall is there. We attached an IR sensor to the front and to the side of the Romi. For the software, we planned to detect the wall in the front and set a wall flag to enter a state that will turn 90 degrees to the right where the side IR sensor can now detect the wall. Then the Romi will go to a state that moves forward until the side IR sensor doesn't recognize the wall anymore and goes to a state that turns turn 90 degrees to the opposite direction and go back to the state to follow the wall and repeat wall follow and turn 90 until it sees the line again. Then it will return to the follow line state. [Example of Possible Code](#wall-handling)

The other main goal of the project was to get the Romi to return to the start. In order to do this, our plan was to take the heading from the IMU and the velocity data from the encoders to create X and Y coordinates of our Romi. [Example of Possible Code](#position-tracking)

[Back to Top](#romi-line-follower-bot)

## In Conclusion
We came into the project knowing very little about the hardware we were using other than basic knowledge of pin assignment and reading sensors. It was a steady but slow climb from that point to now. It definitely had its ups and downs, most notably the hardware failing the day before demos, but along the way we learned a lot about sensor feedback, controls, and python as a tool for mechatronics. Having a lot of freedom to design our system and orient the sensors to achieve the goals lead to a lot of healthy iterating. Going through the troubleshooting of this robot was one of the best practices for learning python. Another key skill we gained from this project is the ability to read through data sheets for the sensors and other peripherals we are using and understand how to wire them. These are the most notable things we learned but alonside those we gained a lot of knowledge about what kind of sensors are used for different applications and how to use the sensors for our own applications. Regardless of the fact that we never got to complete the track, we are still proud of how much we learned this quarter and we are looking forward to how we can apply this knowledge to our future projects. 


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
![get data](https://github.com/Emnabz909/group1.Romi_line_follower/assets/147099440/73b13286-2b12-4abd-9535-c88ce8837309)

## Appendix B (Task Related Diagrams)
### Task Diagram
<img width="723" alt="image" src="https://github.com/Emnabz909/group1.Romi_line_follower/assets/106140514/996e0bfe-8cd4-4158-a207-ca525d3332b2">

#### follow_track
<img width="723" alt="image" src="https://github.com/Emnabz909/group1.Romi_line_follower/assets/106140514/0eeb9cce-42fb-4e67-8c8a-29b58eff41c1"><br>

## Appendix C (Hand Calcs)
![image](https://github.com/Emnabz909/group1.Romi_line_follower/assets/106140514/5b3ebb9a-0180-42f7-b204-30f3935e81a0)


[Back to Top](#romi-line-follower-bot)
