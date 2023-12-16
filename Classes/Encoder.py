#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Nov 30 11:14:44 2023

@author: elliottbryniarski
"""

import pyb
from time import ticks_diff, ticks_us
from math import pi

class Encoder:
    ''' !@brief       Interface with quadrature encoders
        @details      This class is designed for use with a microcontroller,
                      where you can configure encoder input pins and set up
                      callback-based data updates for position and velocity
                      tracking
   '''
    
    def __init__(self, timer, chA_pin, chB_pin):
        ''' !@brief        Constructs an encoder object
             @details      This function is responsible for initializing
                           hardware configuration for the encoder: including
                           timer instances and pins. The functions is also 
                           designed to continuously update and store position 
                           and velocity data from the encoder
             @param        tim: timer associated with the encoder
             @param        chA: pin connected to Channel A of the encoder
             @param        chB: pin connected to Channel B of the encoder
        '''
        
        self.tim = timer
        
        self.upd_cnt = 0
        
        self.position = 0
        self.delta = 0
        
        self.chA = self.tim.channel(1, pin= chA_pin, mode = pyb.Timer.ENC_AB)
        self.chB = self.tim.channel(2, pin= chB_pin, mode = pyb.Timer.ENC_AB)
    
    def update(self):
        ''' !@brief       Updates encoder position and delta
            @details      This function is responsible for tracking the
                          encoder's position and calculating the change in 
                          position (delta) over time. It uses the upd_cnt 
                          attribute to determine when to calculate the delta 
                          and stores the results in class attributes for 
                          further use and data storage
        ''' 
        
        self.upd_cnt += 1
        
        self.position = self.tim.counter()
        
        self.ticks1 = ticks_us()
        
        if self.upd_cnt > 1:
            
            self.delta = self.position - self.position1
            
            self.ticksdiff = ticks_diff(self.ticks2,self.ticks1)
            
        else:
            
            self.delta = 0
        
        self.position1 = self.position
        
        self.ticks2 = self.ticks1
        
        
    def get_position(self):
        ''' !@brief      Gets the most recent encoder position
            @details     This function is responsible for getting the most
                         recent encoder postiton as an integer for later use
            @return      Returns the most recent encoder position as an
                         integer
       '''
        
        return self.position
    
    def get_delta(self):
        ''' !@brief      Gets the most recent encoder delta
            @details     This function is responsible for getting the most
                         recent encoder delta as an integer for later use
            @return      Returns the most recent encoder delta as an
                         integer
       '''  
        
        return self.delta
    
    def get_velocity(self):
        ''' !@brief      Gets the most recent encoder velocity
            @details     This function is responsible for getting the most
                         recent encoder velocity as an integer for later use
            @return      Returns the most recent encoder velocity as an
                         integer
       '''
        
        self.velocity = (self.delta * 2 * pi) / (1440 * self.ticksdiff * 1e-6)
        
        return self.velocity
    
    def zero(self):
        ''' !@brief       Resets the encoder position to zero
            @details      This function reinitializes our encoder data and
                          sets it to a known initial state            
        '''  
        
        self.position = 0
        
        self.position1 = 0
        
        self.upd_cnt = 0
        
class ClosedLoop:
    ''' !@brief       PID Controler for a closed loop system
         @details     This class represents a PID controller used in closed-loop control systems. 
                      The controller continuously adjusts the system's input based on the difference
                      between the desired setpoint and the actual measured value. 
    '''
    
    def __init__(self, kp, ki, kd):
         ''''  @brief       Initializes a ClosedLoop controller with PID gains
               @details     This function is responsible for initializing a ClosedLoop
                            controller with the provided Proportional, Integral, and
                            Derivative gains.

               @param       kp: Proportional Gain
               @param       ki: Integral Gain
               @param       kd: Derivative Gain
         '''
        
        self.kp = kp  # Proportional Gain
        self.ki = ki  # Integral Gain
        self.kd = kd  # Derivative Gain
        
        self.integral = 0
        self.derivative = 0
        self.count = 0
        self.prev_error = 0

    def calculate(self, setpoint, measured_value, timedelta):
        '''@brief       Calculates the control signal for a ClosedLoop controller
           @details     This function is responsible for calculating the control
                        signal based on the provided setpoint, measured value, and
                        time difference.

           @param       setpoint: The desired value or setpoint
           @param       measured_value: The actual measured value
           @param       timedelta: The time difference between the current and previous calculations
    '''
        
        self.error = setpoint - measured_value
        
        proportional = self.kp * self.error
        
        self.integral += self.ki * self.error * timedelta
        
        if self.prev_error == None:
            
            self.derivative = self.kd * ((self.error - self.prev_error)/(timedelta))
            
        else:
            
            self.derivative = 0

        # Calculate the control signal
        control_signal = proportional + self.integral + self.derivative
        
        self.prev_error = self.error
        
        return control_signal
