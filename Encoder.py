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
    
    def __init__(self, timer, chA_pin, chB_pin):
        
        self.tim = timer
        
        self.upd_cnt = 0
        
        self.position = 0
        self.delta = 0
        
        self.chA = self.tim.channel(1, pin= chA_pin, mode = pyb.Timer.ENC_AB)
        self.chB = self.tim.channel(2, pin= chB_pin, mode = pyb.Timer.ENC_AB)
    
    def update(self):
        
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
        
        return self.position
    
    def get_delta(self):
        
        return self.delta
    
    def get_velocity(self):
        
        self.velocity = (self.delta * 2 * pi) / (1440 * self.ticksdiff * 1e-6)
        
        return self.velocity
    
    def zero(self):
        
        self.position = 0
        
        self.position1 = 0
        
        self.upd_cnt = 0
        
class ClosedLoop:
    
    def __init__(self, kp, ki, kd):
        
        self.kp = kp  # Proportional Gain
        self.ki = ki  # Integral Gain
        self.kd = kd  # Derivative Gain
        
        self.integral = 0
        self.derivative = 0
        self.count = 0
        self.prev_error = 0

    def calculate(self, setpoint, measured_value, timedelta):
        
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
