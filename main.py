#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Nov 20 12:23:04 2023

@author: elliottbryniarski
"""

import pyb
import time
import BNO055
import Encoder
import smbus
from task_share import Share, Queue
import cotask
import gc    

class follow_track:
   ''' !@brief        Interface used to get Romi to follow the line
        @details      This class is designed for use with a microcontroller,
                      and romi kit where you can configure various sensors
                      to set up a line follwoing robot
   '''    
    def __init__(self):
        ''' !@brief        Initiatalizes the controller with specific parameters
             @details      This constructor sets up the necessary parameters for 
                           controlling our romi with various sensors and actuators.
             @param        r_w: Radius of the wheels
             @param        L: Width of the romi
             @param        v: Set velocity of romi
             @param        wall: Flag for wall detection
             @param        toggle_variable: Flag for our start button
             @param        i2c: I2C communication setup
             @param        bno: BNO055 sensor instance
             @param        cal_data: Calibration data for our BNO055 sensor
            @param        bno: BNO055 sensor instance
        '''
        # Constants
        self.r_w = 0.035 # Radius of Wheels
        self.L = 0.149   # Width of Car
        self.v = 0.5 # Set Velocity of Car
        
        # Initialize variables
        self.wall = False              # Wall Detect Flag
        self.toggle_variable = False   # Button Flag
        
        i2c = pyb.I2C(1, pyb.I2C.MASTER)
        i2c.init(pyb.I2C.CONTROLLER, baudrate=400_000)
        
        self.bno = BNO055.BNO055()
        
        self.bno.begin()
        
        cal_data = [95, 0, 11, 0, 235, 255, 62, 245, 150, 252, 67, 8, 1, 0, 254, 255, 255, 255, 232, 3, 211, 1]
        self.bno.setCalibration(cal_data)
        
        self.Mot_R_EN = pyb.Pin.cpu.C2
        self.Mot_R_DIR = pyb.Pin.cpu.C6
        self.Mot_R_TIM = pyb.Pin.cpu.C7
        
        self.Mot_L_EN = pyb.Pin.cpu.B7
        self.Mot_L_DIR = pyb.Pin.cpu.C5
        self.Mot_L_TIM = pyb.Pin.cpu.B6
        
        timR = pyb.Timer(8, freq = 20_000)
        timL = pyb.Timer(4, freq = 20_000)
        
        self.PWMR = timR.channel(2, mode = pyb.Timer.PWM, pin = self.Mot_R_TIM)
        self.PWML = timL.channel(1, mode = pyb.Timer.PWM, pin = self.Mot_L_TIM)
        
        ENC_L_A = pyb.Pin.cpu.A0
        ENC_L_B = pyb.Pin.cpu.A1
        
        ENC_L_tim = pyb.Timer(2, period = 65535, prescaler = 0)
        
        ENC_R_A = pyb.Pin.cpu.B4
        ENC_R_B = pyb.Pin.cpu.B5
        
        ENC_R_tim = pyb.Timer(3, period = 65535, prescaler = 0)
        
        ENCR = Encoder(ENC_R_tim, ENC_R_A, ENC_R_B)
        ENCL = Encoder(ENC_L_tim, ENC_L_A, ENC_L_B)
        
        
        self.L4_sens = pyb.ADC(pyb.Pin.cpu.A5)
        self.L3_sens_adc = pyb.ADC(pyb.Pin.cpu.C0)
        self.L2_sens_adc = pyb.ADC(pyb.Pin.cpu.B0)
        self.L1_sens_adc = pyb.ADC(pyb.Pin.cpu.A4)
        self.L_sens_adc = pyb.ADC(pyb.Pin.cpu.C4)

        self.R4_sens = pyb.ADC(pyb.Pin.cpu.A4)
        self.R3_sens_adc = pyb.ADC(pyb.Pin.cpu.C1)
        self.R2_sens_adc = pyb.ADC(pyb.Pin.cpu.C3)
        self.R1_sens_adc = pyb.ADC(pyb.Pin.cpu.A5)
        self.R_sens_adc = pyb.ADC(pyb.Pin.cpu.B1)
        
        L_sens_EN = pyb.Pin.cpu.C3 
        R_sens_EN = pyb.Pin.cpu.C8
        
        L_sens_EN.high()
        R_sens_EN.high()
        
        self.sens_p_gain = 3
        self.sens_i_gain = 0
        self.sens_d_gain = 0
        
        self.line_ctrl = Encoder.ClosedLoop(self.sens_p_gain, self.sens_i_gain, self.sens_d_gain)
        
        self.omega_p_gain = 2.95
        self.omega_i_gain = 0.1
        self.omega_d_gain = 0
        
        self.omega_L_ctrl = Encoder.ClosedLoop(self.omega_p_gain, self.omega_i_gain, self.omega_d_gain)
        self.omega_R_ctrl = Encoder.ClosedLoop(self.omega_p_gain, self.omega_i_gain, self.omega_d_gain)
        
        ENCL.update()
        ENCR.update()
        
        self.count = 0
        self.ticks2 = 0
        
        button_pin = pyb.Pin.cpu.C13

        self.extint = pyb.ExtInt(button_pin, pyb.ExtInt.IRQ_FALLING, pyb.Pin.PULL_UP, self.button_callback)
        
        IR_front_pin = pyb.Pin.cpu.A6
        
        self.IR_Front = pyb.Pin(IR_front_pin, mode = pyb.Pin.IN)
        
        IR_side_pin = pyb.Pin.cpu.A7
        
        self.IR_side = pyb.Pin(IR_side_pin, mode = pyb.Pin.IN)
        
    def run(self):
        
        if self.L3_sens_adc.read() > 4000 and self.R3_sens_adc.read() > 4000:
            dis_from_line = 0
        elif self.L4_sens.read() > 2000 and self.R4_sens.read() > 4000:
            dis_from_line = 0
        elif self.L4_sens.read() > 2000:
            dis_from_line = -4
        elif self.R4_sens.read() > 4000:
            dis_from_line = 4
        elif self.L3_sens_adc.read() > 4000:
            dis_from_line = -3
        elif self.R3_sens_adc.read() > 4000:
            dis_from_line = 3
        elif self.L2_sens_adc.read() > 4000:
            dis_from_line = -2
        elif self.R2_sens_adc.read() > 4000:
            dis_from_line = 2
        elif self.L1_sens_adc.read() > 4000:
            dis_from_line = -1
        elif self.R1_sens_adc.read() > 4000:
            dis_from_line = 1
        elif self.L_sens_adc.read() > 4000:
            dis_from_line = 0
        elif self.R_sens_adc.read() > 4000:
            dis_from_line = 0
        else:
            dis_from_line = 0
            
        x, y, z = self.bno.getVector(BNO055.BNO055.VECTOR_GYROSCOPE)
    
        if abs(z) >= 1.5:
            self.v = 0.05
            self.sens_p_gain = 3
            self.sens_i_gain = 2
            self.sens_d_gain = 0
            
            self.time_slow = time.ticks_ms()
        elif abs(z) < 0.3 and (time.ticks_diff(self.time_slow2,self.time_slow)*1e-3) > 0.7 and self.time_slow2 != 0 and self.time_slow != 0:
            self.v = 0.7
            self.sens_p_gain = 3
            self.sens_i_gain = 0
            self.sens_d_gain = 0
        
        self.time_slow2 = time.ticks_ms()
        
        self.count += 1
        
        self.ticks1 = time.ticks_us()
        
        if self.count > 1:
            
            ticksdiff = time.ticks_diff(self.ticks1,self.ticks2)
            
        else:
            
            ticksdiff = 0
        
        self.ticks2 = self.ticks1
        
        self.yaw_rate = self.line_ctrl.calculate(0, dis_from_line, (ticksdiff * 1e-6))
        
        omegaL = (self.v/self.r_w)+((self.L*self.yaw_rate)/(2*self.r_w))
        
        omegaR = (self.v/self.r_w)-((self.L*self.yaw_rate)/(2*self.r_w))
        
        self.ENCL.update()
        self.ENCR.update()
        
        omegaL_duty = self.omega_L_ctrl.calculate(omegaL, self.ENCL.get_velocity(), (ticksdiff * 1e-6))
        
        omegaR_duty = self.omega_R_ctrl.calculate(omegaR, self.ENCR.get_velocity(), (ticksdiff * 1e-6))
            
        self.prev_omegaR_duty = omegaR_duty
        
        self.prev_omegaL_duty = omegaL_duty

        self.PWMR.pulse_width_percent(omegaR_duty)
        self.PWML.pulse_width_percent(omegaL_duty)
            
        if self.IR_Front.value() == 0:
            self.wall = True
            
        def stop(self):
            
            self.PWMR.pulse_width_percent(0)
            self.PWML.pulse_width_percent(0)     
            self.toggle_variable = 0
            self.ENCL.zero()
            self.ENCR.zero()
    

   
if __name__ == "__main__":
    
    follow_line_task = follow_track()
    
    task1 = cotask.Task(follow_line_task.run, name="Line Follow Task", priority=1, period=100)
    
    cotask.task_list.append(task1)
   
    # Run the memory garbage collector to ensure memory is as defragmented as
    # possible before the real-time scheduler is started
    
    gc.collect()
    
    # Run the scheduler with the chosen scheduling algorithm. Quit if ^C pressed
    while True:
        try:
            cotask.task_list.pri_sched()
        except KeyboardInterrupt:
            break
            
