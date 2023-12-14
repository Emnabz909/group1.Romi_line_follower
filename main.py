#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Nov 20 12:23:04 2023

@author: elliottbryniarski
"""

import pyb
import time
import Encoder
import BNO055

# Variable to be toggled
toggle_variable = False

# Callback function to toggle the variable
def button_callback(line):
    global toggle_variable
    toggle_variable = not toggle_variable

   
if __name__ == "__main__":
    
    # Constants
    r_w = 0.035 # Radius of Wheels
    L = 0.149   # Width of Car
    v = 0.55 # Set Velocity of Car
    
    i2c = pyb.I2C(1, pyb.I2C.MASTER)
    i2c.init(pyb.I2C.CONTROLLER, baudrate=400_000)
    
    bno = BNO055.BNO055()
    
    bno.begin()
    
    cal_data = [95, 0, 11, 0, 235, 255, 62, 245, 150, 252, 67, 8, 1, 0, 254, 255, 255, 255, 232, 3, 211, 1]
    bno.setCalibration(cal_data)
    
    Mot_R_EN = pyb.Pin.cpu.C2
    Mot_R_DIR = pyb.Pin.cpu.C6
    Mot_R_TIM = pyb.Pin.cpu.C7
    
    Mot_L_EN = pyb.Pin.cpu.B7
    Mot_L_DIR = pyb.Pin.cpu.C5
    Mot_L_TIM = pyb.Pin.cpu.B6
    
    timR = pyb.Timer(8, freq = 20_000)
    timL = pyb.Timer(4, freq = 20_000)
    
    PWMR = timR.channel(2, mode = pyb.Timer.PWM, pin = Mot_R_TIM)
    PWML = timL.channel(1, mode = pyb.Timer.PWM, pin = Mot_L_TIM)
    
    ENC_L_A = pyb.Pin.cpu.A0
    ENC_L_B = pyb.Pin.cpu.A1
    
    ENC_L_tim = pyb.Timer(2, period = 65535, prescaler = 0)
    
    ENC_R_A = pyb.Pin.cpu.B4
    ENC_R_B = pyb.Pin.cpu.B5
    
    ENC_R_tim = pyb.Timer(3, period = 65535, prescaler = 0)
    
    ENCR = Encoder.Encoder(ENC_R_tim, ENC_R_A, ENC_R_B)
    ENCL = Encoder.Encoder(ENC_L_tim, ENC_L_A, ENC_L_B)
    
    L3_sens_adc = pyb.ADC(pyb.Pin.cpu.C0)
    L2_sens_adc = pyb.ADC(pyb.Pin.cpu.B0)
    L1_sens_adc = pyb.ADC(pyb.Pin.cpu.A4)
    L_sens_adc = pyb.ADC(pyb.Pin.cpu.C4)
    
    R3_sens_adc = pyb.ADC(pyb.Pin.cpu.C1)
    R2_sens_adc = pyb.ADC(pyb.Pin.cpu.C3)
    R1_sens_adc = pyb.ADC(pyb.Pin.cpu.A5)
    R_sens_adc = pyb.ADC(pyb.Pin.cpu.B1)
    
    L_sens_EN = pyb.Pin.cpu.C3 
    R_sens_EN = pyb.Pin.cpu.C8
    
    L_sens_EN.high()
    R_sens_EN.high()
    
    sens_p_gain = 3
    sens_i_gain = 0.15
    sens_d_gain = 0.4
    
    line_ctrl = Encoder.ClosedLoop(sens_p_gain, sens_i_gain, sens_d_gain)
    
    omega_p_gain = 2.95
    omega_i_gain = 0.1
    omega_d_gain = 0
    
    omega_L_ctrl = Encoder.ClosedLoop(omega_p_gain, omega_i_gain, omega_d_gain)
    omega_R_ctrl = Encoder.ClosedLoop(omega_p_gain, omega_i_gain, omega_d_gain)
    
    ENCL.update()
    ENCR.update()
    
    count = 0
    ticks2 = 0
    
    button_pin = pyb.Pin.cpu.C13

    extint = pyb.ExtInt(button_pin, pyb.ExtInt.IRQ_FALLING, pyb.Pin.PULL_UP, button_callback)
    
    while True:
        
        if toggle_variable == 1:
            
            time1 = time.ticks_ms()
            
            #prev_dis_from_line = 0
            
            #line_change = 0
            
            prev_omegaR_duty = 0
            
            prev_omegaL_duty = 0
            
            time_slow2 = 0
            
            time_slow = 0
            
            while True:

                if L3_sens_adc.read() > 4000 and R3_sens_adc.read() > 4000:
                    dis_from_line = 0
                elif L3_sens_adc.read() > 4000:
                    dis_from_line = 3
                elif R3_sens_adc.read() > 4000:
                    dis_from_line = -3
                elif L3_sens_adc.read() > 4000 and L2_sens_adc.read() > 4000:
                    dis_from_line = 2.5
                elif R3_sens_adc.read() > 4000 and R2_sens_adc.read() > 4000:
                    dis_from_line = -2.5
                elif L2_sens_adc.read() > 4000:
                    dis_from_line = 2
                elif R2_sens_adc.read() > 4000:
                    dis_from_line = -2
                elif L2_sens_adc.read() > 4000 and L1_sens_adc.read() > 4000:
                    dis_from_line = 1.5
                elif R2_sens_adc.read() > 4000 and R1_sens_adc.read() > 4000:
                    dis_from_line = -1.5
                elif L1_sens_adc.read() > 4000:
                    dis_from_line = 1
                elif R1_sens_adc.read() > 4000:
                    dis_from_line = -1
                elif L1_sens_adc.read() > 4000 and L_sens_adc.read() > 4000:
                    dis_from_line = 0.5
                elif R1_sens_adc.read() > 4000 and R_sens_adc.read() > 4000:
                    dis_from_line = -0.5
                elif L_sens_adc.read() > 4000:
                    dis_from_line = 0
                elif R_sens_adc.read() > 4000:
                    dis_from_line = 0
                else:
                    dis_from_line = 0
                    
                x, y, z = bno.getVector(BNO055.BNO055.VECTOR_GYROSCOPE)
            
                if abs(z) >= 1.5:
                    v = 0.07
                    sens_p_gain = 3
                    sens_i_gain = 1.3
                    sens_d_gain = 0.9
                    
                    time_slow = time.ticks_ms()
                elif abs(z) < 0.3 and (time.ticks_diff(time_slow2,time_slow)*1e-3) > 0.4 and time_slow2 != 0 and time_slow != 0:
                    v = 0.55
                    sens_p_gain = 3
                    sens_i_gain = 0.3
                    sens_d_gain = 0.4
                
                time_slow2 = time.ticks_ms()
                
                print(abs(z))
                
                count += 1
                
                ticks1 = time.ticks_us()
                
                if count > 1:
                    
                    ticksdiff = time.ticks_diff(ticks1,ticks2)
                    
                else:
                    
                    ticksdiff = 0
                
                ticks2 = ticks1
                
                yaw_rate = line_ctrl.calculate(0, dis_from_line, (ticksdiff * 1e-6))
                
                omegaL = (v/r_w)+((L*yaw_rate)/(2*r_w))
                
                omegaR = (v/r_w)-((L*yaw_rate)/(2*r_w))
                
                ENCL.update()
                ENCR.update()
                
                omegaL_duty = omega_L_ctrl.calculate(omegaL, ENCL.get_velocity(), (ticksdiff * 1e-6))
                
                omegaR_duty = omega_R_ctrl.calculate(omegaR, ENCR.get_velocity(), (ticksdiff * 1e-6))
                    
                prev_omegaR_duty = omegaR_duty
                
                prev_omegaL_duty = omegaL_duty

                PWMR.pulse_width_percent(omegaR_duty)
                PWML.pulse_width_percent(omegaL_duty)
                
                #print(f'Velocity: {ENCL.get_velocity() * r_w}')
                #print(f'Time between readings: {freq * 1e-6}')
                
                #if ENCL.get_position() < 62247 and ENCL.get_position() != 0:
                    #time2 = time.ticks_ms()
                    #break
                
            PWMR.pulse_width_percent(0)
            PWML.pulse_width_percent(0)     
            toggle_variable = 0
            ENCL.zero()
            ENCR.zero()
            
            #timediff = time.ticks_diff(time2,time1) * 1e-3
            
            #print(f'Time Error = {2 - timediff}')
            