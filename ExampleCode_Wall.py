#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Dec 15 14:39:49 2023

@author: elliottbryniarski
"""
            
if wall == True:
    
    x, y, z = bno.getVector(BNO055.VECTOR_EULER)
    
    wall_dir = x
    
    while abs(wall_dir - wall_dir_start) < 90:
        
        x, y, z = bno.getVector(BNO055.VECTOR_EULER)
        
        print(x)
        
        wall_dir = x
        
        Mot_R_DIR.high()
        
        PWML.pulse_width_percent(50)
        PWMR.pulse_width_percent(50)
        
    wall_perp = True
    print('Turned')
    wall = False
    Mot_R_DIR.low()
    
elif wall_perp == True:
    
    while IR_side.value() == 0:
        
        if L3_sens_adc.read() > 4000 and R3_sens_adc.read() > 4000:
            wall = False
            wall_perp = False
            wall_done = True
            
        PWML.pulse_width_percent(30)
        PWMR.pulse_width_percent(30)
        
    single_wall_done = True
    print('wall done')
    wall_perp = False
    
elif single_wall_done == True:
    
    x, y, z = bno.getVector(BNO055.VECTOR_EULER)
    
    wall_dir = x
    
    while abs(wall_dir - wall_dir_start) < 90:
        
        x, y, z = bno.getVector(BNO055.VECTOR_EULER)
        
        wall_dir = x
        
        PWML.pulse_width_percent(0)
        PWMR.pulse_width_percent(20)
        
    single_wall_done = False
    print('turned')
    wall_perp = True

elif wall_done == True:
    
    x, y, z = bno.getVector(BNO055.VECTOR_EULER)
    
    wall_dir = x
    
    while abs(wall_dir - wall_dir_start) < 90:
        
        x, y, z = bno.getVector(BNO055.VECTOR_EULER)
        
        wall_dir = x
        
        Mot_R_DIR.high()
        
        PWML.pulse_width_percent(50)
        PWMR.pulse_width_percent(50)
        
    wall_perp = False
    wall = False
    wall_done = False
    single_wall_done = False
    print('back on line')
    wall = False
    
    Mot_R_DIR.low()
            
