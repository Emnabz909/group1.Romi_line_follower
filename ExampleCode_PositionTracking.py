#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Dec 15 14:41:01 2023

@author: elliottbryniarski
"""

from math import cos

x, y, z = bno.getVector(BNO055.VECTOR_EULER)

ticks1 = time.ticks_us()

V = ENCL.get_velocity()

delta_t = ticksdiff(ticks1,ticks2)

X_pos = V * delta_t * cos(x) + X_pos

Y_pos = V * delta_t * sin(x) + Y_pos