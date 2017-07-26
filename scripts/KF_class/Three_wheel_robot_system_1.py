#! /usr/bin/python

import math
from math import pow,atan2,sqrt,cos,sin,asin,pi


# linear system definition

#Sample tm
dt = 0.00006

# lin sitem Type  is   x' = Ax + Bu
# and its output is    y = Cx
# so for a 4 state system we have the following:

#State transition matrix
A = [[1,0,0],
     [0,1,0],
     [0,0,1]]

#control matrix
B = [[dt,0,0],
     [0,dt,0],
     [0,0,dt]]


# C matrix 
C = [[1,0,0],
     [0,1,0],
     [0,0,1]]


# gain matrix 
F1 = -0.8          #theta x
F2 = -0.006        #omega x
F3 = -0.8          #theta y
F4 = -0.006        #omega y
F5 = -0.8          #theta z
F6 = -0.006        #omega z


F  = [[F1,0,0,F2,0,0],
      [0,F3,0,0,F4,0],
      [0,0,F5,0,0,F6]]