import numpy as np
import sys

from MotionModel import MotionModel
from MapBuilder import MapBuilder
from SensorModel import SensorModel

from matplotlib import pyplot as plt
from matplotlib import figure as fig
import time

from datetime import datetime


# -------------------------------PARSE THE DATA FILE----------------------------------
# this parses robotdata1 into three lists: odometry, laser position and time, and laser readings

filename = '../log/robotdata1.log' 

txt = open(filename)

results_O = []
results_L_loc = []
results_L = []
mod_results_O = []

with open('robotdata1.log') as inputfile:
    for line in inputfile:
        if line[0] is 'O':
            temp = 'nothing'
            # FOR NOW WE JUST LOOK AT ODOMETRY WHEN THE LASER SCAN COMES IN
            # s = line[2:-1]
            # floats = [float(x) for x in s.split()]
            # results_O.append(floats)

        else:
            s = line[2:-1]
            floats = [float(x) for x in s.split()]
            results_O.append([floats[0], floats[1], floats[2], floats[186]]) # x_loc, y_loc, angle, timestamp
            results_L_loc.append([floats[3], floats[4], floats[5], floats[186]]) # x_loc, y_loc, angle, timestamp
            results_L.append(floats[6:185]) # 180 laser scans		


# -------------------------------MAIN SCRIPT----------------------------------

sensorModel = SensorModel('../map/wean.dat')

#----------------------main loop-------------------------

for other in range(0,12): # THE BOOK SUGGESTS 12 ITERATIONS
    for t in range(1, len(results_O)):       
        (sigma_hit, lambda_short) = sensorModel.learn_params(results_L[t], results_O[t])  

    print (sigma_hit, lambda_short)
