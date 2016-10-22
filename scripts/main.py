import numpy as np
import sys
import pdb

from MotionModel import MotionModel
from MapBuilder import MapBuilder
from SensorModel import SensorModel
from Resampling import Resampling

from matplotlib import pyplot as plt
from matplotlib import figure as fig
import time

from datetime import datetime


# ----------------------------FUNCTIONS TO DISPLAY THE MAP--------------------------------

def mapInit(mapList):
    fig = plt.figure()
    plt.switch_backend('TkAgg')  # this addition is so the display works on Merritt's computer..
    mng = plt.get_current_fig_manager();  # mng.resize(*mng.window.maxsize())
    plt.ion(); plt.imshow(mapList, cmap='Greys'); plt.axis([0, 800, 0, 800]);


def mapShow(mapList, X_bar):
    x_locs = [item[0][0]/10 for item in X_bar]
    y_locs = [item[0][1]/10 for item in X_bar]
    scat = plt.scatter(x_locs, y_locs, c='r', marker='o')
    plt.pause(0.00001)
    scat.remove()

def mapShowScaledWts(mapList, X_bar):
    x_locs = [item[0][0]/10 for item in X_bar]
    y_locs = [item[0][1]/10 for item in X_bar]

    wts = [item[1]*1e8 for item in X_bar]
    # wts = (wts - np.min(wts))*10

    scat = plt.scatter(x_locs, y_locs, c='r', marker='o')
    plt.pause(0.00001)
    scat.remove()

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


results_L = results_L[14:-1]
results_O = results_O[14:-1]            

# -------------------------------MAIN SCRIPT----------------------------------

# -----------------initialize variables-------------------
alpha1 = .0001
alpha2 = .0001
alpha3 = 0.01
alpha4 = 0.01

mot = MotionModel(alpha1, alpha2, alpha3, alpha4)

M = 2000  # number of particles

map = MapBuilder('../map/wean.dat')
mapList = map.getMap()
mapInit(mapList)

sensorModel = SensorModel('../map/wean.dat')
resampler = Resampling()


# ------------initialize particles throughout map--------
# generate list of locations with '0' value
counter = 0
goodLocs_x = []
goodLocs_y = []
for i in range(0,800):
    for j in range(0,800):
        if mapList[i][j]==0:
            counter = counter+1
            goodLocs_x.append(i)
            goodLocs_y.append(j)

# randomly select from those locations with '0' value
p_x = []
p_y = []
for i in range(0,M):
    num_rand=np.random.randint(1,counter)
    #p_x.append(np.random.randint(4000,4500)) # uncomment this if you want a cluster near the robot start loc
    #p_y.append(np.random.randint(3900,4200))
    p_y.append(goodLocs_x[num_rand]*10) # x and y axes are flipped!!! multiplied by 10 to convert to cm
    p_x.append(goodLocs_y[num_rand]*10) # x and y axes are flipped!!!

p_theta = np.random.uniform(-3.14,3.14,M) # change 3.10 to -3.14 when script is fully debugged

#----------------------main loop-------------------------

for t in range(1, 1000):
# for t in range(1, len(results_O)):
    # a = datetime.now()
    X_bar = []

    # assign u_t
    u_t0 = results_O[t-1][0:3]
    u_t1 = results_O[t][0:3]  # this is just x,y,theta (no timestep)

    # assign z_t
    z_t = results_L[t]  # all 180 scans at each timestep

    for m in range(0,M):
        # pull x_t from motion model
        if t == 1:
            x_t0 = [p_x[m], p_y[m], p_theta[m]]  # assign x_t0 as the particles from random initialization
        else:
            x_t0 = X_bar_save[m][0]			

        x_t1 = mot.sample_motion_model(u_t0, u_t1, x_t0)

        # pull w_t from sensor model
        w_t = sensorModel.beam_range_finder_model(z_t, x_t1)

        # update X_bar each timestep
        list = [x_t1, w_t]
        X_bar.append(list)


    X_bar = resampler.low_variance_sampler(X_bar)

    X_bar_save = X_bar  # this is so I can access X_bar in the next iteration

    if t % 10 == 1:
        mapShow(mapList, X_bar)




