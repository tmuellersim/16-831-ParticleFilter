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
    # mng.window.state('zoomed')  # this is also so it works on Merritt's computer...
    plt.ion(); plt.imshow(mapList, cmap='Greys'); plt.axis([0, 800, 0, 800]);


def mapShow(mapList, X_bar, t):
    # start = time.clock()
    x_locs = [item[0][0]/10 for item in X_bar]
    y_locs = [item[0][1]/10 for item in X_bar]
    scat = plt.scatter(x_locs, y_locs, c='r', marker='o')
    # end = time.clock()
    # print "%.2gs" % (end-start)
    plt.pause(0.00001)
    plt.savefig('./images/image' + '%d' % t + '.png', bbox_inches='tight')
    scat.remove()

def mapShowScaledWts(mapList, X_bar):
    # start = time.clock()
    x_locs = [item[0][0]/10 for item in X_bar]
    y_locs = [item[0][1]/10 for item in X_bar]

    wts = [item[1]*1e8 for item in X_bar]
    # wts = (wts - np.min(wts))*10

    scat = plt.scatter(x_locs, y_locs, c='r', marker='o')
    # end = time.clock()
    # print "%.2gs" % (end-start)
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
            results_O.append([floats[0], floats[1], floats[2], floats[186]])  # x_loc, y_loc, angle, timestamp
            results_L_loc.append([floats[3], floats[4], floats[5], floats[186]])  # x_loc, y_loc, angle, timestamp
            results_L.append(floats[6:185])  # 180 laser scans


# results_L = results_L[20:-1]            

# -------------------------------MAIN SCRIPT----------------------------------

# -----------------initialize variables-------------------
alpha1 = 1e-4
alpha2 = 0
alpha3 = 5e-1  # increasing these two variables appears to make the particles move farther
alpha4 = 0

mot = MotionModel(alpha1, alpha2, alpha3, alpha4)

M = 1000  # number of particles

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
for i in range(0, 800):
    for j in range(0, 800):
        if mapList[i][j] == 0:
            counter = counter + 1
            goodLocs_x.append(i)
            goodLocs_y.append(j)

# randomly select from those locations with '0' value
p_x = []
p_y = []
for i in range(0, M):
    num_rand = np.random.randint(1, counter)
    # p_x.append(np.random.randint(4000,4050)) # uncomment this if you want a cluster near the robot start loc
    # p_y.append(np.random.randint(4000,4050))
    p_y.append(goodLocs_x[num_rand]*10)  # x and y axes are flipped!!! multiplied by 10 to convert to cm
    p_x.append(goodLocs_y[num_rand]*10)  # x and y axes are flipped!!!

p_theta = np.random.uniform(-3.14, 3.14, M) # change 3.10 to -3.14 when script is fully debugged

#----------------------main loop-------------------------

for t in range(1, 25):
# for t in range(1, len(results_O)):
    # a = datetime.now()
    X_bar = []

    # calculate u_t
    u_t0 = results_O[t-1][0:3]
    u_t1 = results_O[t][0:3]  # this is just x,y,theta (no timestep)

    # calculate w_t (will use later)
    z_t = results_L[t]  # all 180 scans at each timestep


    # Stack all particles into one numpy array and pass them into each individual function
    if t == 1:
        a = np.asarray(p_x)
        b = np.asarray(p_y)
        c = np.asarray(p_theta)
        x_t0_array = np.concatenate((a[:, None], b[:, None], c[:, None]), axis=1)
    else:
        x_t0_array = np.asarray([x[0] for x in X_bar_save])

    # Pass array of particles into motion model
    x_t1_array = mot.sample_motion_model_array(u_t0, u_t1, x_t0_array)

    # Pull w_t from sensor model
    w_t = sensorModel.beam_range_finder_model_array(z_t, x_t1_array)

    for i in xrange(len(x_t1_array[0])):
        list = [[x[i] for x in x_t1_array], w_t[i]]
        X_bar.append(list)

    # X_bar = resampler.multinomial_sampler(X_bar)
    X_bar = resampler.low_variance_sampler(X_bar)

    X_bar_save = X_bar  # this is so I can access X_bar in the next iteration

    # HERE WE IMPLEMENT THE IMPORTANCE SAMPLING, BUT ERIC SAID WE SHOULD FIRST TRY WITHOUT

    # if t % 10 == 1:
    mapShow(mapList, X_bar, t)
    print 't is: %d' % t




