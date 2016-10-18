import numpy as np
import sys

from MotionModel import MotionModel
from MapBuilder import MapBuilder

from matplotlib import pyplot as plt
from matplotlib import figure as fig
import time

#----------------------------FUNCTION TO SHOW THE MAP--------------------------------

def mapInit(mapList):
	fig = plt.figure()
	plt.switch_backend('TkAgg') #this addition is so the display works on Merritt's computer..
	mng = plt.get_current_fig_manager(); #mng.resize(*mng.window.maxsize())
	mng.window.state('zoomed') #this is also so it works on Merritt's computer...
	plt.ion(); plt.imshow(mapList, cmap='Greys'); plt.axis([0, 800, 0, 800]);


def mapShow(mapList, X_bar):
	x_locs = [item[0][0] for item in X_bar];
	y_locs = [item[0][1] for item in X_bar]
	scat = plt.scatter(x_locs, y_locs, c='r', marker='o')
	#plt.draw()
	plt.pause(0.05)
	scat.remove()




#-------------------------------PARSE THE DATA FILE----------------------------------
#this parses robotdata1 into three lists: odometry, laser position and time, and laser readings

filename = '../log/robotdata1.log' 

txt = open(filename)

results_O = []
results_L_loc = []
results_L = []

with open('robotdata1.log') as inputfile:
    for line in inputfile:
		if line[0] is 'O':
			temp = 'nothing'
			#FOR NOW WE JUST LOOK AT ODOMETRY WHEN THE LASER SCAN COMES IN
			#s = line[2:-1]
			#floats = [float(x) for x in s.split()]
			#results_O.append(floats)

		else:
			s = line[2:-1]
			floats = [float(x) for x in s.split()]
			results_O.append([floats[0], floats[1], floats[2], floats[186]]) #x_loc, y_loc, angle, timestamp
			results_L_loc.append([floats[3], floats[4], floats[5], floats[186]]) #x_loc, y_loc, angle, timestamp
			results_L.append(floats[6:185]) #180 laser scans
			
			
			
#-------------------------------MAIN SCRIPT----------------------------------

#-----------------initialize variables-------------------
alpha1 = 1
alpha2 = 1
alpha3 = 1
alpha4 = 1
mot=MotionModel(alpha1, alpha2, alpha3, alpha4)

M=1000 #number of particles

#initialize particle locations
p_x = np.random.randint(400,450,M) #change this to match the map
p_y = np.random.randint(100,600,M) #change this to match the map
p_theta = np.random.uniform(-3.14,3.14,M)

map=MapBuilder('../map/wean.dat')
mapList = map.getMap()
mapInit(mapList)

#--------------------------------------------------------

for t in range(1, len(results_O)):

	X_bar = []

	#calculate u_t
	u_t0 = results_O[t-1][0:3] 
	u_t1 = results_O[t][0:3] #this is just x,y,theta (no timestep)
	
	#calculate z_t (will use later)
	#z_t = results_L[t]; #all 180 scans at each timestep
	
	for m in range(0,M):
		#pull x_t from motion model
		if (t==1):
			x_t0 = [p_x[m], p_y[m], p_theta[m]] #assign x_t0 as the particles from random initialization
		else:
			x_t0 = x_t1 #assign x_t0 as the old x_t
			
		x_t1 = mot.sample_motion_model(u_t0, u_t1, x_t0)
		
		#pull w_t from sensor model
		w_t = 1.0; #set at 1 right now, all weights equal
		
		#update X_bar each timestep
		list = [x_t1, w_t]
		X_bar.append(list)
		
		
	#HERE WE IMPLEMENT THE IMPORTANCE SAMPLING, BUT ERIC SAID WE SHOULD FIRST TRY WITHOUT	
	#
	#
	
	
	mapShow(mapList, X_bar)




