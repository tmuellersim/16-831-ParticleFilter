import numpy as np

from MotionModel import MotionModel
from MapBuilder import MapBuilder

#-------------------------------PARSE THE DATA FILE----------------------------------
#this parses robotdata1 into three lists: odometry, laser position and time, and laser readings

#filename = '../robotdata1.log' 
#filename = 'data/log/robotdata1/robotdata1.log' 
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
			results_O.append([floats[0], floats[1], floats[2], floats[5]])
			results_L_loc.append([floats[3], floats[4], floats[5]])
			results_L.append(floats[6:-1])
			

#-------------------------------MAIN SCRIPT----------------------------------

#-----------------initialize variables-------------------
alpha1 = 0.1
alpha2 = 0.1
alpha3 = 0.1
alpha4 = 0.1
mot=MotionModel(alpha1, alpha2, alpha3, alpha4)
map=MapBuilder('../map/wean.dat') #I can't seem to call the map

M=100 #number of particles

#initialize particle locations
p_x = np.random.uniform(0,800,M) #change this to match the map
p_y = np.random.uniform(0,800,M) #change this to match the map
p_theta = np.random.uniform(-3.14,3.14,M)

#--------------------------------------------------------

for t in range(1, len(results_O)):

	X_bar = [[[0,0,0], 0.0]]*M #list of lists of x,y,theta and w

	#calculate u_t
	u_t0 = results_O[t-1][0:3] 
	u_t1 = results_O[t][0:3] #this is just x,y,theta (no timestep)
	
	#calculate z_t
	z_t = results_L[t]; #all 180 scans at each timestep
	
	for m in range(0,M):
		#pull x_t from motion model
		x_t0 = [p_x[0], p_y[0], p_theta[0]] #assign x_t0 as one of the particles from random initialization
		x_t1 = mot.sample_motion_model(u_t0, u_t1, x_t0)
		
		#pull w_t from sensor model
		w_t = 1.0; #set at 1 right now, all weights equal
		
		#update X_bar_t
		X_bar[m][0] = x_t1   #[a + b for a, b in zip(X_bar[t][0], x_t1)]
		X_bar[m][1] = w_t   #X_bar[t][1] + w_t
		
		
	#HERE WE IMPLEMENT THE IMPORTANCE SAMPLING, BUT ERIC SAID WE SHOULD FIRST TRY WITHOUT	
	#
	#
	
print X_bar[50]	

#map.visualize_map()		
		
	