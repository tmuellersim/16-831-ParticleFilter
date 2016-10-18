import numpy as np

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

#initalize X_bar_t
X_bar = [[[0,0,0], 0.0]]*len(results_O) #tuple of x,y,theta and w

M=1000 #number of particles

#initialize particle locations
p_x = np.random.uniform(0,800,M) #800 is hard coded here
p_y = np.random.uniform(0,800,M)
p_theta = np.random.uniform(0,800,M)

for t in range(1, len(results_O)):

	#calculate u_t
	u_t = results_O[0:2] #this is just x,y,theta (no timestep)
	
	#calculate z_t
	z_t = results_L[t]; #all 180 scans at each timestep
	
	for i in range(0,M):
		#pull x_t from motion model
		x_t = [49, 2, 34] #just a placeholder
		
		#pull w_t from sensor model
		w_t = 1.5; #just a placeholder
		
		#update X_bar_t
		X_bar[t][0] = [x + y for x, y in zip(X_bar[t][0], x_t)]
		X_bar[t][1] = X_bar[t][1] + w_t
	
print X_bar[0]	
		
		
	