import numpy as np
import math
import time
from matplotlib import pyplot as plt

from MapBuilder import MapBuilder

class SensorModel:

    """
    References: Thrun, Sebastian, Wolfram Burgard, and Dieter Fox. Probabilistic robotics. MIT press, 2005. [pp 153-168]
    """

    def __init__(self, src_path_map):

        self._map_obj = MapBuilder(src_path_map)

        self._z_hit = 5
        self._z_short = 1
        self._z_max = .1
        self._z_rand = 3

        self._sigma_hit = 30
        self._lambda_short = .02
    
        self._max_range = 1000
        self._min_probability = 0.25
        self._subsampling = 10

        self._norm_wts = 1.0 / (self._z_hit + self._z_short + self._z_max + self._z_rand)

    def get_phit(self, z_t1, zstar_t1):
        if (z_t1 <= self._max_range):
            eta = 1 # TODO
            probability = eta * 1.0/math.sqrt(2*math.pi*self._sigma_hit**2) * math.e**( -0.5*(z_t1-zstar_t1)**2 / self._sigma_hit**2)
        else:
            probability = 0
        return probability

    def get_pshort(self, z_t1, zstar_t1):
        if (z_t1 <= self._max_range):
            eta = 1.0/(1 - math.e**(-self._lambda_short*zstar_t1))
            probability = eta*self._lambda_short*math.e**(-self._lambda_short*z_t1)
        else:
            probability = 0
        return probability

    def get_pmax(self, z_t1, zstar_t1):
        if (z_t1 >= 0.98*self._max_range):
            probability = 1.0
        else:
            probability = 0
        return probability

    def get_prand(self, z_t1, zstar_t1):
        if (z_t1 <= self._max_range):
            probability = 1.0/self._max_range
        else:
            probability = 0
        return probability

    def get_pmixture(self, z_t1, zstar_t1):
        prob_zt1 = self._norm_wts * (self._z_hit*self.get_phit(z_t1, zstar_t1) + self._z_short*self.get_pshort(z_t1, zstar_t1) + \
                        self._z_max*self.get_pmax(z_t1, zstar_t1) + self._z_rand*self.get_prand(z_t1, zstar_t1))
        return prob_zt1

    def beam_range_finder_model(self, z_t1_arr, x_t1):
        """
        param[in] z_t1_arr : laser range readings [array of 180 values] at time t
        param[in] x_t1 : particle state belief [x, y, theta] at time t [world_frame]
        param[out] prob_zt1 : likelihood of a range scan zt1 at time t
        """
        xl_t1 = self.particle_to_laser_tf(x_t1)
        zstar_t1_arr = self.rayTrace(xl_t1, self._map_obj, self._min_probability)

        q = 1; j = 0
        for k in range(1,180):
            if ( k % self._subsampling != 0 ):
                continue
            z_t1 = z_t1_arr[k]
            zstar_t1 = zstar_t1_arr[j]; j = j+1
            prob_zt1 = self.get_pmixture(z_t1, zstar_t1)
            q = q*prob_zt1

        return q

    def plot_prob_zt1(self):
        prob_zt1 = []
        arr_zt1 = range(0, self._max_range + 1)
        zstar_t1 = 200
        for z_t1 in arr_zt1:
            prob_zt1.append( self.get_pmixture(z_t1, zstar_t1) )

        fig = plt.figure()
        plt.ion(); plt.plot(arr_zt1, prob_zt1); plt.axis([0, self._max_range, 0, 0.05]); plt.draw()
        time.sleep(100)

    def particle_to_laser_tf(self, x_t1):
        """
        The laser on the robot is approximately 25 cm offset forward from the true center of the robot.
        """
        xl_t1_x = x_t1[0] + 25*math.cos(x_t1[2])
        xl_t1_y = x_t1[1] + 25*math.sin(x_t1[2]) #check this with Paloma!        
        xl_t1_theta = x_t1[2]
        xl_t1 = [xl_t1_x, xl_t1_y, xl_t1_theta]
        return xl_t1 

    def rayTrace(self, xl_t1, mapObj, minProbability):
        """
        param[in] xl_t1 : particle state belief [x, y, theta] shifted to laser position at time t [world_frame]
        param[in] mapObj : object of the MapBuilder class 
        param[out] minProbability : minimum threshold for detecting an obstacle in the occGrid
        """
        occGrid = mapObj.getMap()
        heading = xl_t1[2]
        laserRange = []

        for i in xrange(180):  # 180 degree sweep in laser scan with 1 degree increments

            if ( i % self._subsampling !=0 ):
                continue

            # Initialize current location to origin of laser
            x_current = xl_t1[0]/10 # MODIFIED TO BE IN IMAGE COORDS
            y_current = xl_t1[1]/10 # MODIFIED TO BE IN IMAGE COORDS
            # Set angle of laser, moving clockwise
            laserDir = heading + math.radians(i - 90)
            dy = math.sin(laserDir)
            dx = math.cos(laserDir)

            for d in xrange(101):
                # Check if the current location is in a cell that is occupied
                if occGrid[round(x_current), round(y_current)] > minProbability:  # round to the resolution of the occupancy grid
                    laserRange.append(d*10)  # multiply by 10 to convert from occ grid pixel size of 10 cm to cm
                    break
                # Walk along the ray until you find an obstacle
                x_current += dx
                y_current += dy
                # Check if walked outside map area
                if x_current > 799 or y_current > 799 or x_current < 0 or y_current < 0:
                    laserRange.append(1000)
                    break

            # Check if reached the end of the range without detecting obstacle, set range to 10 if true
            if d == 100:
                laserRange.append(d * 10)

        return laserRange

if __name__=='__main__':

    # src_path_map = '../map/wean.dat'
    # sensor1 = SensorModel(src_path_map)
    # sensor1.plot_prob_zt1()

    pass