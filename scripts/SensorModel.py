import numpy as np
import math
import time
from matplotlib import pyplot as plt
from scipy.stats import norm
import pdb

from MapBuilder import MapBuilder

class SensorModel:

    """
    References: Thrun, Sebastian, Wolfram Burgard, and Dieter Fox. Probabilistic robotics. MIT press, 2005. [pp 153-168]
    """

    def __init__(self, src_path_map):

        self._map_obj = MapBuilder(src_path_map)

        self._z_hit = 1 #1
        self._z_short = 0.01 #.01
        self._z_max = 0.01 #.01
        self._z_rand = 990 #1000

        self._sigma_hit = 50  # 100
        self._lambda_short = 1 / 60.0
    
        self._max_range = 1000
        self._min_probability = 0.35
        self._subsampling = 2

        # self._norm_wts = 1.0 / (self._z_hit + self._z_short + self._z_max + self._z_rand)
        self._norm_wts = 1.0

    def get_phit(self, z_t1, zstar_t1):
        if (z_t1 <= self._max_range):
            eta = 1  # norm((z_t1-zstar_t1), self._sigma_hit).cdf(self._max_range)  # Eta will nearly always be close to 1, remove to reduce computation time
            probability = eta * 1.0/math.sqrt(2*math.pi*self._sigma_hit**2) * math.e**( -0.5*(z_t1-zstar_t1)**2 / self._sigma_hit**2)
        else:
            probability = 0
        return probability

    def get_pshort(self, z_t1, zstar_t1):
        if (z_t1 <= zstar_t1):
            eta = 1.0 #1.0/(1 - math.e**(-self._lambda_short*zstar_t1))
            # probability = eta*self._lambda_short*math.e**(-self._lambda_short*z_t1)
            probability = eta * math.e ** (-self._lambda_short * z_t1)  # Removed lambda to make tuning a little more intuitive
        else:
            probability = 0
        return probability

    def get_pmax(self, z_t1, zstar_t1):
        if (z_t1 >= 0.99*self._max_range):
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

    def get_pmixture_array(self, z_t1, zstar_t1):
        prob_zt1 = self._norm_wts * (self._z_hit*self.get_phit_array(z_t1, zstar_t1) + self._z_short*self.get_pshort_array(z_t1, zstar_t1) + \
                        self._z_max*self.get_pmax_array(z_t1, zstar_t1) + self._z_rand*self.get_prand_array(z_t1, zstar_t1))
        return prob_zt1

    def get_phit_array(self, z_t1, zstar_t1):
        if (z_t1 <= self._max_range):
            eta = 1  # norm((z_t1-zstar_t1), self._sigma_hit).cdf(self._max_range)  # Eta will nearly always be close to 1, remove to reduce computation time
            probability = eta * 1.0 / math.sqrt(2 * math.pi * self._sigma_hit ** 2) * math.e ** (
            -0.5 * (z_t1 - zstar_t1) ** 2 / self._sigma_hit ** 2)
        else:
            probability = 1.0
        return probability

    def get_pshort_array(self, z_t1, zstar_t1):
        eta = 1.0  # 1.0/(1 - math.e**(-self._lambda_short*zstar_t1))
        probability = np.ones(len(zstar_t1)) * eta * math.e ** (-self._lambda_short * z_t1)
        probability[z_t1 <= zstar_t1] = 0
        return probability

    def get_pmax_array(self, z_t1, zstar_t1):
        if (z_t1 >= 0.99 * self._max_range):
            probability = np.ones(len(zstar_t1))
        else:
            probability = np.zeros(len(zstar_t1))
        return probability

    def get_prand_array(self, z_t1, zstar_t1):
        if (z_t1 <= self._max_range):
            probability = np.ones(len(zstar_t1)) / self._max_range
        else:
            probability = np.zeros(len(zstar_t1))
        return probability

    def beam_range_finder_model(self, z_t1_arr, x_t1):
        """
        param[in] z_t1_arr : laser range readings [array of 180 values] at time t
        param[in] x_t1 : particle state belief [x, y, theta] at time t [world_frame]
        param[out] prob_zt1 : likelihood of a range scan zt1 at time t
        """
        xl_t1 = self.particle_to_laser_tf(x_t1)
        zstar_t1_arr = self.rayTrace(xl_t1, self._map_obj, self._min_probability)

        q = 1
        j = 0
        for k in range(0, 180):
            if k % self._subsampling != 0:
                continue
            z_t1 = z_t1_arr[k]
            zstar_t1 = zstar_t1_arr[j]
            j += 1
            prob_zt1 = self.get_pmixture(z_t1, zstar_t1)

            # prob_zt1 = self.get_pmixture(z_t1, zstar_t1) * 10e2

            # The log of products is the sum of the log of each
            # q += math.log(prob_zt1)
            q *= prob_zt1

        return q    
        # return math.pow(math.e, q)  # This number may still be ridiculously small, check that it isn't zero

    def beam_range_finder_model_array(self, z_t1_arr, x_t1_array):
        """
        param[in] z_t1_arr : laser range readings [array of 180 values] at time t
        param[in] x_t1_array : array of particle state beliefs [x, y, theta] at time t [world_frame] [m x 3]
        param[out] prob_zt1_array : array of likelihood of a range scan zt1 at time t
        """
        xl_t1_array = self.particle_to_laser_tf_array(x_t1_array)

        zstar_t1_arr = self.rayTrace_array(xl_t1_array, self._map_obj, self._min_probability)

        q = np.ones(len(zstar_t1_arr))
        j = 0
        for k in xrange(0, 180, self._subsampling):
            z_t1 = z_t1_arr[k]
            zstar_t1_array = zstar_t1_arr[:, j]
            j += 1
            prob_zt1 = self.get_pmixture_array(z_t1, zstar_t1_array)

            # prob_zt1 = self.get_pmixture(z_t1, zstar_t1) * 10e2

            # The log of products is the sum of the log of each
            # q += math.log(prob_zt1)
            q *= prob_zt1

        return q
        # return math.pow(math.e, q)  # This number may still be ridiculously small, check that it isn't zero

    def get_eta(self, z_t1, zstar_t1):
        eta = math.pow((self.get_phit(z_t1, zstar_t1) + self.get_pshort(z_t1, zstar_t1) + self.get_pmax(z_t1, zstar_t1) + self.get_prand(z_t1, zstar_t1)), -1)
        return eta
        
    def learn_params(self, z_t1_arr, x_t1):
        """
        param[in] z_t1_arr : laser range readings [array of 180 values] at time t
        param[in] x_t1 : particle state belief [x, y, theta] at time t [world_frame]
        param[out] prob_zt1 : likelihood of a range scan zt1 at time t
        """
        
        #the next 9 lines of code are pretty much the same as beam_range_finder_model()
        xl_t1 = self.particle_to_laser_tf(x_t1)
        zstar_t1_arr = self.rayTrace(xl_t1, self._map_obj, self._min_probability)

        q = 1; j = 0
        e_hit = []; e_short = []; e_max = []; e_rand = []
        for k in range(0, 180):
            if k % self._subsampling != 0:
                continue
            z_t1 = z_t1_arr[k]
            zstar_t1 = zstar_t1_arr[j]
            
            eta = self.get_eta(z_t1, zstar_t1) #this is very similar to get_pmixture()
            e_hit.append(eta*self.get_phit(z_t1, zstar_t1))
            e_short.append(eta*self.get_pshort(z_t1, zstar_t1))
            e_max.append(eta*self.get_pmax(z_t1, zstar_t1))
            e_rand.append(eta*self.get_prand(z_t1, zstar_t1))
            j += 1
           
        # do we think that |Z|^-1 from lines 10 through 13 translates to (1.0/j)?
        self._z_hit = (1.0/(j+1))*sum(e_hit)
        self._z_short = (1.0/(j+1))*sum(e_short)
        self._z_max = (1.0/(j+1))*sum(e_max)
        self._z_rand = (1.0/(j+1))*sum(e_rand)

        # these are just operations for line 14 of the algorithm (is the square correct?)
        zi_minus_zstar = [a-b for a,b in zip(z_t1_arr, zstar_t1_arr)]
        zi_minus_zstar_square = [h*h for h in zi_minus_zstar]
        e_mult_zdiff = [c*d for c,d in zip(e_hit, zi_minus_zstar_square)]
        
        #implement lines 14 and 15 of algorithm
        self._sigma_hit = math.sqrt((1.0/sum(e_hit))*sum(e_mult_zdiff))
        self._lambda_short = sum(e_short)/sum([a*b for a,b in zip(e_short,z_t1_arr)])

        return(self._sigma_hit, self._lambda_short)

    def plot_prob_zt1(self):
        prob_zt1 = []
        arr_zt1 = range(0, self._max_range + 1)
        zstar_t1 = 500
        for z_t1 in arr_zt1:
            prob_zt1.append( self.get_pmixture(z_t1, zstar_t1) )

        fig = plt.figure()
        plt.ion()
        plt.plot(arr_zt1, prob_zt1)
        plt.draw()
        # plt.axis([0, self._max_range]);
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

    def particle_to_laser_tf_array(self, x_t1_array):
        """
        The laser on the robot is approximately 25 cm offset forward from the true center of the robot.
        """
        xl_t1_x = x_t1_array[0] + 25*np.cos(x_t1_array[2])
        xl_t1_y = x_t1_array[1] + 25*np.sin(x_t1_array[2]) #check this with Paloma!
        xl_t1_theta = x_t1_array[2]
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

        x_start = xl_t1[0] / 10  # MODIFIED TO BE IN IMAGE COORDS
        y_start = xl_t1[1] / 10  # MODIFIED TO BE IN IMAGE COORDS
        # Check that the particle isn't outside the map, put it on the edge if it is
        if x_start > 799:
            x_start = 799
        if x_start < 1:
            x_start = 1
        if y_start > 799:
            y_start = 799
        if y_start < 1:
            y_start = 1

        stepSize = 2
        subsampleRate = self._subsampling

        for i in xrange(0, 180, subsampleRate):  # 180 degree sweep in laser scan with 1 degree increments

            # if ( i % self._subsampling !=0 ):
            #     continue

            # Initialize current location to origin of laser
            x_current = x_start
            y_current = y_start

            # Set angle of laser, moving clockwise
            laserDir = heading + math.radians(i - 90)
            dy = math.sin(laserDir) * stepSize
            dx = math.cos(laserDir) * stepSize

            for d in xrange(101):
                # Check if the current location is in a cell that is occupied
                if occGrid[int(round(y_current)), int(
                        round(x_current))] > minProbability:  # round to the resolution of the occupancy grid
                    if d == 0:  # check for particle on a boundary, set range to some epsilon above 0
                        laserRange.append(25)
                        break
                    laserRange.append(d * 10)  # multiply by 10 to convert from occ grid pixel size of 10 cm to cm
                    break
                # Walk along the ray until you find an obstacle
                x_current += dx
                y_current += dy
                # Check if walked outside map area
                if x_current > 799 or y_current > 799 or x_current < 0 or y_current < 0:
                    laserRange.append(950)
                    break

            # End of the range reached without detecting obstacle, set range to 1000 if true
            laserRange.append(d * 10)

        return laserRange

    def rayTrace_array(self, xl_t1_array, mapObj, minProbability):
        """
        param[in] xl_t1 : particle state belief [x, y, theta] shifted to laser position at time t [world_frame]
        param[in] mapObj : object of the MapBuilder class
        param[in] minProbability : minimum threshold for detecting an obstacle in the occGrid
        """
        occGrid = mapObj.getMap()
        heading = xl_t1_array[2]

        x_start = xl_t1_array[0] / 10  # MODIFIED TO BE IN IMAGE COORDS
        y_start = xl_t1_array[1] / 10  # MODIFIED TO BE IN IMAGE COORDS
        # Check that the particle isn't outside the map, put it on the edge if it is
        x_start[x_start > 799] = 799
        x_start[x_start < 1] = 1
        y_start[y_start > 799] = 799
        y_start[y_start < 1] = 1

        stepSize = 1
        subsampleRate = self._subsampling
        numParticles = len(x_start)

        laserRange = np.zeros((numParticles, int(round(180 / subsampleRate))))

        for i in xrange(0, 180, subsampleRate):  # 180 degree sweep in laser scan with 1 degree increments
            # time1 = time.time()
            # if ( i % self._subsampling !=0 ):
            #     continue

            # Initialize current location to origin of laser
            x_current = np.copy(x_start)
            y_current = np.copy(y_start)

            # Set angle of laser, moving clockwise
            laserDir = heading + math.radians(i - 90)
            dy = np.sin(laserDir) * stepSize
            dx = np.cos(laserDir) * stepSize

            for m in xrange(numParticles):
                for d in xrange(101):
                    try:
                        # Check if the current location is in a cell that is occupied
                        if occGrid[int(round(y_current[m])), int(round(x_current[m]))] > minProbability:  # round to the resolution of the occupancy grid
                            if d == 0:  # check for particle initialized on a boundary, set range to some epsilon
                                # above 0
                                laserRange[m, i / subsampleRate] = 25
                                break
                            laserRange[m, i / subsampleRate] = d * 10  # multiply by 10 to convert from occ grid pixel size of 10 cm to cm
                            break
                        # Walk along the ray until you find an obstacle
                        x_current[m] += dx[m]
                        y_current[m] += dy[m]
                    except:
                        # Walked outside map area, set to max range
                        laserRange[m, i/subsampleRate] = 999
                        break
                # End of the range reached without detecting obstacle, set range to 1000 if true
                if d == 100:
                    laserRange[m, i/subsampleRate] = d * 10
            # time2 = time.time()
            # print time2 - time1
        return laserRange


if __name__=='__main__':

    src_path_map = '../map/wean.dat'
    sensor1 = SensorModel(src_path_map)
    sensor1.plot_prob_zt1()

    pass
