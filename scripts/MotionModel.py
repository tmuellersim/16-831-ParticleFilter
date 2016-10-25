import sys
import numpy as np
import math

class MotionModel:

    """
    References: Thrun, Sebastian, Wolfram Burgard, and Dieter Fox. Probabilistic robotics. MIT press, 2005. [pp 132-139]
    """

    def __init__(self, alpha1, alpha2, alpha3, alpha4):

        self._alpha1 = alpha1
        self._alpha2 = alpha2
        self._alpha3 = alpha3
        self._alpha4 = alpha4

    def sample_motion_model(self, u_t0, u_t1, x_t0):
        """
        :param[in] u_t0 : particle state odometry reading [x, y, theta] at time (t-1) [odometry_frame]   
        :param[in] u_t1 : particle state odometry reading [x, y, theta] at time t [odometry_frame]
        :param[in] x_t0 : particle state belief [x, y, theta] at time (t-1) [world_frame]
        :param[out] x_t1 : particle state belief [x, y, theta] at time t [world_frame]
        """
        delta_rot1 = math.atan2(u_t1[1]-u_t0[1], u_t1[0]-u_t0[0]) - u_t0[2]
        delta_trans = math.sqrt( (u_t1[0]-u_t0[0])**2 + (u_t1[1]-u_t0[1])**2 )
        delta_rot2 = u_t1[2] - u_t0[2] - delta_rot1

        # print 'delta_rot1: %.6e' % delta_rot1**2
        # print 'delta_trans: %.6e' % delta_trans**2
        # print 'delta_rot2: %.6e' % delta_rot2**2

        std_rot1 = math.sqrt(self._alpha1*delta_rot1**2 + self._alpha2*delta_trans**2)
        std_trans = math.sqrt(self._alpha3*delta_trans**2 + self._alpha4*delta_rot1**2 + self._alpha4*delta_rot2**2)
        std_rot2 = math.sqrt(self._alpha1*delta_rot2**2 + self._alpha2*delta_trans**2)

        noise_rot1 = np.random.normal(0, std_rot1) if std_rot1 > 0 else 0
        noise_trans = np.random.normal(0, std_trans) if std_trans > 0 else 0
        noise_rot2 = np.random.normal(0, std_rot2) if std_rot2 > 0 else 0

        delta_rot1_hat = delta_rot1 - noise_rot1
        delta_trans_hat = delta_trans - noise_trans
        delta_rot2_hat = delta_rot2 - noise_rot2

        x_t1 = [x_t0[0] + delta_trans_hat*math.cos(x_t0[2]+delta_rot1_hat),
                x_t0[1] + delta_trans_hat*math.sin(x_t0[2]+delta_rot1_hat),
                x_t0[2] + delta_rot1_hat + delta_rot2_hat]

        return x_t1

    def sample_motion_model_array(self, u_t0, u_t1, x_t0_array):
        """
        :param[in] u_t0 : particle state odometry reading [x, y, theta] at time (t-1) [odometry_frame]
        :param[in] u_t1 : particle state odometry reading [x, y, theta] at time t [odometry_frame]
        :param[in] x_t0 : particle state belief [x, y, theta] at time (t-1) [world_frame]
        :param[out] x_t1 : particle state belief [x, y, theta] at time t [world_frame]
        """
        delta_rot1 = math.atan2(u_t1[1] - u_t0[1], u_t1[0] - u_t0[0]) - u_t0[2]
        delta_trans = math.sqrt((u_t1[0] - u_t0[0]) ** 2 + (u_t1[1] - u_t0[1]) ** 2)
        delta_rot2 = u_t1[2] - u_t0[2] - delta_rot1

        # print 'delta_rot1: %.6e' % delta_rot1**2
        # print 'delta_trans: %.6e' % delta_trans**2
        # print 'delta_rot2: %.6e' % delta_rot2**2

        std_rot1 = math.sqrt(self._alpha1 * delta_rot1 ** 2 + self._alpha2 * delta_trans ** 2)
        std_trans = math.sqrt(
            self._alpha3 * delta_trans ** 2 + self._alpha4 * delta_rot1 ** 2 + self._alpha4 * delta_rot2 ** 2)
        std_rot2 = math.sqrt(self._alpha1 * delta_rot2 ** 2 + self._alpha2 * delta_trans ** 2)

        noise_rot1 = np.random.normal(0, std_rot1) if std_rot1 > 0 else 0
        noise_trans = np.random.normal(0, std_trans) if std_trans > 0 else 0
        noise_rot2 = np.random.normal(0, std_rot2) if std_rot2 > 0 else 0

        delta_rot1_hat = delta_rot1 - noise_rot1
        delta_trans_hat = delta_trans - noise_trans
        delta_rot2_hat = delta_rot2 - noise_rot2

        x_t1_array = [x_t0_array[:, 0] + delta_trans_hat * np.cos(x_t0_array[:, 2] + delta_rot1_hat),
                      x_t0_array[:, 1] + delta_trans_hat * np.sin(x_t0_array[:, 2] + delta_rot1_hat),
                      x_t0_array[:, 2] + delta_rot1_hat + delta_rot2_hat]

        return x_t1_array

if __name__=="__main__":

    pass