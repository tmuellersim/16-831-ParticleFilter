import numpy as np
import pdb

class Resampling:

    """
    References: Thrun, Sebastian, Wolfram Burgard, and Dieter Fox. Probabilistic robotics. MIT press, 2005. [pp 108-113]
    """

    def __init__(self):
        self._log_offset = 30
        pass

    def multinomial_sampler(self, X_bar):
        xt1_list = [item[0] for item in X_bar]
        wts_list = [item[1]+self._log_offset for item in X_bar]

        print wts_list
        # pdb.set_trace()

        wts_list = wts_list/np.sum(wts_list)

        xt1_freqs = np.random.multinomial(len(wts_list), wts_list)
        X_bar_resampled = []

        for m in range(0, len(wts_list)):
            for n in range(0, xt1_freqs[m]):
                list = [ xt1_list[m], wts_list[m] ]
                X_bar_resampled.append(list)

        return X_bar_resampled

    def low_variance_sampler(self, X_bar):
        xt1_list = [item[0] for item in X_bar]
        wts_list = [item[1]+self._log_offset for item in X_bar]

        wts_list = wts_list/np.sum(wts_list)

        # print wts_list
        # pdb.set_trace()

        X_bar_resampled = []
        M = len(wts_list)
        r = np.random.uniform(0, 1.0/M)
        i = 0; c = wts_list[i]

        for m in range(0,M):
            U = r + float(m)/M
            while (U > c):
                i = i + 1
                c = c + wts_list[i]

            list = [ xt1_list[i], wts_list[i] ]
            X_bar_resampled.append(list)

        return X_bar_resampled

if __name__ == "__main__":
    pass