import sys
import numpy as np

from matplotlib import pyplot as plt
from matplotlib import figure as fig
import time

class MapBuilder:

    def __init__(self, src_path_map):

        self._prob = np.genfromtxt(src_path_map, skip_header=7)
        self._prob[self._prob < 0] = -1
        self._prob[self._prob > 0] = 1 - self._prob[self._prob > 0]

        self._resolution = 10
        self._size_x = self._prob.shape[0]
        self._size_y = self._prob.shape[1]

    def visualize_map(self):
        fig = plt.figure()
        mng = plt.get_current_fig_manager(); mng.resize(*mng.window.maxsize())
        plt.ion(); plt.imshow(self._prob, cmap='Greys'); plt.axis([0, self._size_x, 0, self._size_y]); plt.draw()
        time.sleep(100)

    def getMap(self):
        return self._prob

if __name__=="__main__":
    
    # src_path_map = '../map/wean.dat'
    # map1 = MapBuilder(src_path_map)
    # map1.visualize_map()

    pass
