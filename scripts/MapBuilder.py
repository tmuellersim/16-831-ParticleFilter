import sys
import numpy as np

class MapBuilder:

    def __init__(self, src_path_map):

        self._prob = np.genfromtxt(src_path_map, skip_header=7)
        self._prob[self._prob < 0] = -1
        self._prob[self._prob > 0] = 1 - self._prob[self._prob > 0]

        self._resolution = 10
        self._size_x = self._prob.shape[0]
        self._size_y = self._prob.shape[1]

def main(args):
    src_path_map = '../map/wean.dat'
    map1 = MapBuilder(src_path_map)

if __name__=="__main__":
    main(sys.argv)