import math
from MapBuilder import MapBuilder

def rayTrace(x_t_laser, occGrid, minProbability):
    # x_t_laser <-- laser pose from odometry message, taken from Laser message in log (2nd odometry message)
    # occGrid is the MapBuilder class
    # minProbability is the threshold for detecting an obstacle in the occGrid
    # x_t_laser should be in the frame of the laser, not the robot origin!

    # occGrid.visualize_map()
    heading = x_t_laser[2]
    laserRange = []

    for i in xrange(180):  # 180 degree sweep in laser scan with 1 degree increments
        # Initialize current location to origin of laser
        x_current = x_t_laser[0]
        y_current = x_t_laser[1]
        # Set angle of laser, moving clockwise
        laserDir = math.radians(90 - i) + heading
        dy = math.sin(laserDir)
        dx = math.cos(laserDir)

        for d in xrange(101):
            # Check if the current location is in a cell that is occupied
            if occGrid._prob[round(x_current), round(y_current)] > minProbability:  # round to the resolution of the occupancy grid
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


## Test below this line ##
# x_t1 = [456, 713, math.pi/2]
# mapPath = '../map/wean.dat'
# minProbability = 0.95
# rayTrace(x_t1, mapPath, minProbability)
