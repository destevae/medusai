import numpy as np
import math
import time
#from xarm.wrapper import XArmAPI
import SASutils

if __name__ == "__main__":

    """xarm = SASutils.robotsUtils("192.168.1.242", [182.2, 33, 678.6, 3.1, -67.3, -179.3], False)
    xarm.setupBot(True)
    points = [ [[182.2, 33, 678.6, 3.1, -50, -179.3], 5, 0]]# [ [[10,10,10,0,0,0],3,0], [[20,20,20,5,3,2],3,0],[[30,30,30,5,3,2],3,1]]
    xarm.p2pTraj(points)"""

    # edit initial position
    initPos = [173.9, 89.9, 608.4, -177.7, -83.5, -1.2]
    xarm = SASutils.robotsUtils("192.168.1.242", initPos, False)
    xarm.setupBot(True)

    # points for strumming
    points = [ [[213.8, 93, 526.5, -176.8, -66.9, -1.2], 7, 0], [[213.8, 93, 526.5, -176.8, -32.7, -1.2], 4, 0]]
    xarm.p2pTraj(points)
