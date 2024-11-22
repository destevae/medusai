import numpy as np
import math
import time
#from xarm.wrapper import XArmAPI
import SASutils
from socket import *
if __name__ == "__main__":
    UDP_IP = ""
    UDP_PORT = 8005
    sock = socket(AF_INET, SOCK_DGRAM)  # UDP
    sock.bind((UDP_IP, UDP_PORT))

    # edit initial position
    initPos = [173.9, 89.7, 608.4, -177.2, -83, -1.4]
    xarm = SASutils.robotsUtils("192.168.1.237", initPos, False)
    xarm.setupBot(True)

    initPos2 = [-578.2, 280.7, 574.3, -134, -54.7, 123.7]
    xarm2 = SASutils.robotsUtils("192.168.1.244", initPos2, False)
    xarm2.setupBot(True)

    # points for strumming
    input("press enter when robot stops moving")
    while True:
        message, address = sock.recvfrom(1024)
        print(message)
        print(message == b'pluck')
        if message == b'pluck1':
            points = [ [[223.8, 49.2, 526.5, -176.8, -77.6, -1.2], 3, 0], [[223.8, 49.2, 526.5, -176.8, -32.7, -1.2], 2, 0], [[173.9, 89.7, 608.4, -177.2, -83, -1.4], 3, 0]]
            xarm.p2pTraj(points)
            # break
        if message == b'pluck2':
            points = [[[-669.2, 226.1, 549.1, -127.8, -44.6, 129], 4, 0], [[-670.8, 208.7, 519.3, -138.7, -33.5, 144.1], 2, 0], [[-578.2, 280.7, 574.3, -134, -54.7, 123.7], 3, 0]]
            xarm2.p2pTraj(points)
            # break

