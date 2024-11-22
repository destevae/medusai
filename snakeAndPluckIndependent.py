import time
import numpy as np
from collections import deque
import threading
import queue
import math
import socket
import medusaiutils

pluck_queue = queue.Queue()
pluck_queue2 = queue.Queue()
p1 = False  # Corrected to False
p2 = False  # Corrected to False
# new booleans
canStrum1 = True
canStrum2 = True

def pluck1(robot1):
    points = [
        [[173.9, 89.7, 608.4, -177.2, -83, -1.4], 4, 0],
        [[223.8, 49.2, 526.5, -176.8, -77.6, -1.2], 1, 0],
        [[223.8, 49.2, 526.5, -176.8, -32.7, -1.2], 2, 0],
        [[173.9, 89.7, 608.4, -177.2, -83, -1.4], 2, 0]
    ]
    robot1.p2pTraj(points)
    print("Pluck1 action completed.")

def pluck2(robot2):
    points = [
        [[-578.2, 280.7, 574.3, -134, -54.7, 123.7], 3, 0],
        [[-669.6, 228.2, 547.5, -117.6, -41, 120.9], 2, 0],
        [[-673.5, 205.7, 516.8, -130.3, -31, 139.1], 2, 0],
        [[-578.2, 280.7, 574.3, -134, -54.7, 123.7], 2, 0]
    ]
    robot2.p2pTraj(points)
    print("Pluck2 action completed.")

def manual_pluck_trigger():
    global canStrum1
    global canStrum2

    while True:
        user_input = input("Enter 'pluck1' to pluck robot 1, 'pluck2' to pluck robot 2, or 'exit' to quit: ")
        if user_input == 'pluck1' and canStrum1:
            print("can no longer strum 1")
            canStrum1 = False
            pluck_queue.put("pluck1")
        elif user_input == 'pluck2' and canStrum2:
            print("can no longer strum 2")
            canStrum2 = False
            pluck_queue2.put("pluck2")
        elif user_input == 'exit':
            print("Exiting the script.")
            break
        else:
            print("Invalid input, please enter 'pluck1', 'pluck2', or 'exit'.")

def robomove():
    direction = -1
    max = 20
    speed = 0
    face = 0
    add = 0
    tomove = []
    global canStrum1
    global canStrum2

    while True:
        # for xarm in xarms:
        tomove.append(xarms[0].snakebeat(amps, speeds[speed], phases))

        for i in range(len(tomove[0])):
            # if not pluck_queue.empty():
            #     break

            start = time.time()
            num = 0
            # for xarm in xarms:
            pos = tomove[num][i].copy()
            xarms[0].movexArm(pos)
            num += 1
            t_elapse = time.time() - start
            while t_elapse < tstep:
                time.sleep(0.0001)
                t_elapse = time.time() - start

        if not pluck_queue.empty():
            action = pluck_queue.get()
            if action == "pluck1": # what's the purpose of this xarm == xarms[0]?
                # print("CURRENT POSITION !!!!!!!!!!!! ", xarms[0].getPosition())
                pluck1(xarms[0])
                print("can strum 1 now")
                canStrum1 = True

def robomove2():
    direction = -1
    max = 20
    speed = 0
    face = 0
    add = 0
    tomove = []
    global canStrum1
    global canStrum2

    while True:
        # for xarm in xarms:
        tomove.append(xarms[1].snakebeat(amps, speeds[speed], phases))

        for i in range(len(tomove[0])):
            # if not pluck_queue.empty():
            #     break

            start = time.time()
            num = 0
            # for xarm in xarms:
            pos = tomove[num][i].copy()
            xarms[1].movexArm(pos)
            num += 1
            t_elapse = time.time() - start
            while t_elapse < tstep:
                time.sleep(0.0001)
                t_elapse = time.time() - start

        if not pluck_queue2.empty():
            action = pluck_queue2.get()
            if action == "pluck2":
                # print("CURRENT POSITION !!!!!!!!!!!! ", xarms[1].getPosition())
                pluck2(xarms[1])
                print("can strum 2 now")
                canStrum2 = True

check = 15

print("DONE", check)

global nbeats
nbeats = 4
global tstep
tstep = 0.004

UDP_IP = "127.0.0.1"
UDP_PORT = 5006
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
test = 130

# robot 1, robot 2
robots = [['192.168.1.237', [5, -10, 0, 100, 0, 12, 0]], ['192.168.1.244', [98, 31, 25, 110, 0, 22, 0]]]
xarms = []

speeds = np.linspace(3, 0.5, 20)
amps = [0, 5, 0, 15, 0, 30, 0]
phases = [0, 0, 0, 0.5, 0, 0.3, 0]

for robot in robots:
    xarms.append(medusaiutils.robotsUtils(robot[0], robot[1], sim=False))

for xarm in xarms:
    IP = xarm.snakebeat(amps, speeds[0], phases)
    print(IP[0])
    xarm.setupBot(IP[0])

input("Press enter when robots stop moving to start the script.")

threading.Thread(target=robomove, daemon=True).start()

threading.Thread(target=robomove2, daemon=True).start()

# threading.Thread(target=manual_pluck_trigger, daemon=True).start()
#
# while True:
manual_pluck_trigger()
