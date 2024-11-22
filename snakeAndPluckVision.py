import time
import ast
import numpy as np
from collections import deque
import threading
import queue
import math
import socket
import medusaiutils
import select
# from socket import *

#### ARM 1 IP AND UDP ####
UDP_IP = ""
UDP_PORT = 8005
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
try:
    sock.bind((UDP_IP, UDP_PORT))
    print(f"Successfully bound to {UDP_IP}:{UDP_PORT}")
except socket.error as e:
    print(f"Error binding to {UDP_IP}:{UDP_PORT} - {e}")

#### ARM 2 IP AND UDP ####
UDP_IP2 = "0.0.0.0"
UDP_PORT2 = 5006
sock2 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# sock2.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # Allow address reuse
try:
    sock2.bind((UDP_IP2, UDP_PORT2))
    print(f"Successfully bound to {UDP_IP2}:{UDP_PORT2}")
except socket.error as e:
    print(f"Error binding to {UDP_IP2}:{UDP_PORT2} - {e}")



pluck_queue = queue.Queue()
pluck_queue2 = queue.Queue()
p1 = False  # Corrected to False
p2 = False  # Corrected to False
# new booleans
canStrum1 = True
canStrum2 = True

# issue: the one that wants to pluck pops, while the other continues snaking
# is it because its trying to snake while plucking at the same time?

def pluck1(robot1):
    points = [
        [[173.9, 89.7, 608.4, -177.2, -83, -1.4], 3, 0],
        [[223.8, 49.2, 526.5, -176.8, -77.6, -1.2], 1, 0],
        [[223.8, 49.2, 526.5, -176.8, -32.7, -1.2], 2, 0],
        [[173.9, 89.7, 608.4, -177.2, -83, -1.4], 2, 0]
        #[[5, -10, 0, 115, 0, 24.3664, 0], 8, 0]
    ]
    # print("initPos !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!", initPos)
    robot1.p2pTraj(points)
    print("Pluck1 action completed.")

def pluck2(robot2):
    points = [
        [[-578.2, 280.7, 574.3, -134, -54.7, 123.7], 3, 0],
        [[-669.2, 226.1, 549.1, -127.8, -44.6, 129], 2, 0],
        [[-670.8, 208.7, 519.3, -138.7, -33.5, 144.1], 2, 0],
        [[-578.2, 280.7, 574.3, -134, -54.7, 123.7], 2, 0]
        # [[98, 31, 25, 110, 0, 22, 0], 8, 0]
    ]
    # print("initPos !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!", initPos)
    robot2.p2pTraj(points)
    print("Pluck2 action completed.")

def manual_pluck_trigger():
    global canStrum1
    global canStrum2

    while True:
        readable, _, _ = select.select([sock, sock2], [], [], 0.1)

        for s in readable:
            if s == sock:  # Data received from sock (UDP_PORT 8005)
                message, address = sock.recvfrom(1024)
                print("MESSAGE", message)

                if message == b'pluck1' and canStrum1:
                    print("can no longer strum 1")
                    canStrum1 = False
                    pluck_queue.put("pluck1")

            elif s == sock2:  # Data received from sock2 (UDP_PORT 5006)
                data, address = sock2.recvfrom(1024)
                if data != "noone":
                    print("DATA", data)

                # elif message != b'pluck1' and canStrum2:
                if ("noone" not in data.decode("utf-8")) and len(data.decode("utf-8")) > 2 and canStrum2:
                    rec_arr = ast.literal_eval(data.decode("utf-8"))[0].split("/")
                    feet = [float(num) for num in rec_arr[0].strip("[]").split()]
                    left_wrist = [float(num) for num in rec_arr[1].strip("[]").split()]
                    left_shoulder = [float(num) for num in rec_arr[2].strip("[]").split()]
                    right_shoulder = [float(num) for num in rec_arr[3].strip("[]").split()]
                    right_wrist = [float(num) for num in rec_arr[4].strip("[]").split()]
                    print("right wrist", right_wrist)

                    # strum for arm 2
                    # if right_shoulder[1] - right_wrist[1] < -25 and canStrum2:
                    # right wrist VERTICAL 163.64
                    # right wrist HORIZONTAL 1087.3
                    # right wrist VERTICAL 180.42
                    # right wrist HORIZONTAL 1117.2
                    # right wrist VERTICAL 225.27
                    # right wrist HORIZONTAL 1080.4
                    # right wrist VERTICAL 161.55
                    # right wrist HORIZONTAL 1068.3
                    # right wrist VERTICAL 183.53
                    # right wrist HORIZONTAL 1080.2

                    if 1060 <= right_wrist[0] <= 1090 and 138 <= right_wrist[1] <= 228 and canStrum2:
                        canStrum2 = False
                        print("Can no longer strum 2")
                        print("right wrist VERTICAL", right_wrist[1])
                        print("right wrist HORIZONTAL", right_wrist[0])
                        # strumQueue[0].put(arm1.get_position()[1])  # change depending on which arm to strum
                        pluck_queue2.put("pluck2")


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

"""UDP_IP = "127.0.0.1"
UDP_PORT = 5006
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))"""
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
