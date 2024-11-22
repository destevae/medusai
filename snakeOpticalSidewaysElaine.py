import time
import numpy as np
from collections import deque
# from inputimeout import inputimeout, TimeoutOccurred
import threading
import queue
import math
import socket
import medusaiutils

canStrum1 = True
canStrum2 = True
pluck_queue = queue.Queue()
pluck_queue2 = queue.Queue()
# def snakebeat(duration,phase):
#     t = np.arange(0,duration+tstep,tstep)
#     print(t[-1])

#     traj = [round(-0.5*math.cos((math.pi*(q-phase*duration))/duration)+0.5,4) for q in t]
#     return traj


def robomove():
    direction = -1
    max = 20

    speed = q.get()
    face = 0
    add = 0
    global canStrum1
    global canStrum2

    while True:

        ### going up ###
        if q.qsize() > 0:
            input= q.queue[-1]
            speed = input[1]
            if speed >10:
                speed = 10
            # ampscale =
            face = input[0]/20
            if face >20:
                face = 20
            #
            # amps[1] = 1*speed
            amps[3] = 2*speed

            amps[0] = face * 5
            amps[2] = face * 15
            amps[4] = face * 30

            # speed = q.queue[-1]

            print("q speed",speed)
            q.queue.clear()
            if speed > len(speeds)-1:
                speed = len(speeds)-1  # item = q.get()
        tomove = []
        # for xarm in xarms:
        tomove.append(xarms[0].snakebeathalf(amps, speeds[0], phases))
        # end = float(traj[-1])
        # IP = 1
        for i in range(len(tomove[0])):
            start = time.time()
            num = 0
            # for xarm in xarms:
            pos = tomove[num][i].copy()
                # if abs(add) < 20:


                # pos[2] += (face-1) * 0.2

            xarms[0].movexArm(pos)
            num += 1
            t_elapse = time.time() - start
            while t_elapse < tstep:
                time.sleep(0.0001)
                t_elapse = time.time() - start

        if not pluck_queue.empty():
            action = pluck_queue.get()
            if action == "pluck1":
                # print("CURRENT POSITION !!!!!!!!!!!! ", xarms[1].getPosition())
                pluck(1, xarms[0])
                print("can strum 1 now")
                canStrum1 = True

        ### going down ####
        #[x+1 for x in mylist]
        if q.qsize() > 0:
            input = q.queue[-1]
            speed = input[1]
            if speed > 15:
                speed = 15
            # ampscale =
            face = input[0] / 20
            if face > 20:
                face = 20

            amps[3] = 2 * speed

            amps[0] = face * 5
            amps[2] = face * 15
            amps[4] = face * 30
            # speed = q.queue[-1]
            print("q speed", speed)
            q.queue.clear()
            if speed > len(speeds) - 1:
                speed = len(speeds) - 1  # item = q.get()
        tomove = []
        # for xarm in xarms:
        tomove.append(xarms[0].snakebeathalf(amps, speeds[0],[x+1 for x in phases]))
        # end = float(traj[-1])
        # IP = 1
        for i in range(len(tomove[0])):
            start = time.time()
            num = 0
            # for xarm in xarms:
            xarms[0].movexArm(tomove[num][i])
            num += 1
            t_elapse = time.time() - start
            while t_elapse < tstep:
                time.sleep(0.0001)
                t_elapse = time.time() - start

        if not pluck_queue.empty():
            action = pluck_queue.get()
            if action == "pluck1":
                # print("CURRENT POSITION !!!!!!!!!!!! ", xarms[1].getPosition())
                pluck(1, xarms[0])
                print("can strum 1 now")
                canStrum1 = True

def robomove2():
    direction = -1
    max = 20

    speed = q2.get()
    face = 0
    add = 0
    global canStrum1
    global canStrum2

    while True:

        ### going up ###
        if q2.qsize() > 0:
            input= q2.queue[-1]
            speed = input[1]
            if speed >10:
                speed = 10
            # ampscale =
            face = input[0]/20
            if face >20:
                face = 20
            #
            # amps[1] = 1*speed
            amps[3] = 2*speed

            amps[0] = face * 5
            amps[2] = face * 15
            amps[4] = face * 30

            # speed = q.queue[-1]

            print("q speed",speed)
            q2.queue.clear()
            if speed > len(speeds)-1:
                speed = len(speeds)-1  # item = q.get()
        tomove = []
        # for xarm in xarms:
        tomove.append(xarms[1].snakebeathalf(amps, speeds[0], phases))
        # end = float(traj[-1])
        # IP = 1
        for i in range(len(tomove[0])):
            start = time.time()
            num = 0
            # for xarm in xarms:
            pos = tomove[num][i].copy()
                # if abs(add) < 20:


                # pos[2] += (face-1) * 0.2

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
                pluck(2, xarms[1])
                print("can strum 2 now")
                canStrum2 = True

        ### going down ####
        #[x+1 for x in mylist]
        if q2.qsize() > 0:
            input = q2.queue[-1]
            speed = input[1]
            if speed > 15:
                speed = 15
            # ampscale =
            face = input[0] / 20
            if face > 20:
                face = 20

            amps[3] = 2 * speed

            amps[0] = face * 5
            amps[2] = face * 15
            amps[4] = face * 30
            # speed = q.queue[-1]
            print("q speed", speed)
            q2.queue.clear()
            if speed > len(speeds) - 1:
                speed = len(speeds) - 1  # item = q.get()
        tomove = []
        # for xarm in xarms:
        tomove.append(xarms[1].snakebeathalf(amps, speeds[0],[x+1 for x in phases]))
        # end = float(traj[-1])
        # IP = 1
        for i in range(len(tomove[0])):
            start = time.time()
            num = 0
            # for xarm in xarms:
            xarms[1].movexArm(tomove[num][i])
            num += 1
            t_elapse = time.time() - start
            while t_elapse < tstep:
                time.sleep(0.0001)
                t_elapse = time.time() - start

        if not pluck_queue2.empty():
            action = pluck_queue2.get()
            if action == "pluck2":
                # print("CURRENT POSITION !!!!!!!!!!!! ", xarms[1].getPosition())
                pluck(2, xarms[1])
                print("can strum 2 now")
                canStrum2 = True

def pluck(robotNum, usedRobot):
    if robotNum == 1:
        points = [
            [[173.9, 89.7, 608.4, -177.2, -83, -1.4], 4, 0],
            [[223.8, 49.2, 526.5, -176.8, -77.6, -1.2], 1, 0],
            [[223.8, 49.2, 526.5, -176.8, -32.7, -1.2], 2, 0],
            [[173.9, 89.7, 608.4, -177.2, -83, -1.4], 2, 0]
        ]
    else:
        points = [
            [[-578.2, 280.7, 574.3, -134, -54.7, 123.7], 3, 0],
            [[-669.6, 228.2, 547.5, -117.6, -41, 120.9], 2, 0],
            [[-673.5, 205.7, 516.8, -130.3, -31, 139.1], 2, 0],
            [[-578.2, 280.7, 574.3, -134, -54.7, 123.7], 2, 0]
        ]
    # print("initPos !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!", initPos)
    usedRobot.p2pTraj(points)
    print("Pluck", robotNum, " action completed.")

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


check = 15

print("DONE", check)

global nbeats
nbeats = 4
global tstep
tstep = 0.004
q = queue.Queue()
q2 = queue.Queue()

UDP_IP = "127.0.0.1"
UDP_PORT = 5005
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
test = 130
# while test not in range(60,120):
#     print(test)
#     print("yellow")
#     if test <60:
#         test = test*2
#     else:
#         test = test/2


robots = [['192.168.1.237', [5,-10,0,100,0,12,0]], ['192.168.1.244', [98,31,25,110,0,22,0]] ]
xarms = []

speeds = np.linspace(3, 0.5, 20)
ampscale = np.linspace(1, 5, 20)
amps = [0, 5, 0, 3, 0, -30, 0]#[0, 5, 0, 15, 0, -30, 0]
phases = [0, 0, 0.0, 0, 0, 0.3, 0]
### ONLY SIDEWAYS BEHAVIORS ####
# amps = [5, 0, 15, 0, 30, 0, 0]
# phases = [0, 0, 0.5, 0, 0.3, 0, 0]
#be
for robot in robots:
    xarms.append(medusaiutils.robotsUtils(robot[0], robot[1], sim=False))

for xarm in xarms:
    IP = xarm.snakebeathalf(amps,speeds[0],phases)
    print(IP[0])
    xarm.setupBot(IP[0])
threading.Thread(target=robomove, daemon=True).start()
threading.Thread(target=robomove2, daemon=True).start()
input("press enter when robots stop moving to start script")


# in order to hard code values, we can alternate between sleeps and the while True loop
# confirmed this sleep works - but maybe we don't need it, since we'll switch to the True loop immediately after plucking
# work on speeding up the plucking functions and timing them


threading.Thread(target=manual_pluck_trigger, daemon=True).start()

q.put(0)
q2.put(0)
while True:
    # print("wating")
    data, addr = sock.recvfrom(1024)
    array = np.frombuffer(data,dtype = int)
    # print("Received array:", array)
    q.put(array)
    q2.put(array)
    # q.put(int(data.decode('utf-8')))


    # data_list.append(int(data.decode('utf-8')))
    # print(f"Received data: {int(data.decode('utf-8'))}")

    # print(bpm)
    # print(timeb)

    #     except TimeoutOccurred:
    #         exit
    #         engage = False
    #     # input()
    # print("timeout")


