import ast
import time
import numpy as np
from xarm.wrapper import XArmAPI
import socket
from threading import Thread
from queue import Queue
import random
import copy
from pythonosc.udp_client import SimpleUDPClient
from SASutils import robotsUtils

# global vars
time_to_move = 3
counter_threshold = 2
amt_to_move = .07

# user vars [feet_x, feet_y, lw_x, lw_y, ls_x, ls_y]
user = [None, None, None, None, None, None]

#arm 1 (middle left 237)
queueOne = Queue()
strumQueueOne = Queue()
initPosPoppingOne = [0.0, -42.0, 5.0, 71.2, -10.1, 58.4, 35.2]
# initPosPoppingOne = [0.0, -14.7, 51.7, 78.5, -4.1, -7.2, 35.2] # ARM 1 STARTING POS
# finPosPoppingOne = [0, 15.6, 7.2, 142.9, -10.1, 10.1, 35.2]
finPosPoppingOne = [0.0, -14.7, 51.7, 78.5, -10.1, -7.2, 35.2]
horBoundsOne = [-60, 80]
verBoundsOne = [60, 116] # for joint 3 (0-indexed)
currPosOne = initPosPoppingOne
currVelOne = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
currTimeOne = 0
prevStatusOne = "back"
counterOne = 0
statusOne = False
# snaking_tracker_one = 0
# rand_t_one = random.randint(rand_t_low, rand_t_high)
# mod_one = int((2 * rand_t_one)/.004)
# snaking_one = snaking(rand_t_one)
startTrackingOne = False

#arm 2 (middle right, 244)
queueTwo = Queue()
strumQueueTwo = Queue()
initPosPoppingTwo = [109.7, 24.6, 10.0, 74.2, -13.9, 57.0, 0.0] # ARM 2 STARTING POSITION
finPosPoppingTwo = [109.7, 65.1, 10.0, 126.5, 13.1, -25.8, 0.0]
horBoundsTwo = [-17.0, 50.0]
verBoundsTwo = [] #TODO: middle arm should not have vertical bounds, actually
currPosTwo = initPosPoppingTwo
currVelTwo = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
currTimeTwo = 0
prevStatusTwo = "back"
counterTwo = 0
statusTwo = False
# snaking_tracker_two = 0[0.0, -14.7, 51.7, 78.5, -4.1, -7.2, 35.2]
# rand_t_two = random.randint(rand_t_low, rand_t_high)
# mod_two = int((2 * rand_t_three) / .004)
# snaking_two = snaking(rand_t_two)
startTrackingTwo = False

# combined arm vars
queue = [queueOne, queueTwo]
strumQueue = [strumQueueOne, strumQueueTwo]
initPosPopping = [initPosPoppingOne, initPosPoppingTwo]
finPosPopping = [finPosPoppingOne, finPosPoppingTwo]
currPos = [currPosOne, currPosTwo]
currVel = [currVelOne, currVelTwo]
currTime = [currTimeOne, currTimeTwo]
prevStatus = [prevStatusOne, prevStatusTwo]
counter = [counterOne, counterTwo]
status = [statusOne, statusTwo]
# snaking_tracker = [snaking_tracker_one, snaking_tracker_two]
# snakings = [snaking_one, snaking_two]
# mods = [mod_one, mod_two]
startTracking = [startTrackingOne, startTrackingTwo]
horBounds = [horBoundsOne, horBoundsTwo]
canStrum = True

def scaleToRobot(position):
    robmin = 08.4
    robmax = -36
    vismin = -80
    vismax = 50
    scaledV = -(position - vismin)*(robmax-robmin)/(vismax-vismin) + robmin
    return scaledV

def fifth_poly(q_i, q_f, v_i, vf, t):
    if (t == 0):
        print("STOP")
        print(q_i)
        print(q_f)
        print(v_i)
        print(vf)
    # time/0.005
    traj_t = np.arange(0, t, 0.004)
    dq_i = v_i
    dq_f = vf
    ddq_i = 0
    ddq_f = 0
    a0 = q_i
    a1 = dq_i
    a2 = 0.5 * ddq_i
    a3 = 1 / (2 * t ** 3) * (20 * (q_f - q_i) - (8 * dq_f + 12 * dq_i) * t - (3 * ddq_f - ddq_i) * t ** 2)
    a4 = 1 / (2 * t ** 4) * (30 * (q_i - q_f) + (14 * dq_f + 16 * dq_i) * t + (3 * ddq_f - 2 * ddq_i) * t ** 2)
    a5 = 1 / (2 * t ** 5) * (12 * (q_f - q_i) - (6 * dq_f + 6 * dq_i) * t - (ddq_f - ddq_i) * t ** 2)
    traj_pos = a0 + a1 * traj_t + a2 * traj_t ** 2 + a3 * traj_t ** 3 + a4 * traj_t ** 4 + a5 * traj_t ** 5
    traj_vel = a1  + 2*a2 * traj_t ** 1 + 3*a3 * traj_t ** 2 + 4*a4 * traj_t ** 3 + 5*a5 * traj_t ** 4
    return traj_pos, traj_vel

def setup(strumD):
    i = 0
    for a in arms:
        a.set_simulation_robot(on_off=False)
        a.motion_enable(enable=True)
        a.clean_warn()
        a.clean_error()
        a.set_mode(0)
        a.set_state(0)
        angle = IPstring[i].copy()
        i += 1
        # angle[0] = angle[0]- strumD/2
        print(angle)
        a.set_servo_angle(angle=angle, wait=True, speed=10, acceleration=0.25, is_radian=False)

def pose_to_pose(initPos, finPos, vi, vf, t, armNum):
    print(armNum)
    global queue
    global canStrum
    global strumQueue
    with queue[armNum].mutex:
        queue[armNum].queue.clear()
        queue[armNum].all_tasks_done.notify_all()
        queue[armNum].unfinished_tasks = 0

    currArmAngles = [round(arms[armNum].angles[0], 1), round(arms[armNum].angles[1], 1), round(arms[armNum].angles[2], 1),
                     round(arms[armNum].angles[3], 1), round(arms[armNum].angles[4], 1), round(arms[armNum].angles[5], 1),
                     round(arms[armNum].angles[6], 1)]

    if t == 0:
        print("T IS ZERO")
        print(arms[armNum].angles)
        print(finPos)
        t = t/0
        # arms[armNum].set_servo_angle(angle=finPos, wait=True, speed=10, acceleration=0.25, is_radian=False)
        # currPos[armNum] = finPos
    else:
        posTrajs = []
        velTrajs = []
        for i in range(len(initPos)):  # generate trajs and vels for each joint
            tempTraj, tempVel = fifth_poly(initPos[i], finPos[i], vi[i], vf, t)  # tempTraj, tempVel has t/.004 length
            posTrajs.append(tempTraj)
            velTrajs.append(tempVel)
        reformatPos = [currArmAngles]
        reformatVel = [currVel[armNum]]
        for i in range(len(posTrajs[0])):
            posRow = []
            velRow = []
            for j in range(7):
                posRow.append(posTrajs[j][i])
                velRow.append(velTrajs[j][i])
            if finPos == finPosPopping[armNum]:
                queue[armNum].put([reformatPos[-1], posRow, currTime[armNum] + ((i+1) * 0.004), reformatVel[-1],
                          velRow])  # each queue item has curr_pos, next_pos, next_time, curr_vel, next_vel for all joints
            else:
                queue[armNum].put([reformatPos[-1], posRow, currTime[armNum] - ((i + 1) * 0.004), reformatVel[-1],
                                   velRow])
            reformatPos.append(posRow)
            reformatVel.append(velRow)

def zone_3(x, y): # middle zone
    inzone = (890 <= x <= 1050)
    return inzone

def left_arm_control(left_wrist_x, left_wrist_y, left_shoulder_x, left_shoulder_y):
    verBounds = [65, 116]

    if left_wrist_y > left_shoulder_y:
        print("LOW")
        return verBounds[0]
    else:
        print("ABOVE")
        diff = left_shoulder_y - left_wrist_y
        angle_to_move = diff * 2
        return min(116, 78.5 + angle_to_move)

def visionThread():
    global no_one_tracker
    global arms
    global counter
    global prevStatus
    global status
    global user
    global canStrum

    while True:
        # print("yo")
        data, addr = sock.recvfrom(1024)
        # print(data.decode("utf-8"))
        # print("hi")
        if ("noone" in data.decode("utf-8")):
            ### return to og position
            # print("no one here")
            for i in range(len(arms)):
                if prevStatus[i] == "back":
                    counter[i] += 1
                else:
                    counter[i] = 0
                    prevStatus[i] = "back"
                if counter[i] > 5 and status[i]:
                    status[i] = False
                    print("arm" + str(i) + ") go back, no one detected")
                    pose_to_pose(currPos[i], initPosPopping[i], currVel[i], 0, currTime[i], i)
        else:
            if len(data.decode("utf-8")) > 2:
                # print("liteval", ast.literal_eval(data.decode("utf-8")))
                # print("datadecode", len(data.decode("utf-8")))
                rec_arr = ast.literal_eval(data.decode("utf-8"))[0].split("/")
                # print("recarr", rec_arr)
                feet = [float(num) for num in rec_arr[0].strip("[]").split()]
                left_wrist = [float(num) for num in rec_arr[1].strip("[]").split()]
                left_shoulder = [float(num) for num in rec_arr[2].strip("[]").split()]
                right_shoulder = [float(num) for num in rec_arr[3].strip("[]").split()]
                # print("LEFT WRIST", left_wrist)
                # print("LEFT SHOULDER", left_shoulder)
                # print("LEFT WRIST", left_wrist)
                # print("RIGHT SHOULDER",right_shoulder)
                print("feet", feet)

                # strum for arm 1
                if left_shoulder[1] - left_wrist[1] < -25 and canStrum:
                    canStrum = False
                    print("Can no longer strum")
                    print("leftWrist", left_wrist[1])
                    strumQueue[0].put(arm1.get_position()[1]) # change depending on which arm to strum
                    # strum(arm1.get_position()[1])

                    # y-values at point of strum, 210, 258, 268.
                    # what is a way to get robot to strum based on depth? location of foot?
                    # highest location of foot recorded (when human is close to robot) = 921
                    #----------------> maybe strum when foot >= 930 and left_wrist is between certain range

                # user = [feet[0], feet[1], left_wrist[0], left_wrist[1], left_shoulder[0], left_shoulder[1]]
                relativepos = left_wrist[1]-left_shoulder[1]
                robopos = scaleToRobot(relativepos)
                print(left_wrist[1]-left_shoulder[1])
                armNum = 0
                queue[armNum].put(robopos)


                # NEW!! right shoulder queue put
                # relativepos1 = left_wrist[0] - left_shoulder[0]
                # robopos1 = scaleToRobot(relativepos1)
                # print("robopos1", robopos1)
                # armNum = 0
                # queue[armNum].put(robopos1)

                # print()
                #
                # armDetect = [False, False]
                #
                # x_coord = float(feet[0])
                # y_coord = float(feet[1])
                #
                # if zone_3(x_coord, y_coord):
                #     armDetect[0] = True
                #     armDetect[1] = True
                #     for i in range(0,2):
                #         if prevStatus[i] == "out":
                #             counter[i] += 1
                #         else:
                #             counter[i] = 0
                #             prevStatus[i] = "out"
                #         if counter[i] > counter_threshold and not(status[i]):
                #             status[i] = True
                #             print("arm " + str(i) + " out")
                #             pose_to_pose(currPos[i], finPosPopping[i], currVel[i], 0, time_to_move - currTime[i], i)
                #
                # for i in range(len(armDetect)):
                #     if not (armDetect[i]):
                #         if prevStatus[i] == "back":
                #             counter[i] += 1
                #         else:
                #             counter[i] = 0
                #             prevStatus[i] = "back"
                #         if counter[i] > counter_threshold and status[i]:
                #             status[i] = False
                #             print("arm" + str(i) + ") go back, no one detected IN RANGE")
                #             pose_to_pose(currPos[i], initPosPopping[i], currVel[i], 0, currTime[i], i)

def robotThread():
    global user
    global canStrum
    maxspeed = 20
    while True:
        start_time = time.time()
        tts = time.time() - start_time
        # for i in range(len(arms)):
        if not(queue[0].empty()):
            # print("hi")
            desiredpos = desiredpos1 = queue[0].get()

            robotpos = arms[0].angles
            goalpos = robotpos.copy()

            if abs(desiredpos-robotpos[5])>maxspeed:
                desiredpos = robotpos[5] + (maxspeed * (desiredpos-robotpos[5])/abs(desiredpos-robotpos[5]))
            # new thing commented out
            # if abs(desiredpos - robotpos[4]) > maxspeed:
                # desiredpos1 = robotpos[4] + (maxspeed * (desiredpos - robotpos[4]) / abs(desiredpos - robotpos[4]))


            # want to add robotpos[3]
            """if abs(desiredpos-robotpos[3])>maxspeed:
                desiredpos1 = robotpos[3] + (maxspeed * (desiredpos - robotpos[3]) / abs(desiredpos - robotpos[3]))"""
            goalpos[5] = desiredpos
            # goalpos[4] = desiredpos1
            # goalpos[3] = desiredpos1

            # print("api", error)
            print("goalpos", goalpos)
            # arms[0].set_mode(0)
            # arms[0].set_state(state=0)
            # arms[0].set_servo_angle(servo_id = 8, angle=goalpos, speed=5, is_radian=False, wait=True)
            # issue is set_servo_angle_j cannot control speed, and having a larger robot angle move with that instant speed is too fast
            arms[0].set_servo_angle_j(angles=goalpos, is_radian=False)


        if not strumQueue[0].empty():
            initPos = strumQueue[0].get()
            strumArm1(initPos)
            print("Can strum now!")
            canStrum = True
            #     # startTracking[i] = False
            #     info = queue[i].get()
            #     next_pos = [round(info[1][0], 1), round(info[1][1], 1), round(info[1][2], 1), round(info[1][3], 1),
            #                 round(info[1][4], 1), round(info[1][5], 1), round(info[1][6], 1)]
            #
            #     arms[i].set_servo_angle_j(angles=next_pos, is_radian=False)
            #     currPos[i] = next_pos
            #     currTime[i] = info[2]
            #     currVel[i] = info[4]
            # else:
            #     if status[0]:
            #         if startTracking[0]:
            #             next_pos = currPos[0]
            #         else:
            #             next_pos = arms[0].angles
            #             startTracking[0] = True
            #         thirdJVel = 0.0
            #         if user[3] is not None and zone_3(user[0], user[1]):
            #             joint_pos = left_arm_control(user[2], user[3], user[4], user[5])
            #             if (joint_pos > currPos[0][3] + 2):
            #                 next_pos[3] = currPos[0][3] + amt_to_move
            #                 thirdJVel = amt_to_move
            #             elif (joint_pos < currPos[0][3] - 2):
            #                 next_pos[3] = currPos[0][3] - amt_to_move
            #                 thirdJVel = -amt_to_move
            #         arms[0].set_servo_angle_j(angles = next_pos, is_radian = False)
            #         currPos[0] = next_pos
            #         currVel[i] = [0.0, 0.0, 0.0, thirdJVel, 0.0, 0.0, 0.0]


        # print(tts)
        while tts < 0.004:
            tts = time.time() - start_time
            time.sleep(0.0001)

def strumArm1(initPos):
    print("STRUMMING!!!!!!")
    print("initPos", initPos)
    # utils.p2pTraj([[173.9, 89.7, 608.4, -177.2, -83, -1.4], 3, 0])
    points =  [[[223.8, 49.2, 526.5, -176.8, -77.6, -1.2], 3, 0], [[223.8, 49.2, 526.5, -176.8, -32.7, -1.2], 2, 0],[initPos, 3, 0]]
    utils.initPos = initPos
    utils.p2pTraj(points)

if __name__ == '__main__':
    joint = 0
    global baseamp
    global uamp
    scaledbase = 2
    baseamp = 350
    uamp = 30
    global IPstring

    IP1 = [20,-56.5,5,54.6,-10.1,19.6,35.2]
    IP2 = initPosPoppingTwo
    IPstring = [IP1, IP2]

    UDP_IP = "0.0.0.0"
    UDP_PORT = 5006
    SEND_IP = "192.168.1.50"
    SEND_PORT = 5006
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    oscSend = SimpleUDPClient(SEND_IP, SEND_PORT)

    # oscSend_RIP.send_message("/noone", 0)

    arm1 = XArmAPI('192.168.1.237')  # middle left
    arm2 = XArmAPI('192.168.1.244')  # middle right
    global arms
    arms = [arm1, arm2]

    utils = robotsUtils('192.168.1.237', arm1.get_position())

    setup(2)
    input("press enter when robots stop moving")
    for a in arms:
        if a is not None:
            a.set_mode(1)
            a.set_state(0)
    time.sleep(0.5)

    visionThread = Thread(target=visionThread)
    robotThread = Thread(target=robotThread)
    visionThread.start()
    robotThread.start()

    run = True
    while run:
        if input("u") == 'q':
            # visionThread.join()
            # robotThread.join()
            # # break
            print("bye")
            run = False
