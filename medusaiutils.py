import numpy as np
import math
import time
from pythonosc import udp_client

global stepT
stepT = 0.004

class robotsUtils:
    def __init__(self, IPadress, initPos, sim=False):
        self.arm = IPadress
        self.initPos = initPos
        # global startP
        # startP = initPos
        self.tstep = 0.004
        # if sim == False:
        from xarm.wrapper import XArmAPI
        self.xArm = XArmAPI(self.arm)
        self.sim = sim
        # self.IPtoSEND = "127.0.0.1" # "192.168.1.50"
        per = 4
        self.ports = tuple(10001 + i for i in range(per)) + tuple(11001 + i for i in range(per)) + tuple(
            12001 + i for i in range(per)) + tuple(13001 + i for i in range(per))
        # this is indexed at 1
        self.routes = (('melody',) * 8) + (('pitch',) * 8)
        self.client = ...

        self.IPtoSEND = "127.0.0.1"
        #self.ports = (5010, 5011, 5012, 5013)
        #self.routes = ('melody', 'pitch', 'rhythm', 'rhythm')
        #self.client = 2

    def setupBot(self,IP = 0):
        if not self.sim:
            self.xArm.set_simulation_robot(on_off=False)
            self.xArm.motion_enable(enable=True)
            self.xArm.clean_warn()
            self.xArm.clean_error()
            self.xArm.set_mode(0)
            self.xArm.set_state(0)
            self.xArm.set_servo_angle(angle=IP, wait=True, speed=20, acceleration=0.5, is_radian=False)
            time.sleep(0.1)
            self.realTimeMode()

    def realTimeMode(self):
        self.xArm.set_mode(1)
        self.xArm.set_state(0)

    def getPosition(self):
        return self.xArm.get_position()[1]

    # def onetimesnake(self, amp, t, phase):

    def snakebeat(self, amp, duration, phase):
        t = np.arange(0, 2 * duration + self.tstep, self.tstep)
        # print(t[-1])
        traj = []
        for i in range(7):
            traj.append([round(
                -amp[i] * math.cos((math.pi * (q - phase[i] * duration)) / duration) + amp[i] + self.initPos[i], 4) for
                         q in t])
        traj = np.array(traj)
        traj = np.transpose(traj)
        return traj

    def snakebeathalf(self, amp, duration, phase):
        t = np.arange(0,  duration + self.tstep, self.tstep)
        # print(t[-1])
        traj = []
        for i in range(7):
            traj.append([round(
                -amp[i] * math.cos((math.pi * (q - phase[i] * duration)) / duration) + amp[i] + self.initPos[i], 4) for
                         q in t])
        traj = np.array(traj)
        traj = np.transpose(traj)
        return traj


    def movexArm(self, traj):
        if self.sim == False:
            # togo = []
            # for i in range(7):
            #     togo.append(+traj[i])
            self.xArm.set_servo_angle_j(angles=traj, is_radian=False)
        else:
            print(traj)

    def fifth_poly(self, q_i, q_f, v_i, vf, t):
        # time/0.005
        traj_t = np.arange(0, t, stepT)
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
        for i in range(len(traj_pos)):
            traj_pos[i] = round(traj_pos[i], 5)
        return traj_pos

    def Singlep2ptraj(self, pi, pf, t):
        trajectories = [None] * 6
        for i in range(6):
            trajectories[i] = self.fifth_poly(pi[i], pf[i], 0, 0, t)

        trajectories = np.array(trajectories).T
        return trajectories

    def p2pTraj(self, points):
        print('here!!!!!!!!!!!!!!!!!!!!!!!')
        pointarray = []
        timearray = []
        soundarray = []
        for p in points:
            pointarray.append(p[0])
            timearray.append(p[1])
            soundarray.append(p[2])
        if self.sim == False:
            IP = self.xArm.position
        else:

            print("CHECK THAT YOUR FINAL POSITION MATCHES THIS ONE SIMULATION MODE WILL PROBABLY BREAK IF GOING FROM FORWARD KIN TO INVERSE KIN")
            IP = self.initPos
        traj = self.Singlep2ptraj(IP, pointarray[0], timearray[0])
        sound = list(np.linspace(0., 0., math.ceil(timearray[0] / stepT)))
        self.movexArm2(traj, soundarray[0])
        for i in (range(len(points) - 1)):
            # toAdd =

            traj = self.Singlep2ptraj(pointarray[i], pointarray[i + 1], timearray[i + 1])
            self.movexArm2(traj, soundarray[i + 1])
            # if soundarray[i+1] > 0:
            #     sound = sound+ list(np.linspace(0., 1., math.ceil(timearray[i+1]/stepT)))
            # else:
            #     sound = sound + list(np.linspace(0., 0., math.ceil(timearray[i+1]/stepT)))

        ### This p2p automatically goes back to the original position it was called from ###
        traj = self.Singlep2ptraj(pointarray[-1],IP, timearray[0])
        self.movexArm2(traj, soundarray[0])
        # return traj,sound


    def movexArm2(self, traj, sound):
        self.client = udp_client.SimpleUDPClient(self.IPtoSEND, (5010, 5011, 5012, 5013)[sound - 1])
        soundarr = np.linspace(0., sound, len(traj))
        count = 0
        if sound > 0:
            self.client.send_message("/on", 1)
        for i in traj:
            start_time = time.time()
            tts = time.time() - start_time
            if self.sim == False:
                self.xArm.set_servo_cartesian(i, speed=100, mvacc=2000)
                if sound > 0:
                    send = soundarr[count]
            else:
                print(i)
                if sound > 0:
                    print(soundarr[count])
                    self.client.send_message(f"/{('melody', 'pitch', 'rhythm', 'rhythm')[sound - 1]}", soundarr[count])

            count += 1

            while tts < 0.004:
                tts = time.time() - start_time
                time.sleep(0.0001)
        if sound > 0:
            self.client.send_message("/on", 0)

