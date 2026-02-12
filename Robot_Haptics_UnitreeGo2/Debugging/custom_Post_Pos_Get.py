# -*- coding: utf-8 -*-
"""
 When you want to know the joint positions of the robot. Run this script. It will give you 20s to manually change the joint positions, then it will print out the joint positions.
""" 

from encodings import hz
import time
import sys
import math

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.utils.thread import Thread
import unitree_legged_const as go2
crc = CRC()

class Go2Channel:
    def __init__(self, ether_name: str=""):
        self.pub = None
        self.sub = None
        self.low_state = None

        if len(ether_name)>1:
            ChannelFactoryInitialize(0, ether_name)
        else:
            ChannelFactoryInitialize(0)

        # Create a publisher to publish the data defined in UserData class
        self.pub = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.pub.Init()

        # Create a subscriber to receive the latest robot state every certain seconds 
        self.low_state = None
        self.sub = ChannelSubscriber("rt/lowstate", LowState_)
        self.sub.Init(self.LowStateMessageHandler, 10)      

    def LowStateMessageHandler(self, msg: LowState_):
        self.low_state = msg

    def ReadMotorPosition(self, joint_idx :int=0):
        if self.low_state is None or self.low_state.motor_state is None:
            return None
        q = self.low_state.motor_state[joint_idx].q
        return q
    

class Go2Leg:
    def __init__(self):
        self.channel = None
        self.cmd = unitree_go_msg_dds__LowCmd_()
        self._startPos = [0.0] * 12

        # Predefined positions
        self._standPos = [0.0, 0.67, -1.3, 0.0, 0.67, -1.3, 0.0, 0.67, -1.3, 0.0, 0.67, -1.3]
        self.laydownPos = [0.0, 1.36, -2.65, 0.0, 1.36, -2.65, -0.2, 1.36, -2.65, 0.2, 1.36, -2.65]


    @property
    def go2_channel(self):
        return self.channel

    @go2_channel.setter
    def go2_channel(self, channel: Go2Channel):
        self.channel = channel

    def init_cmd(self):
        self.cmd = unitree_go_msg_dds__LowCmd_()
        self.cmd.head[0]=0xFE
        self.cmd.head[1]=0xEF
        self.cmd.level_flag = 0xFF
        self.cmd.gpio = 0
        return self.cmd

    def send_cmd(self):
        self.cmd.crc = crc.Crc(self.cmd)
        self.channel.pub.Write(self.cmd)
    
    def init_state(self):
        self.cmd = self.init_cmd()
        for i in range(20):
            self.cmd.motor_cmd[i].mode = 0x01  # (PMSM) mode
            self.cmd.motor_cmd[i].q= go2.PosStopF
            self.cmd.motor_cmd[i].kp = 0
            self.cmd.motor_cmd[i].dq = go2.VelStopF
            self.cmd.motor_cmd[i].kd = 0
            self.cmd.motor_cmd[i].tau = 0
        self.send_cmd()

    def ready_state(self):
        while self.go2_channel.low_state is None:
            time.sleep(0.01)
        for joint_idx in range(12):
            self._startPos[joint_idx] = self.go2_channel.ReadMotorPosition(joint_idx)


    def relax(self):
        t_move = 20.0
        n = max(1, int(t_move * 200))
        for step in range(1, n + 1):
            s = step / n
            for joint_idx in range(12):
                self.cmd.motor_cmd[joint_idx].mode = 0x01
                self.cmd.motor_cmd[joint_idx].kp = 0.0 
                self.cmd.motor_cmd[joint_idx].kd = 0.0 
                self.cmd.motor_cmd[joint_idx].tau = 0.0  

        self.send_cmd()
        time.sleep(1.0 / 200)

    def get_current_Pos(self):

        kp=60.0
        kd=5.0

        self.relax()
        pos = [0.0] * 12
        for joint_idx in range(12):
            pos[joint_idx] = self.go2_channel.ReadMotorPosition(joint_idx)

        for joint_idx in range(12):
            self.cmd.motor_cmd[joint_idx].mode = 0x01
            self.cmd.motor_cmd[joint_idx].q = pos[joint_idx]
            self.cmd.motor_cmd[joint_idx].kp = kp
            self.cmd.motor_cmd[joint_idx].kd = kd
            self.cmd.motor_cmd[joint_idx].dq = 0.0
            self.cmd.motor_cmd[joint_idx].tau = 0.0
        self.send_cmd()


        # hold at the end to settle
        for _ in range(int(20 * 200)):
            for j in range(12):
                i = j
                self.cmd.motor_cmd[i].mode = 0x01
                self.cmd.motor_cmd[i].kp   = kp
                self.cmd.motor_cmd[i].kd   = kd
                self.cmd.motor_cmd[i].dq   = 0.0
                self.cmd.motor_cmd[i].tau  = 0.0
            self.send_cmd()
            time.sleep(1.0 / hz)


    def recovery(self):
            self._move_all_to(self.laydownPos, t_move=1.5, hz=200, kp=55.0, kd=4.0)
            self._hold_current_Pos(t_lock_time=1.0)
            self._move_all_to(self._standPos, t_move=1.5, hz=200, kp=55.0, kd=4.0)
            self._hold_current_Pos(t_lock_time=1.5)
            self._move_all_to(self.laydownPos, t_move=1.5, hz=200, kp=55.0, kd=4.0)

if __name__ == '__main__':
    if len(sys.argv)>1:
        go2_channel = Go2Channel(sys.argv[1])
    else:
        go2_channel = Go2Channel()

    go2_leg = Go2Leg()
    go2_leg.go2_channel = go2_channel
    go2_leg.init_state()
    go2_leg.ready_state()
    print("You have 20s to manually change the joint positions...")
    go2_leg.get_current_Pos()
    # reset to recovery pose
    go2_leg.recovery()
    

