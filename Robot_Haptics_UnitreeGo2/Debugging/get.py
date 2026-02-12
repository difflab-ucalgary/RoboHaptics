import time
import sys

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
        q = self.low_state.motor_state[joint_idx].q
        return q
    

class Go2Leg:
    def __init__(self):
        self.channel = None
        self.cmd = unitree_go_msg_dds__LowCmd_()
        self._startPos = [0.0] * 12

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
        for joint_idx in range(12):
            self._startPos[joint_idx] = self.go2_channel.ReadMotorPosition(joint_idx)
    
    def joint_relax(self):    
        for joint_idx in range(12):
            self.cmd.motor_cmd[joint_idx].mode = 0x01
            self.cmd.motor_cmd[joint_idx].kp = 0.0 
            self.cmd.motor_cmd[joint_idx].kd = 0.0 
            self.cmd.motor_cmd[joint_idx].tau = 0.0  
            self.send_cmd()

    def getJointPos(self):
        # print(f"[INFO] Joint 'FR_0' pos: {go2_leg.channel.low_state.motor_state[0].q}")        
        # print(f"[INFO] Joint 'FR_1' pos: {go2_leg.channel.low_state.motor_state[1].q}")        
        # print(f"[INFO] Joint 'FR_2' pos: {go2_leg.channel.low_state.motor_state[2].q}")  

        # print(f"[INFO] Joint 'FL_0' pos: {go2_leg.channel.low_state.motor_state[3].q}")        
        # print(f"[INFO] Joint 'FL_1' pos: {go2_leg.channel.low_state.motor_state[4].q}")        
        # print(f"[INFO] Joint 'FL_2' pos: {go2_leg.channel.low_state.motor_state[5].q}") 

        # print(f"[INFO] Joint 'RR_0' pos: {go2_leg.channel.low_state.motor_state[6].q}")        
        # print(f"[INFO] Joint 'RR_1' pos: {go2_leg.channel.low_state.motor_state[7].q}")        
        # print(f"[INFO] Joint 'RR_2' pos: {go2_leg.channel.low_state.motor_state[8].q}") 

        # print(f"[INFO] Joint 'RL_0' pos: {go2_leg.channel.low_state.motor_state[9].q}")        
        # print(f"[INFO] Joint 'RL_1' pos: {go2_leg.channel.low_state.motor_state[10].q}")        
        # print(f"[INFO] Joint 'RL_2' pos: {go2_leg.channel.low_state.motor_state[11].q}\n\n") 

        print(f"FRpos: {go2_leg.channel.low_state.motor_state[0].q, go2_leg.channel.low_state.motor_state[1].q, go2_leg.channel.low_state.motor_state[2].q,}")        
        print(f"FLpos: {go2_leg.channel.low_state.motor_state[3].q, go2_leg.channel.low_state.motor_state[4].q, go2_leg.channel.low_state.motor_state[5].q,}")
        print(f"RRpos: {go2_leg.channel.low_state.motor_state[6].q, go2_leg.channel.low_state.motor_state[7].q, go2_leg.channel.low_state.motor_state[8].q,}")
        print(f"RLpos: {go2_leg.channel.low_state.motor_state[9].q, go2_leg.channel.low_state.motor_state[10].q, go2_leg.channel.low_state.motor_state[11].q,}")
        time.sleep(0.01)                   
        return 0
    
if __name__ == '__main__':
    if len(sys.argv)>1:
        go2_channel = Go2Channel(sys.argv[1])
    else:
        go2_channel = Go2Channel()

    go2_leg = Go2Leg()
    go2_leg.go2_channel = go2_channel
    go2_leg.init_state()
    time.sleep(1.0)

    go2_leg.ready_state()
    # Enable this to relax all joints, can change to joint pos manually.
    go2_leg.joint_relax()
    # read joint positions
    while True:
        go2_leg.getJointPos()   

