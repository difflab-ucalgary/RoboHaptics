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

        self._standPos = [0.0, 0.67, -1.3, 0.0, 0.67, -1.3, 0.0, 0.67, -1.3, 0.0, 0.67, -1.3]
        self.laydownPos = [0.0, 1.36, -2.65, 0.0, 1.36, -2.65, -0.2, 1.36, -2.65, -0.2, 1.36, -2.65]
        self.kf1_weight_shift = [
                0.067, -1.16, -2.65,     # FR 
                -0.226, +0.659, -1.085,   # FL (stance)
                -0.255, +0.953, -1.648,   # RR (stance)
                -0.251, +1.073, -1.956    # RL (stance)
                ]
        self.fr_contact = [
                -0.226, -1.16, -0.9,     # FR
                -0.226, +0.659, -1.085,   # FL (stance)
                -0.255, +0.953, -1.648,   # RR (stance)
                -0.251, +1.073, -1.956    # RL (stance)
                ]
        self._middlePos = [-0.226, -1.16, -1.5,     # FR
                -0.226, +0.659, -1.085,   # FL (stance)
                -0.255, +0.953, -1.648,   # RR (stance)
                -0.251, +1.073, -1.956    # RL (stance)
                ]
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

    def _hold_current_Pos(self, q_target, t_lock_time = 5.0, hz=200, kp=80.0, kd=5.0):
        """
        Lock all joints at their current positions.
        """
        n = max(1, int(t_lock_time * hz))
        for step in range(1, n + 1):
            for joint_idx in range(12):
                self.cmd.motor_cmd[joint_idx].mode = 0x01
                self.cmd.motor_cmd[joint_idx].q = q_target[joint_idx]
                self.cmd.motor_cmd[joint_idx].kp = kp
                self.cmd.motor_cmd[joint_idx].kd = kd
                self.cmd.motor_cmd[joint_idx].dq = 0.0
                self.cmd.motor_cmd[joint_idx].tau = 0.0
            self.send_cmd()
            time.sleep(1.0 / hz)


    def _move_all_to(self, q_target, t_move=1.2, hz=200, kp=55.0, kd=4.0, lock_kp=60.0, lock_kd=5.0):
        """
        Smoothly move all joints to q_target.
        While moving, keep all other joints locked at their captured angles.
        """
        # start from current positions
        q_now = [self.go2_channel.ReadMotorPosition(i) for i in range(12)]

        n = max(1, int(t_move * hz))
        for step in range(1, n + 1):
            alpha = step / float(n)
            q_cmd = [q_now[k]*(1.0 - alpha) + q_target[k]*alpha for k in range(12)]

            # command all joints
            for j in range(3):
                i = j  # all indices are 0-11
                self.cmd.motor_cmd[i].mode = 0x01
                self.cmd.motor_cmd[i].q    = q_cmd[j]
                self.cmd.motor_cmd[i].kp   = kp
                self.cmd.motor_cmd[i].kd   = kd
                self.cmd.motor_cmd[i].dq   = 0.0
                self.cmd.motor_cmd[i].tau  = 0.0
            for j in range(3,12):
                i = j
                self.cmd.motor_cmd[i].mode = 0x01
                self.cmd.motor_cmd[i].q    = q_target[j]
                self.cmd.motor_cmd[i].kp   = 70
                self.cmd.motor_cmd[i].kd   = 70
                self.cmd.motor_cmd[i].dq   = 0.0
                self.cmd.motor_cmd[i].tau  = 0.0
            # send once per cycle
            self.send_cmd()
            time.sleep(1.0 / hz)

        # small hold at the end to settle
        for _ in range(int(0.3 * hz)):
            for j in range(12):
                i = j
                self.cmd.motor_cmd[i].mode = 0x01
                self.cmd.motor_cmd[i].q    = q_target[j]
                self.cmd.motor_cmd[i].kp   = kp
                self.cmd.motor_cmd[i].kd   = kd
                self.cmd.motor_cmd[i].dq   = 0.0
                self.cmd.motor_cmd[i].tau  = 0.0
            self.send_cmd()
            time.sleep(1.0 / hz)

    def fr_tap(self):
        # Move to stand position
        self._move_all_to(self._standPos, t_move=1.5, hz=200, kp=55.0, kd=4.0)
        self._hold_current_Pos(q_target=self._standPos, t_lock_time=1.0)

        # Preform weight shift to prepare for tap
        self._move_all_to(self.kf1_weight_shift, t_move=0.2, hz=200, kp=60.0, kd=4.0)
        self._hold_current_Pos(q_target=self.kf1_weight_shift, t_lock_time=1.0)

        self._move_all_to(self.fr_contact, t_move=1.0, hz=200, kp=30.0, kd=1.0)
        self._hold_current_Pos(q_target=self.fr_contact, t_lock_time=2.5)

        # Move to stand position
        self._move_all_to(self._standPos, t_move=0.5, hz=200, kp=70.0, kd=4.0)
        self._hold_current_Pos(q_target=self._standPos, t_lock_time=1.0)
        self._move_all_to(self.laydownPos, t_move=1.5, hz=200, kp=70.0, kd=4.0)

if __name__ == '__main__':
    if len(sys.argv)>1:
        go2_channel = Go2Channel(sys.argv[1])
    else:
        go2_channel = Go2Channel()

    go2_leg = Go2Leg()
    go2_leg.go2_channel = go2_channel
    go2_leg.init_state()
    go2_leg.ready_state()

    go2_leg.fr_tap()