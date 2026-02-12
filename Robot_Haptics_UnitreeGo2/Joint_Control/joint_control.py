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

    def ReadMotorPosition(self, joint_idx: int = 0):
        if self.low_state is None or self.low_state.motor_state is None:
            return None
        return self.low_state.motor_state[joint_idx].q

    

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
        while self.go2_channel.low_state is None:
            time.sleep(0.01)
        for joint_idx in range(12):
            self._startPos[joint_idx] = self.go2_channel.ReadMotorPosition(joint_idx)
# lock joint, move joint


    def _move_f_to(self, q_target, t_move=1.2, kp=55.0, kd=4.0, lock_kp=60.0, lock_kd=5.0):
        """
        Smoothly move F joints (0,1,2,3,4,5) to q_target = [q0,q1,q2,q3,q4,q5].
        While moving, keep all other joints locked at their captured angles.
        """
        # start from current F
        q_now = [self.go2_channel.ReadMotorPosition(0),
                 self.go2_channel.ReadMotorPosition(1),
                 self.go2_channel.ReadMotorPosition(2),
                 self.go2_channel.ReadMotorPosition(3),
                 self.go2_channel.ReadMotorPosition(4),
                 self.go2_channel.ReadMotorPosition(5)]

        n = max(1, int(t_move * 200))
        for step in range(1, n + 1):
            alpha = step / float(n)
            q_cmd = [q_now[k]*(1.0 - alpha) + q_target[k]*alpha for k in range(6)]

            # command F joints
            for j in range(6):
                self.cmd.motor_cmd[j].mode = 0x01
                self.cmd.motor_cmd[j].q    = q_cmd[j]
                self.cmd.motor_cmd[j].kp   = kp
                self.cmd.motor_cmd[j].kd   = kd
                self.cmd.motor_cmd[j].dq   = 0.0
                self.cmd.motor_cmd[j].tau  = 0.0
           
            # send once per cycle
            self.send_cmd()
            time.sleep(1.0 / 200)


    def _move_fl_to(self, q_target, t_move=1.2,  kp=55.0, kd=4.0, lock_kp=60.0, lock_kd=5.0):
        """
        Smoothly move FL joints (3,4,5) to q_target = [q3,q4,q5].
        While moving, keep all other joints locked at their captured angles.
        """
        # start from current FL
        q_now = [self.go2_channel.ReadMotorPosition(3),
                 self.go2_channel.ReadMotorPosition(4),
                 self.go2_channel.ReadMotorPosition(5)]

        n = max(1, int(t_move * 200))
        for step in range(1, n + 1):
            alpha = step / float(n)
            q_cmd = [q_now[k]*(1.0 - alpha) + q_target[k]*alpha for k in range(3)]

            # command FL joints
            for j in range(3):
                i = j + 3  # FL indices are 3,4,5
                self.cmd.motor_cmd[i].mode = 0x01
                self.cmd.motor_cmd[i].q    = q_cmd[j]
                self.cmd.motor_cmd[i].kp   = kp
                self.cmd.motor_cmd[i].kd   = kd
                self.cmd.motor_cmd[i].dq   = 0.0
                self.cmd.motor_cmd[i].tau  = 0.0

            # keep others locked at captured angles
            self._lock_non_fl(kp=lock_kp, kd=lock_kd)

            # send once per cycle
            self.send_cmd()
            time.sleep(1.0 / 200)

        # small hold at the end to settle
        for _ in range(int(0.3 * 200)):
            for j in range(3):
                i = j + 3
                self.cmd.motor_cmd[i].mode = 0x01
                self.cmd.motor_cmd[i].q    = q_target[j]
                self.cmd.motor_cmd[i].kp   = kp
                self.cmd.motor_cmd[i].kd   = kd
                self.cmd.motor_cmd[i].dq   = 0.0
                self.cmd.motor_cmd[i].tau  = 0.0
            self._lock_non_fl(kp=lock_kp, kd=lock_kd)
            self.send_cmd()
            time.sleep(1.0 / 200)

    def _move_fr_to(self, q_target, t_move=1.2,  kp=55.0, kd=4.0, lock_kp=60.0, lock_kd=5.0):
        """
        Smoothly move FR joints (0,1,2) to q_target = [q0,q1,q2].
        While moving, keep all other joints locked at their captured angles.
        """
        # start from current FR
        q_now = [self.go2_channel.ReadMotorPosition(0),
                 self.go2_channel.ReadMotorPosition(1),
                 self.go2_channel.ReadMotorPosition(2)]

        n = max(1, int(t_move * 200))
        for step in range(1, n + 1):
            alpha = step / float(n)
            q_cmd = [q_now[k]*(1.0 - alpha) + q_target[k]*alpha for k in range(3)]

            # command FR joints
            for j in range(3):
                i = j  # FR indices are 0,1,2
                self.cmd.motor_cmd[i].mode = 0x01
                self.cmd.motor_cmd[i].q    = q_cmd[j]
                self.cmd.motor_cmd[i].kp   = kp
                self.cmd.motor_cmd[i].kd   = kd
                self.cmd.motor_cmd[i].dq   = 0.0
                self.cmd.motor_cmd[i].tau  = 0.0

            # keep others locked at captured angles
            self._lock_non_fr(kp=lock_kp, kd=lock_kd)

            # send once per cycle
            self.send_cmd()
            time.sleep(1.0 / 200)

        # small hold at the end to settle
        for _ in range(int(0.3 * 200)):
            for j in range(3):
                i = j
                self.cmd.motor_cmd[i].mode = 0x01
                self.cmd.motor_cmd[i].q    = q_target[j]
                self.cmd.motor_cmd[i].kp   = kp
                self.cmd.motor_cmd[i].kd   = kd
                self.cmd.motor_cmd[i].dq   = 0.0
                self.cmd.motor_cmd[i].tau  = 0.0
            self._lock_non_fr(kp=lock_kp, kd=lock_kd)
            self.send_cmd()
            time.sleep(1.0 / 200)

    def _hold_current_Pos(self, q_target, t_lock_time = 5.0,  kp=60.0, kd=5.0):
        """
        Lock all joints at their current positions.
        """
        n = max(1, int(t_lock_time * 200))
        for step in range(1, n + 1):
            for joint_idx in range(12):
                self.cmd.motor_cmd[joint_idx].mode = 0x01
                self.cmd.motor_cmd[joint_idx].q = q_target[joint_idx]
                self.cmd.motor_cmd[joint_idx].kp = kp
                self.cmd.motor_cmd[joint_idx].kd = kd
                self.cmd.motor_cmd[joint_idx].dq = 0.0
                self.cmd.motor_cmd[joint_idx].tau = 0.0
            self.send_cmd()
            time.sleep(1.0 / 200)



    def _lock_non_fl(self, kp=60.0, kd=5.0):
        """
        Hold all non-FL joints (indices 0-2, 6..11) at locked_q with stiffness.
        Call this repeatedly while moving FL so they stay locked.
        """
        for i in range(0, 3):
            self.cmd.motor_cmd[i].mode = 0x01
            self.cmd.motor_cmd[i].kp   = kp
            self.cmd.motor_cmd[i].kd   = kd
            self.cmd.motor_cmd[i].dq   = 0.0
            self.cmd.motor_cmd[i].tau  = 0.0

        for i in range(6, 12):
            self.cmd.motor_cmd[i].mode = 0x01
            self.cmd.motor_cmd[i].kp   = kp
            self.cmd.motor_cmd[i].kd   = kd
            self.cmd.motor_cmd[i].dq   = 0.0
            self.cmd.motor_cmd[i].tau  = 0.0

    def _lock_non_fr(self, kp=60.0, kd=5.0):
        """
        Hold all non-FR joints (indices 3..11) at locked_q with stiffness.
        Call this repeatedly while moving FR so they stay locked.
        """
        for i in range(3, 12):
            self.cmd.motor_cmd[i].mode = 0x01
            self.cmd.motor_cmd[i].kp   = kp
            self.cmd.motor_cmd[i].kd   = kd
            self.cmd.motor_cmd[i].dq   = 0.0
            self.cmd.motor_cmd[i].tau  = 0.0


    def stable(self):
            # # Manual stable the edu
        while True:
            for joint_idx in range(12):
                self.cmd.motor_cmd[joint_idx].mode = 0x01
                self.cmd.motor_cmd[joint_idx].kp = 60.0
                self.cmd.motor_cmd[joint_idx].kd = 5.0
                self.cmd.motor_cmd[joint_idx].tau = 0.0
                self.send_cmd()

if __name__ == '__main__':
    if len(sys.argv)>1:
        go2_channel = Go2Channel(sys.argv[1])
    else:
        go2_channel = Go2Channel()

    go2_leg = Go2Leg()
    go2_leg.go2_channel = go2_channel
    go2_leg.init_state()
    go2_leg.ready_state()

    # Call the methods here.