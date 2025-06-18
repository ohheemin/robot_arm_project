#!/usr/bin/env python3
import rclpy
import numpy as np
import math
from rclpy.node import Node
from dynamixel_sdk import *
import time
from scipy.optimize import minimize

# === 공통 설정 ===
DEVICENAME = '/dev/ttyUSB0'

XM_DXL_ID = [1, 2]  # XM1: joint2, XM2: base
AX_DXL_ID = [3, 4]  # AX1: joint1, AX2: joint3
XC_DXL_ID = [5]     # XC1: end-effector

XM_PROTOCOL_VERSION = 2.0
AX_PROTOCOL_VERSION = 1.0
XC_PROTOCOL_VERSION = 2.0

ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116
ADDR_PROFILE_VELOCITY = 112

AX_ADDR_TORQUE_ENABLE = 24
AX_ADDR_GOAL_POSITION = 30
AX_ADDR_MOVING_SPEED = 32

XC_ADDR_TORQUE_ENABLE = 64
XC_ADDR_GOAL_POSITION = 116
XC_ADDR_GOAL_VELOCITY = 112

XM_TORQUE_ENABLE = 1
AX_TORQUE_ENABLE = 1
XC_TORQUE_ENABLE = 1

# DH Parameters: [theta_offset, d, a, alpha]
DH_params = [
    [0,                 0.059,    0,        0],
    [0, 0.059,    0,  -np.pi/2],
    [-np.pi/2,   0,    0.13,        0],
    [0,   0,    0.13,        0],
    [0, 0, 0.10,  0],
]

num_joints = len(DH_params)

# === 유틸 함수 ===
def angle_to_position(angle_deg, protocol_version):
    if protocol_version == 1.0:
        return int(angle_deg / 300.0 * 1023)
    else:
        return int(angle_deg / 360.0 * 4095)

def forward_kinematics(joints):
    T = np.eye(4)
    positions = []
    for i, (theta_offset, d, a, alpha) in enumerate(DH_params):
        theta = joints[i] + theta_offset
        T_i = np.array([
            [np.cos(theta), -np.sin(theta)*np.cos(alpha),  np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
            [np.sin(theta),  np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
            [0,              np.sin(alpha),                np.cos(alpha),               d],
            [0,              0,                            0,                           1]
        ])
        T = T @ T_i
        positions.append(T[:3, 3])
    return positions

def inverse_kinematics(target_pos, initial_guess=np.zeros(num_joints)):
    bounds = [
        (math.radians(-180), math.radians(180)),
        (math.radians(-90), math.radians(90)),
        (math.radians(-180), math.radians(180)),
        (math.radians(-60), math.radians(0)),
        (math.radians(-180), math.radians(180)),
    ]

    # bounds = [
    #     (-180 - 0, 180 - 0),    # XM1 (base) 범위: -180° ~ 180° (기준값 + joint_angles[0])
    #     (0 - 150, 300 - 150),   # AX1 (joint1) 범위: 0° ~ 300° (기준값 150° + joint_angles[1])
    #     (-180 - 180, 180 - 180), # XM1 (joint2) 범위: -180° ~ 180° (기준값 180° + joint_angles[2])
    #     (0 - 150, 300 - 150),   # AX2 (joint3) 범위: 0° ~ 300° (기준값 150° + joint_angles[3])
    #     (-180 - 270, 180 - 270)  # XC1 (end-effector) 범위: -180° ~ 180° (기준값 270° + joint_angles[4])
    # ]

    def objective_function(joints):
        pos = forward_kinematics(joints)[-1]
        print(f"Current Position: {pos}")
        return np.sum((pos - target_pos) ** 2)

    #bounds = [(-np.pi/2, np.pi/2)] * num_joints
    #bounds[2] = (0, np.pi)
    result = minimize(objective_function, initial_guess, method='L-BFGS-B', bounds=bounds)
    return result.x if result.success else initial_guess

# === 모터 제어 클래스 ===
class DynamixelMotor:
    def __init__(self, port_handler, dxl_id, name, baudrate, protocol_version, node_logger, is_ax=False, is_xc=False):
        self.dxl_id = dxl_id
        self.baudrate = baudrate
        self.port_handler = port_handler
        self.packet_handler = PacketHandler(protocol_version)
        self.logger = node_logger
        self.name = name
        self.is_ax = is_ax
        self.is_xc = is_xc
        self.port_handler.setBaudRate(baudrate)

    def setting(self):
        self.port_handler.setBaudRate(self.baudrate)
        if not self.port_handler.setBaudRate(self.baudrate):
            self.logger.error(f'{self.name} Baudrate 설정 실패')
            raise RuntimeError(f'{self.name} Baudrate 설정 실패')

        if self.is_ax:
            result, error = self.packet_handler.write1ByteTxRx(self.port_handler, self.dxl_id, AX_ADDR_TORQUE_ENABLE, AX_TORQUE_ENABLE)
        elif self.is_xc:
            result, error = self.packet_handler.write1ByteTxRx(self.port_handler, self.dxl_id, XC_ADDR_TORQUE_ENABLE, XC_TORQUE_ENABLE)
        else:
            result, error = self.packet_handler.write1ByteTxRx(self.port_handler, self.dxl_id, ADDR_TORQUE_ENABLE, XM_TORQUE_ENABLE)
        self._check_comm_result(result, error, 'Enable Torque')

    def move(self, angle_deg, velocity=50):
        self.setting()
        protocol = AX_PROTOCOL_VERSION if self.is_ax else XC_PROTOCOL_VERSION if self.is_xc else XM_PROTOCOL_VERSION
        position = angle_to_position(angle_deg, protocol)

        if self.is_ax:
            self.packet_handler.write2ByteTxRx(self.port_handler, self.dxl_id, AX_ADDR_MOVING_SPEED, velocity)
            result, error = self.packet_handler.write2ByteTxRx(self.port_handler, self.dxl_id, AX_ADDR_GOAL_POSITION, position)
        elif self.is_xc:
            self.packet_handler.write4ByteTxRx(self.port_handler, self.dxl_id, XC_ADDR_GOAL_VELOCITY, velocity)
            result, error = self.packet_handler.write4ByteTxRx(self.port_handler, self.dxl_id, XC_ADDR_GOAL_POSITION, position)
        else:
            self.packet_handler.write4ByteTxRx(self.port_handler, self.dxl_id, ADDR_PROFILE_VELOCITY, velocity)
            result, error = self.packet_handler.write4ByteTxRx(self.port_handler, self.dxl_id, ADDR_GOAL_POSITION, position)
        self._check_comm_result(result, error, f'Move to {angle_deg}°')

    def _check_comm_result(self, result, error, operation):
        if result != COMM_SUCCESS:
            self.logger.error(f'{self.name} {operation} 실패: {self.packet_handler.getTxRxResult(result)}')
        elif error != 0:
            self.logger.warn(f'{self.name} {operation} 에러 발생: {self.packet_handler.getRxPacketError(error)}')
        else:
            self.logger.info(f'{self.name} {operation} 성공')

# === ROS2 노드 ===
class DynamixelControlNode(Node):
    def __init__(self):
        super().__init__('dynamixel_control_node')
        self.port_handler = PortHandler(DEVICENAME)
        if not self.port_handler.openPort():
            self.get_logger().error('포트 열기 실패')
            return
        self.get_logger().info('포트 열기 성공')

        # 모터 인스턴스 생성
        self.base = DynamixelMotor(self.port_handler, 1, "base", 1000000,  XM_PROTOCOL_VERSION, self.get_logger())  # base
        self.joint1 = DynamixelMotor(self.port_handler, 3, "joint1", 1000000,  AX_PROTOCOL_VERSION, self.get_logger(), is_ax=True)  # joint1
        self.joint2 = DynamixelMotor(self.port_handler, 2, "joint2", 1000000,  XM_PROTOCOL_VERSION, self.get_logger())  # joint2
        self.joint3 = DynamixelMotor(self.port_handler, 4,  "joint3", 1000000,  AX_PROTOCOL_VERSION, self.get_logger(), is_ax=True)  # joint3
        self.end_effector = DynamixelMotor(self.port_handler, 5, "end_effector", 1000000,  XC_PROTOCOL_VERSION, self.get_logger(), is_xc=True)  # end

    def move_to_initial_position(self):
        # 초기 위치로 이동 (각도를 0으로 설정)
        self.base.move(180)  # base
        self.joint1.move(150)  # joint1
        self.joint2.move(180)  # joint2
        self.joint3.move(150)  # joint3
        self.end_effector.move(270)  # end-effector

    def move_to_initial_position_again(self):
        # 초기 위치로 이동 (각도를 0으로 설정)
        self.base.move(180)  # base
        self.joint1.move(150)  # joint1
        self.joint2.move(180)  # joint2
        self.joint3.move(150)  # joint3
        self.end_effector.move(355)  # end-effector
          # 초기화 후 대기

    def process(self):
        # 초기 위치로 이동
        self.move_to_initial_position()

        # 목표 위치 (x, y, z) # 실제로 박스는 -0.19, -0.02, 0.095에 위치   
        target_pos = np.array([-0.19 - 0.04, 0, 0.095 + 0.050])
        joint_angles_rad = inverse_kinematics(target_pos)
        print(f"Calculated Joint Radian: {joint_angles_rad}")
        joint_angles_deg = np.degrees(joint_angles_rad)
        print("Joint angles (degrees):", joint_angles_deg)
        time.sleep(3.0)
        
        # 각도(deg)로 변환 후 모터 회전
        self.base.move(180 - joint_angles_deg[1])
        self.joint1.move(150 + joint_angles_deg[2])
        self.joint2.move(180 - joint_angles_deg[3])
        self.joint3.move(150 + joint_angles_deg[4])
        time.sleep(5.0)
        
        self.end_effector.move(355)
        time.sleep(1.0)
        self.move_to_initial_position_again()

# === main ===
def main(args=None):
    rclpy.init(args=args)
    node = DynamixelControlNode()
    node.process()
    rclpy.spin_once(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
