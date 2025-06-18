#!/usr/bin/env python3
import rclpy
import numpy as np
import math
import time
from rclpy.node import Node
from dynamixel_sdk import *
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from ikpy import chain
from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink

# === 공통 설정 ===
DEVICENAME = '/dev/ttyUSB0'

XM_DXL_ID = [2, 1]  # XM2: base, XM1: joint2
AX_DXL_ID = [1, 4]  # AX1: joint1, AX2: joint3
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

# === 유틸 함수 ===
def angle_to_position(angle_deg, protocol_version):
    if protocol_version == 1.0:
        return int(angle_deg / 300.0 * 1023)
    else:
        return int(angle_deg / 360.0 * 4095)

# === IKPY Chain 생성 ===
def create_robot_chain():
    dh_params = [
        [0,                 0.059,    0,        0],
        [0,                 0.059,    0,        -np.pi/2],
        [-np.pi/2,           0,    0.13,        0],
        [0,                   0,    0.13,        0],
        [0,                   0,    0.08,        0],
    ]
    links = [OriginLink()]
    for i, (theta_offset, d, a, alpha) in enumerate(dh_params):
        name = f"link_{i+1}"
        translation = [0, 0, d]
        orientation = [0, 0, alpha]
        if i > 0:
            # For subsequent links, translation is along x after rotation
            translation = [a, 0, d]

        rotation_axis = [0, 0, 1]  # Assuming revolute joints around Z

        links.append(
            URDFLink(
                name=name,
                origin_translation=translation,
                origin_orientation=orientation,
                rotation=rotation_axis
            )
        )
    return Chain(name='robot_arm', links=links)
# === 모터 제어 클래스 ===
class DynamixelMotor:
    def __init__(self, port_handler, dxl_id, baudrate, motor_name, protocol_version, node_logger, is_ax=False, is_xc=False):
        self.dxl_id = dxl_id
        self.baudrate = baudrate
        self.name = motor_name
        self.port_handler = port_handler
        self.packet_handler = PacketHandler(protocol_version)
        self.logger = node_logger
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
        self.motor_base = DynamixelMotor(self.port_handler, XM_DXL_ID[0], 1000000, 'XM2_Base', XM_PROTOCOL_VERSION, self.get_logger())
        self.ax_motor_joint1 = DynamixelMotor(self.port_handler, AX_DXL_ID[0], 115200, 'AX1_Joint1', AX_PROTOCOL_VERSION, self.get_logger(), is_ax=True)
        self.motor_joint2 = DynamixelMotor(self.port_handler, XM_DXL_ID[1], 57600, 'XM1_Joint2', XM_PROTOCOL_VERSION, self.get_logger())
        self.ax_motor_joint3 = DynamixelMotor(self.port_handler, AX_DXL_ID[1], 115200, 'AX2_Joint3', AX_PROTOCOL_VERSION, self.get_logger(), is_ax=True)
        self.xc_motor_end_effector = DynamixelMotor(self.port_handler, XC_DXL_ID[0], 115200, 'XC1_EndEffector', XC_PROTOCOL_VERSION, self.get_logger(), is_xc=True)

        # IKPY Chain 생성
        self.robot_chain = create_robot_chain()

        # Publisher 생성 (우리는 subscriber는 더 이상 필요 없음)
        self.publisher_ = self.create_publisher(JointState, '/joint_angles', 10)

    def move_to_initial_position(self):
        # 초기 위치로 이동 (각도를 기준값으로 설정)
        self.motor_base.move(0)           # XM2 (base)
        self.ax_motor_joint1.move(150)     # AX1 (joint1)
        self.motor_joint2.move(180)       # XM1 (joint2)
        self.ax_motor_joint3.move(150)     # AX2 (joint3)
        self.xc_motor_end_effector.move(270) # XC1 (end-effector)
        time.sleep(1.0)  # 초기화 후 대기

    def calculate_and_move(self, target_position):
        try:
            # 역기구학 계산
            inverse_kinematic_angles = self.robot_chain.inverse_kinematics(
                target_position=target_position,
                initial_position=[0.0] * 5  # 초기 관절 각도 추정치 (라디안)
            )
            print('1111111111')

            # 역기구학 계산 결과의 길이를 확인하고 인덱싱 오류를 방지
            if len(inverse_kinematic_angles) < 5:
                self.get_logger().warn(f"계산된 관절 각도가 예상보다 적음: {len(inverse_kinematic_angles)} 개")
                return

            # 첫 번째 OriginLink의 각도는 제외하고 실제 관절 각도만 사용
            joint_angles_rad = inverse_kinematic_angles  # OriginLink 제외
            joint_angles_deg = np.degrees(joint_angles_rad)

            self.get_logger().info(f"계산된 관절 각도 (deg): {joint_angles_deg}")

            # JointState 메시지 퍼블리시
            joint_state_msg = JointState()
            joint_state_msg.header.stamp = self.get_clock().now().to_msg()
            joint_state_msg.name = ['base', 'joint1', 'joint2', 'joint3', 'end_effector']
            joint_state_msg.position = joint_angles_rad.tolist()
            self.publisher_.publish(joint_state_msg)

            # 계산된 각도로 모터 제어
            self.motor_base.move(joint_angles_deg[0])
            self.ax_motor_joint1.move(150 + joint_angles_deg[1])
            self.motor_joint2.move(180 + joint_angles_deg[2])
            self.ax_motor_joint3.move(150 + joint_angles_deg[3])
            self.xc_motor_end_effector.move(270 + joint_angles_deg[4])

        except ValueError as e:
            self.get_logger().warn(f"역기구학 계산 실패: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = DynamixelControlNode()

    # 초기 위치로 이동
    node.move_to_initial_position()

    while rclpy.ok():
        try:
            x_str = input("Enter target x coordinate: ")
            y_str = input("Enter target y coordinate: ")
            z_str = input("Enter target z coordinate: ")

            target_x = float(x_str)
            target_y = float(y_str)
            target_z = float(z_str)

            target_position = [target_x, target_y, target_z]
            node.calculate_and_move(target_position)

        except ValueError:
            node.get_logger().warn("Invalid input. Please enter numeric values for coordinates.")
        except KeyboardInterrupt:
            node.get_logger().info("Exiting manual control.")
            break

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()