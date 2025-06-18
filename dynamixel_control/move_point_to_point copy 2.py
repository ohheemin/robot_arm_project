import rclpy
import numpy as np
import math
from rclpy.node import Node
from dynamixel_sdk import *
import time
from scipy.optimize import minimize
from scipy.interpolate import CubicSpline

# === 공통 설정 ===
DEVICENAME = '/dev/ttyUSB0'

XM_PROTOCOL_VERSION = 2.0
AX_PROTOCOL_VERSION = 1.0
XC_PROTOCOL_VERSION = 2.0

# Dynamixel 제어 주소
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116
ADDR_PROFILE_VELOCITY = 112 

AX_ADDR_TORQUE_ENABLE = 24
AX_ADDR_GOAL_POSITION = 30
AX_ADDR_MOVING_SPEED = 32 # AX용

XC_ADDR_TORQUE_ENABLE = 64
XC_ADDR_GOAL_POSITION = 116
# XC의 프로필 속도 주소도 112 (XM과 동일)
# 사용자의 원래 코드에서는 XC_ADDR_GOAL_VELOCITY였으나, 일반적인 프로필 속도 주소로 통일

# 토크 상태
XM_TORQUE_ENABLE = 1
AX_TORQUE_ENABLE = 1
XC_TORQUE_ENABLE = 1

# DH Parameters: [theta_offset, d, a, alpha]
# IK 계산을 위한 조인트 순서 0부터 4까지로 가정.
DH_params = [
    [0,                 0.040,    0,        0],       # Joint 0 (Base 회전)
    [0,                 0.050,    0,  -np.pi/2],   # Joint 1 (어깨)
    [-np.pi/2,          0,        0.125,    0],       # Joint 2 (팔꿈치)
    [0,                 0,        0.13,     0],       # Joint 3 (손목 피치)
    [0,                 0,        0.10,     0],       # Joint 4 (손목 롤/요, 엔드 이펙터)
]

num_joints = len(DH_params)

# === 유틸리티 함수 ===
def angle_to_position(angle_deg, protocol_version):
    """
    각도(도)를 다이나믹셀 포지션 값으로 변환합니다.
    프로토콜 버전에 따라 스케일이 다릅니다 (1.0: 0-1023, 2.0: 0-4095).
    """
    if protocol_version == 1.0: # AX 시리즈
        return int(angle_deg / 300.0 * 1023)
    else: # XM/XC 시리즈
        return int(angle_deg / 360.0 * 4095)

def euler_to_rotation_matrix(roll_deg, pitch_deg, yaw_deg):
    """
    ZYX 외인성(extrinsic) 오일러 각(Roll, Pitch, Yaw)으로부터 회전 행렬을 생성합니다.
    R = Rz(yaw) @ Ry(pitch) @ Rx(roll)
    """
    roll_rad = np.radians(roll_deg)
    pitch_rad = np.radians(pitch_deg)
    yaw_rad = np.radians(yaw_deg)

    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(roll_rad), -np.sin(roll_rad)],
        [0, np.sin(roll_rad), np.cos(roll_rad)]
    ])

    Ry = np.array([
        [np.cos(pitch_rad), 0, np.sin(pitch_rad)],
        [0, 1, 0],
        [-np.sin(pitch_rad), 0, np.cos(pitch_rad)]
    ])

    Rz = np.array([
        [np.cos(yaw_rad), -np.sin(yaw_rad), 0],
        [np.sin(yaw_rad), np.cos(yaw_rad), 0],
        [0, 0, 1]
    ])
    
    return Rz @ Ry @ Rx

def forward_kinematics(joints):
    """
    주어진 조인트 각도로 엔드 이펙터의 최종 변환 행렬(4x4 동차 변환)을 계산합니다.
    """
    T = np.eye(4)
    for i, (theta_offset, d, a, alpha) in enumerate(DH_params):
        theta = joints[i] + theta_offset
        T_i = np.array([
            [np.cos(theta), -np.sin(theta)*np.cos(alpha),  np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
            [np.sin(theta),  np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
            [0,              np.sin(alpha),                np.cos(alpha),               d],
            [0,              0,                            0,                           1]
        ])
        T = T @ T_i
    return T # 최종 엔드 이펙터의 4x4 변환 행렬 반환

def inverse_kinematics(target_pose_matrix, initial_guess=np.zeros(num_joints)):
    """
    목표 자세(위치 및 방향을 포함하는 4x4 변환 행렬)에 도달하기 위한 조인트 각도를 계산합니다.
    scipy.optimize.minimize를 사용하여 위치 및 방향 오차를 최소화합니다.
    """
    target_pos = target_pose_matrix[:3, 3]
    target_rot = target_pose_matrix[:3, :3]

    # 각 조인트의 회전 범위 (라디안)
    bounds = [
        (math.radians(-180), math.radians(180)), # Joint 0
        (math.radians(-180), math.radians(180)), # Joint 1
        (math.radians(-150), math.radians(150)), # Joint 2
        (math.radians(-150), math.radians(150)), # Joint 3
        (math.radians(-180), math.radians(180)), # Joint 4
    ]

    def objective_function(joints):
        current_pose_matrix = forward_kinematics(joints)
        current_pos = current_pose_matrix[:3, 3]
        current_rot = current_pose_matrix[:3, :3]

        # 위치 오차 (유클리드 거리의 제곱)
        position_error = np.sum((current_pos - target_pos) ** 2)

        # 방향 오차 (회전 행렬의 Frobenius norm)
        orientation_error = np.sum((current_rot - target_rot) ** 2)

        orientation_weight = 1.0 
        
        return position_error + orientation_weight * orientation_error

    result = minimize(objective_function, initial_guess, method='L-BFGS-B', bounds=bounds)
    return result.x if result.success else initial_guess # 최적화된 조인트 각도 반환

# === 모터 제어 클래스 ===
class DynamixelMotor:
    """
    개별 다이나믹셀 모터를 제어하는 클래스.
    토크 설정, 목표 위치 이동 등의 기능을 포함합니다.
    """
    def __init__(self, port_handler, dxl_id, name, baudrate, protocol_version, node_logger, is_ax=False, is_xc=False):
        self.dxl_id = dxl_id
        self.baudrate = baudrate
        self.port_handler = port_handler
        self.packet_handler = PacketHandler(protocol_version)
        self.logger = node_logger
        self.name = name
        self.is_ax = is_ax
        self.is_xc = is_xc
        self.port_handler.setBaudRate(baudrate) # 포트 핸들러의 보드레이트 설정

    def setting(self):
        """
        모터의 보드레이트를 설정하고 토크를 활성화합니다.
        """
        if not self.port_handler.setBaudRate(self.baudrate):
            self.logger.error(f'{self.name} Baudrate 설정 실패')
            raise RuntimeError(f'{self.name} Baudrate 설정 실패')

        # 프로토콜 버전에 따라 토크 활성화 주소가 다름
        if self.is_ax:
            result, error = self.packet_handler.write1ByteTxRx(self.port_handler, self.dxl_id, AX_ADDR_TORQUE_ENABLE, AX_TORQUE_ENABLE)
        elif self.is_xc:
            result, error = self.packet_handler.write1ByteTxRx(self.port_handler, self.dxl_id, XC_ADDR_TORQUE_ENABLE, XC_TORQUE_ENABLE)
        else: # XM 시리즈
            result, error = self.packet_handler.write1ByteTxRx(self.port_handler, self.dxl_id, ADDR_TORQUE_ENABLE, XM_TORQUE_ENABLE)
        self._check_comm_result(result, error, 'Enable Torque')

    def move(self, angle_deg, velocity=50):
        """
        모터를 지정된 각도(도)로 이동시키고 속도를 설정합니다.
        """
        self.setting() # 이동 전에 토크 설정 및 보드레이트 확인
        protocol = AX_PROTOCOL_VERSION if self.is_ax else XC_PROTOCOL_VERSION if self.is_xc else XM_PROTOCOL_VERSION
        position = angle_to_position(angle_deg, protocol)

        # 프로토콜 및 모터 종류에 따라 제어 방식이 다름
        if self.is_ax:
            self.packet_handler.write2ByteTxRx(self.port_handler, self.dxl_id, AX_ADDR_MOVING_SPEED, velocity) # AX는 Moving Speed
            result, error = self.packet_handler.write2ByteTxRx(self.port_handler, self.dxl_id, AX_ADDR_GOAL_POSITION, position)
        elif self.is_xc:
            self.packet_handler.write4ByteTxRx(self.port_handler, self.dxl_id, ADDR_PROFILE_VELOCITY, velocity) # XC는 Profile Velocity
            result, error = self.packet_handler.write4ByteTxRx(self.port_handler, self.dxl_id, XC_ADDR_GOAL_POSITION, position)
        else: # XM
            self.packet_handler.write4ByteTxRx(self.port_handler, self.dxl_id, ADDR_PROFILE_VELOCITY, velocity) # XM은 Profile Velocity
            result, error = self.packet_handler.write4ByteTxRx(self.port_handler, self.dxl_id, ADDR_GOAL_POSITION, position)
        self._check_comm_result(result, error, f'Move to {angle_deg}°')

    def _check_comm_result(self, result, error, operation):
        """
        다이나믹셀 통신 결과를 확인하고 로깅합니다.
        """
        if result != COMM_SUCCESS:
            self.logger.error(f'{self.name} {operation} 실패: {self.packet_handler.getTxRxResult(result)}')
        elif error != 0:
            self.logger.warn(f'{self.name} {operation} 에러 발생: {self.packet_handler.getRxPacketError(error)}')
        else:
            self.logger.info(f'{self.name} {operation} 성공')

# === ROS2 노드 ===
class DynamixelControlNode(Node):
    """
    다이나믹셀 로봇 팔을 제어하는 ROS2 노드입니다.
    IK, 스플라인 궤적 생성, 모터 명령 전송 기능을 포함합니다.
    """
    def __init__(self):
        super().__init__('dynamixel_control_node')
        self.port_handler = PortHandler(DEVICENAME)
        if not self.port_handler.openPort():
            self.get_logger().error('포트 열기 실패')
            return
        self.get_logger().info('포트 열기 성공')

        # 모터 인스턴스 생성 (사용자 코드의 ID 및 모터 종류를 따름)
        self.base = DynamixelMotor(self.port_handler, 1, "base", 1000000,  XM_PROTOCOL_VERSION, self.get_logger())
        self.joint1 = DynamixelMotor(self.port_handler, 2, "joint1", 1000000,  XM_PROTOCOL_VERSION, self.get_logger())
        self.joint2 = DynamixelMotor(self.port_handler, 3, "joint2", 1000000,  AX_PROTOCOL_VERSION, self.get_logger(),is_ax=True)
        self.joint3 = DynamixelMotor(self.port_handler, 4,  "joint3", 1000000,  AX_PROTOCOL_VERSION, self.get_logger(), is_ax=True)
        self.end_effector = DynamixelMotor(self.port_handler, 5, "end_effector", 1000000,  XC_PROTOCOL_VERSION, self.get_logger(), is_xc=True)

    def move_to_initial_position(self):

        self.get_logger().info("--- Moving to initial position ---")
        self.base.move(212)
        self.joint1.move(182)
        self.joint2.move(150)
        self.joint3.move(240)
        self.end_effector.move(270)
        time.sleep(2.0) # 초기화 후 대기

    def cubic_spline_trajectory(self, joint_angles_initial, joint_angles_final, time_points):

        spline_trajectory = []
        for initial, final in zip(joint_angles_initial, joint_angles_final):
            times = [0, max(time_points)]  # 시간 0과 목표시간
            angles = [initial, final]
            spline = CubicSpline(times, angles, bc_type='clamped')
            spline_trajectory.append(spline(time_points))
        return np.array(spline_trajectory)

    def move_to_initial_position_again(self):
  
        self.get_logger().info("--- Moving to initial position again ---")
        self.base.move(212)
        self.joint1.move(182)
        self.joint2.move(150)
        self.joint3.move(240)
        # self.end_effector.move(355)
        # time.sleep(2.0)
        # self.end_effector.move(270) 
        # time.sleep(1.0)
    
    def move_to_joint(self, base_angle=212, joint1_angle=182, joint2_angle=150, joint3_angle=240):
 
        self.get_logger().info("--- Moving to  joint--")
        self.base.move(base_angle)
        self.joint1.move(joint1_angle)
        self.joint2.move(joint2_angle)
        self.joint3.move(joint3_angle)
        time.sleep(1.0)


    def move_to_xyz_rpy(self, x, y, z, roll_deg, pitch_deg, yaw_deg, mode='trajectory', velocity=50):
        """
        지정된 XYZ 위치와 Roll-Pitch-Yaw 방향으로 로봇 팔을 이동시킵니다.
        IK를 사용하여 조인트 각도를 계산하고 스플라인 또는 선형 보간으로 부드럽게 이동합니다.
        """

        def convert_to_motor_angles(joint_angles_deg):
            """DH 파라미터 기반 각도를 모터 제어 각도로 변환"""
            motor_base = 210 - joint_angles_deg[1]
            motor_joint1 = 182 - joint_angles_deg[2]
            motor_joint2 = 150 + joint_angles_deg[3]
            motor_joint3 = 240 + joint_angles_deg[4]
            return motor_base, motor_joint1, motor_joint2, motor_joint3

        # 목표 자세 (위치 및 방향) 행렬 생성
        target_rotation_matrix = euler_to_rotation_matrix(roll_deg, pitch_deg, yaw_deg)
        target_pose_matrix = np.eye(4)
        target_pose_matrix[:3, :3] = target_rotation_matrix
        target_pose_matrix[:3, 3] = np.array([x, y, z])

        self.get_logger().info(f"Target Pose Matrix:\n{target_pose_matrix}")

        joint_angles_rad = inverse_kinematics(target_pose_matrix, initial_guess=np.zeros(num_joints)) 
        self.get_logger().info(f"Calculated Joint Radian: {joint_angles_rad}")
        
        joint_angles_deg = np.degrees(joint_angles_rad)
        self.get_logger().info(f"Joint angles (degrees): {joint_angles_deg}")

        joint_angles_initial = np.zeros(num_joints)
        joint_angles_final = joint_angles_deg

        if mode == 'trajectory':
            time_points = np.linspace(0, 5, 20)
            spline_trajectory = self.cubic_spline_trajectory(joint_angles_initial, joint_angles_final, time_points)

            for t in range(len(time_points)):
                motor_angles = convert_to_motor_angles(spline_trajectory[:, t])
                self.base.move(motor_angles[0], velocity)
                self.joint1.move(motor_angles[1],velocity)
                self.joint2.move(motor_angles[2],velocity)
                self.joint3.move(motor_angles[3], velocity)
                time.sleep(0.1)

            self.end_effector.move(355)
            time.sleep(1.0)

        elif mode == 'direct':
            motor_angles = convert_to_motor_angles(joint_angles_deg)
            self.base.move(motor_angles[0],velocity)
            self.joint1.move(motor_angles[1],velocity)
            self.joint2.move(motor_angles[2], velocity)
            self.joint3.move(motor_angles[3], velocity)
            time.sleep(0.1)

        elif mode == 'linear':
            num_steps = 10
            joint_angles_diff = joint_angles_final - joint_angles_initial

            for step in range(num_steps):
                joint_angles_current = joint_angles_initial + (joint_angles_diff / num_steps) * step
                motor_angles = convert_to_motor_angles(joint_angles_current)
                self.base.move(motor_angles[0], velocity)
                self.joint1.move(motor_angles[1], velocity)
                self.joint2.move(motor_angles[2], velocity)
                self.joint3.move(motor_angles[3], velocity)
                time.sleep(0.05)



    def process(self):
        """
        로봇의 전체 픽 앤 플레이스(pick-and-place) 프로세스를 실행합니다.
        """
        # 초기 위치로 이동
        self.get_logger().info("초기 위치로 이동")
        self.move_to_initial_position()
            
        # 목표 위치 및 그리퍼 방향 설정
        gripper_orientation_r = 0
        gripper_orientation_p = 0
        gripper_orientation_y = 90

        self.get_logger().info("픽업대로 이동")
        # 실제 박스 위치: x값에 -0.095 추가
        pick_x = -0.095 - 0.14 
        pick_y = 0.0          
        pick_z_grasp = 0.04 + 0.01# 물체를 잡을 Z 좌표 (물체 높이에 맞춤)
    
        # 물체를 쌓을 위치 (Place)
        place_x = -0.095 - 0.14
        place_y = 0.0
        place_z = 0.04 + 0.01 # 물체를 쌓을 Z 좌표 (물체 높이에 맞춤)

        # 3. 물체 잡기 위치로 이동 (Pick Grasp)
        self.get_logger().info("POSITION 1: 물체 잡기 위치로 이동")
        
        self.move_to_xyz_rpy(pick_x, pick_y, pick_z_grasp + 0.1, 
                             gripper_orientation_r, gripper_orientation_p, gripper_orientation_y, mode='direct', velocity=50)
        
        self.move_to_xyz_rpy(pick_x, pick_y, pick_z_grasp, 
                             gripper_orientation_r, gripper_orientation_p, gripper_orientation_y, mode='linear', velocity=30)


        # 4. 그리퍼 닫기 (고정된 값으로 제어)
        self.get_logger().info("그리퍼 닫기")
        self.end_effector.move(355) 
        time.sleep(1.0)
        self.get_logger().info("물체 쌓기 위해 초기 위치로 이동")
        self.move_to_initial_position_again()

        # 5. 물체 쌓기 위치로 이동 (Place)
        self.get_logger().info("물체 쌓기 위치로 이동")
        self.move_to_xyz_rpy(pick_x, pick_y, pick_z_grasp + 0.1, 
                             gripper_orientation_r, gripper_orientation_p, gripper_orientation_y, mode='direct', velocity=50)
        self.move_to_xyz_rpy(pick_x, pick_y, pick_z_grasp, 
                             gripper_orientation_r, gripper_orientation_p, gripper_orientation_y, mode='linear', velocity=30)
        
        
        # 3. 물체 잡기 위치로 이동 (Pick Grasp)
        self.get_logger().info("\n--- Moving to grasp position ---")
        pick_z_grasp = 0.04 + 0.01# 물체를 잡을 Z 좌표 (물체 높이에 맞춤)
        self.move_to_xyz_rpy(place_x + 0.025, place_y, pick_z_grasp-0.025, 
                             gripper_orientation_r, gripper_orientation_p, gripper_orientation_y)

        self.move_to_xyz_rpy(pick_x + 0.025, pick_y, place_z + 10, 
                             gripper_orientation_r, gripper_orientation_p, gripper_orientation_y, 'direct')
        self.move_to_xyz_rpy(pick_x + 0.025, pick_y, place_z, 
                             gripper_orientation_r, gripper_orientation_p, gripper_orientation_y, 'linear')
        
        # 4. 그리퍼 열기 (고정된 값으로 제어)
        self.get_logger().info("그리퍼 열기")
        self.end_effector.move(355, velocity=10) 
        time.sleep(1.0)

        # 5. 물체 쌓기 위치로 이동 (Place)        
        self.move_to_initial_position_again()

        self.get_logger().info("픽 앤 플레이스 프로세스 완료")
        
# === main 함수 ===
def main(args=None):
    rclpy.init(args=args)
    node = DynamixelControlNode()
    node.process() # 픽 앤 플레이스 프로세스 실행
    rclpy.spin_once(node) # 노드 실행 (단일 스핀)
    node.destroy_node() # 노드 소멸
    rclpy.shutdown() # ROS2 시스템 종료

if __name__ == '__main__':
    main()
    