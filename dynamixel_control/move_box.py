import rclpy
import numpy as np
import math
from rclpy.node import Node
from dynamixel_sdk import *
import time
from scipy.optimize import minimize
from scipy.interpolate import CubicSpline
from scipy.optimize import differential_evolution
from ikpy import chain
from ikpy import utils
from ikpy.chain import Chain
from ikpy.link import URDFLink
# === 공통 설정 (기존과 동일) ===
DEVICENAME = '/dev/ttyUSB0'
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
XM_TORQUE_ENABLE = 1
AX_TORQUE_ENABLE = 1
XC_TORQUE_ENABLE = 1
# theta d a alpha
DH_params = [
    [0,        0.065, 0,       0],         # Joint 0
    [0,        0.052, 0,  np.pi/2],         # Joint 1
    [np.pi/2,  0,      0.126296,   0],         # Joint 2
    [0,        0,    0.123994,    0],         # Joint 3
    [np.pi/2,  0,     0.08,    np.pi/2],         # Joint 4 
]

num_joints = len(DH_params)
# === 유틸리티 함수 (기존과 동일) ===
def angle_to_position(angle_deg, protocol_version):
    if protocol_version == 1.0:
        return int(angle_deg / 300.0 * 1023)
    else:
        return int(angle_deg / 360.0 * 4095)

def euler_to_rotation_matrix(roll_deg, pitch_deg, yaw_deg):
    roll_rad = np.radians(roll_deg)
    pitch_rad = np.radians(pitch_deg)
    yaw_rad = np.radians(yaw_deg)
    Rx = np.array([[1, 0, 0], [0, np.cos(roll_rad), -np.sin(roll_rad)], [0, np.sin(roll_rad), np.cos(roll_rad)]])
    Ry = np.array([[np.cos(pitch_rad), 0, np.sin(pitch_rad)], [0, 1, 0], [-np.sin(pitch_rad), 0, np.cos(pitch_rad)]])
    Rz = np.array([[np.cos(yaw_rad), -np.sin(yaw_rad), 0], [np.sin(yaw_rad), np.cos(yaw_rad), 0], [0, 0, 1]])
    return Rz @ Ry @ Rx

def forward_kinematics(joints):
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
    return T
# === IKPY 모델 생성 (수정) ===
def create_ikpy_chain(dh_params):
    links = [
        URDFLink(
            name="base_link",
            origin_translation=[0, 0, 0],
            origin_orientation=[0, 0, 0],
            rotation=[0, 0, 0]  # 고정 베이스
        )
    ]

    for i, params in enumerate(dh_params):
        theta_offset, d, a, alpha = params

        if i == 0:  # Joint 0: z축 회전
            links.append(
                URDFLink(
                    name=f"joint_{i}",
                    origin_translation=[0, 0, d],
                    origin_orientation=[0, 0, 0],
                    rotation=[0, 0, 1],
                    bounds=(-np.radians(100), np.radians(100))
                )
            )
        elif i == 1:  # Joint 1: z축 회전
            links.append(
                URDFLink(
                    name=f"joint_{i}",
                    origin_translation=[0, 0, d],
                    origin_orientation=[alpha, 0, 0],
                    rotation=[0, 0, 1],
                    bounds=(-np.radians(180), np.radians(180))
                )
            )
        elif i == 2:  # Joint 2: z축 회전
            links.append(
                URDFLink(
                    name=f"joint_{i}",
                    origin_translation=[a, 0, 0],
                    origin_orientation=[0, 0, 0],
                    rotation=[0, 0, 1],
                    bounds=(-np.radians(130), np.radians(130)
                )
            )
            )
        elif i == 3:  # Joint 3: z축 회전 (기존 y축 회전 → z축 회전으로 맞춤)
            links.append(
                URDFLink(
                    name=f"joint_{i}",
                    origin_translation=[a, 0, 0],
                    origin_orientation=[alpha, 0, 0],
                    rotation=[0, 0, 1],
                    bounds=(-np.radians(120), np.radians(120))
                )
            )
        elif i == 4:  # Joint 4: 프리즘 조인트 (x축 방향 슬라이더)
            links.append(
                URDFLink(
                    name=f"joint_{i}",
                    origin_translation=[a, 0, 0],
                    origin_orientation=[alpha, 0, 0],
                    rotation=[0, 0, 0], # 회전 없음
                    bounds=(-np.radians(180), np.radians(180))
                )
            )
        else:
            raise ValueError("Unexpected joint index")

    return Chain(name="arm", links=links)

ikpy_chain = create_ikpy_chain(DH_params)
# 모든 theta를 0으로 설정 (프리즘 조인트는 d 값)
#initial_joints = [0.0] * (len(DH_params)) + [0.0] # 회전 조인트는 0, 프리즘도 초기값을 0으로 가정
initial_joints = [0, 0, np.pi/2, 0, 0,  np.deg2rad(270)]
# Forward kinematics 계산
fk_results = ikpy_chain.forward_kinematics(initial_joints)
# End-effector의 위치 정보
print("\n--- End-effector 위치 (theta=0) ---")
print(f"위치: {fk_results}")
print(f"변환 행렬:\n{fk_results[-1]}")

# === IKPY를 이용한 역기구학 함수 (수정됨) ===
def inverse_kinematics_ikpy(target_position, target_orientation=None, initial_guess=None, orientation_mode=None):
    """
    ikpy 라이브러리를 사용하여 목표 위치 및 방향에 대한 역기구학을 계산합니다.
    """
    print(f"목표 위치: {target_position}")
    print(f"목표 방향: {target_orientation}")
    print('1111111111111111111111')
    if initial_guess is None:
        print("초기 추정값:", initial_guess)
    else:
        initial_guess = np.array(initial_guess)
        print("초기 추정값(else):", initial_guess)
    # initial_guess = np.zeros(6) # 이 줄이 삭제되었습니다.
    ik_results = ikpy_chain.inverse_kinematics(
        target_position=target_position,
        target_orientation=target_orientation,  # None 대신 target_orientation 사용
        initial_position=initial_guess,
        orientation_mode=orientation_mode, # orientation_mode 사용
    )
    print(f"IKPY 결과: {ik_results}")
    return ik_results

def local_to_global_pick(origin_x, origin_y, dx, dy, angle_deg):
    theta = np.deg2rad(angle_deg)
    xg = origin_x + dx*math.cos(abs(theta)) 
    yg = origin_y + dx*math.sin(abs(theta)) 
    return xg, yg

def local_to_global_box(origin_x, origin_y, dx, dy, angle_deg):
    theta = np.deg2rad(angle_deg)
    xg = origin_x + dy*math.cos(abs(theta)) + dx*math.cos(abs(theta)) 
    yg = origin_y + dy*math.sin(abs(theta)) - dx*math.sin(abs(theta)) 
    return xg, yg

def local_to_global_place(origin_x, origin_y, dx, dy, angle_deg):
    theta = np.deg2rad(angle_deg)
    xg = origin_x + dx*math.cos(abs(theta)) 
    yg = origin_y - dx*math.sin(abs(theta)) 
    return xg, yg


# === 모터 제어 클래스 (기존과 동일) ===
class DynamixelMotor():
    ADDR_PRESENT_POSITION = 132
    AX_ADDR_PRESENT_POSITION = 36
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
            self.packet_handler.write4ByteTxRx(self.port_handler, self.dxl_id, ADDR_PROFILE_VELOCITY, velocity)
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
    def get_present_position(self):
        if self.is_ax:
            data, result, error = self.packet_handler.read2ByteTxRx(self.port_handler, self.dxl_id, self.AX_ADDR_PRESENT_POSITION)
            if result == COMM_SUCCESS and error == 0:
                return data * 300.0 / 1023
        else:
            data, result, error = self.packet_handler.read4ByteTxRx(self.port_handler, self.dxl_id, self.ADDR_PRESENT_POSITION)
            if result == COMM_SUCCESS and error == 0:
                return data * 360.0 / 4095
        self.logger.error(f'{self.name} 현재 위치 읽기 실패')
        return None
    def wait_until_goal_reached(self, goal_angle_deg, timeout_sec=10.0, tolerance_deg=8.0):
        goal_position = angle_to_position(goal_angle_deg, AX_PROTOCOL_VERSION if self.is_ax else XC_PROTOCOL_VERSION if self.is_xc else XM_PROTOCOL_VERSION)
        start_time = time.time()
        while True:
            present_position_deg = self.get_present_position()
            if present_position_deg is not None:
                present_position = angle_to_position(present_position_deg, AX_PROTOCOL_VERSION if self.is_ax else XC_PROTOCOL_VERSION if self.is_xc else XM_PROTOCOL_VERSION)
                if abs(present_position - goal_position) <= angle_to_position(tolerance_deg, AX_PROTOCOL_VERSION if self.is_ax else XC_PROTOCOL_VERSION if self.is_xc else XM_PROTOCOL_VERSION):
                    self.logger.info(f'{self.name} 목표 각도 {goal_angle_deg}° 도달')
                    return True
                if time.time() - start_time > timeout_sec:
                    self.logger.warn(f'{self.name} 목표 각도 {goal_angle_deg}° 도달 실패 (타임아웃)')
                    self.logger.warn(f'{self.name} 현재 각도: {present_position_deg}° (오차: {abs(present_position - goal_position) * 360.0 / (4095.0 if not self.is_ax else 1023.0)}°)')
                    return False
            else:
                self.logger.warn(f'{self.name} 현재 각도 읽기 실패')
                if time.time() - start_time > timeout_sec:
                    return False
            time.sleep(0.01)

# === ROS2 노드 (수정됨) ===
class DynamixelControlNode(Node):
    def __init__(self, angle1, angle2, distance1, distance2):
        super().__init__('dynamixel_control_node')
        self.port_handler = PortHandler(DEVICENAME)
        if not self.port_handler.openPort():
            self.get_logger().error('포트 열기 실패')
            return
        self.get_logger().info('포트 열기 성공')
        self.base = DynamixelMotor(self.port_handler, 1, "base", 1000000,  XM_PROTOCOL_VERSION, self.get_logger())
        self.joint1 = DynamixelMotor(self.port_handler, 2, "joint1", 1000000,  XM_PROTOCOL_VERSION, self.get_logger())
        self.joint2 = DynamixelMotor(self.port_handler, 3, "joint2", 1000000,  AX_PROTOCOL_VERSION, self.get_logger(),is_ax=True)
        self.joint3 = DynamixelMotor(self.port_handler, 4,  "joint3", 1000000,  AX_PROTOCOL_VERSION, self.get_logger(), is_ax=True)
        self.end_effector = DynamixelMotor(self.port_handler, 5, "end_effector", 1000000,  XC_PROTOCOL_VERSION, self.get_logger(), is_xc=True)
        self.previous_joint_angles_rad = np.array([0, 0, np.pi/2, 0, 0,  0.1]) 
        self.angle1 = angle1
        self.angle2 = angle2
        self.distance1 = distance1
        self.distance2 = distance2
    
    def read_current_angle(self):
            current_angles = []
            current_angles_final = [0,0,0,0,0,0]
            for motor in [self.base, self.joint1, self.joint2, self.joint3, self.end_effector]:
                position_deg = motor.get_present_position()
                if position_deg is None:
                    return None
                current_angles.append(position_deg)
            print("current_angles", current_angles)
            current_angles_final[0] = 0
            current_angles_final[1] = 210 - current_angles[0] # Joint 0 -> Base\
            current_angles_final[2] = 179 + 90 - current_angles[1] # Joint 1 -> Joint1
            current_angles_final[3] = -150 + current_angles[2] # Joint 2 -> Joint2
            current_angles_final[4] = -150 + current_angles[3] # Joint 3 -> Joint3
            current_angles_final[5] = 270 - current_angles[4] # Joint 4 -> End Effector
            self.get_logger().info(f"현재 각도: {current_angles}")
            return current_angles_final


    def read_current_motor_angle(self):
            current_angles = []
            for motor in [self.base, self.joint1, self.joint2, self.joint3, self.end_effector]:
                position_deg = motor.get_present_position()
                if position_deg is None:
                    return None
                current_angles.append(position_deg)
            print("current_angles", current_angles)
            # motor_base = 210 - joint_angles_deg[0] # Joint 0 -> Base
            # motor_joint1 = 179 - (joint_angles_deg[1] - 90) # Joint 1 -> Joint1
            # motor_joint2 = 150 + joint_angles_deg[2] # Joint 2 -> Joint2
            # motor_joint3 = 150 + joint_angles_deg[3] # Joint 3 -> Joint3
            return current_angles
    
    def rotate_joint(self, joint=None, angle=10, velocity=50):
        current_angles = self.read_current_motor_angle()
        if current_angles is None:
            return
        print(current_angles)
        if joint == 'base':
            new_angle = current_angles[0] + angle
            self.base.move(new_angle, velocity)
            self.base.wait_until_goal_reached(new_angle)
        elif joint == 'joint1':
            new_angle = current_angles[1] + angle
            self.joint1.move(new_angle, velocity)
            self.joint1.wait_until_goal_reached(new_angle)
        elif joint == 'joint2':
            new_angle = current_angles[2] + angle
            self.joint2.move(new_angle, velocity)
            self.joint2.wait_until_goal_reached(new_angle)
        elif joint == 'joint3':
            new_angle = current_angles[3] + angle
            self.joint3.move(new_angle, velocity)
            self.joint3.wait_until_goal_reached(new_angle)
        elif joint == 'end_effector':   
            new_angle = current_angles[4] + angle
            self.end_effector.move(new_angle, velocity)
            self.end_effector.wait_until_goal_reached(new_angle)
        else:
            self.get_logger().error("Invalid joint name")
            return
        time.sleep(1.0)


    def rotation_matrix_y(self,theta_rad):
        return np.array([
            [np.cos(theta_rad), 0, np.sin(theta_rad)],
            [0, 1, 0],
            [-np.sin(theta_rad), 0, np.cos(theta_rad)]
        ])


    def move_to_initial_position(self, velocity=50):
        self.get_logger().info("--- Moving to initial position ---")
        self.base.move(212, velocity)
        self.joint1.move(179, velocity)
        self.joint2.move(150-40,velocity)  
        self.joint3.move(150-90, velocity)
        self.end_effector.move(270, velocity)
        self.base.wait_until_goal_reached(212)
        self.joint1.wait_until_goal_reached(179)
        self.joint2.wait_until_goal_reached(150-40)
        self.joint3.wait_until_goal_reached(150-90)
        self.end_effector.wait_until_goal_reached(270)
        time.sleep(2.0)


    def move_to_box_position(self, velocity=50):
        self.get_logger().info("--- Moving to initial position ---")
        self.base.move(212, velocity)
        self.joint1.move(179, velocity)
        self.joint2.move(150-90,velocity)  
        self.joint3.move(150-90, velocity)
        self.base.wait_until_goal_reached(212)
        self.joint1.wait_until_goal_reached(179)
        self.joint2.wait_until_goal_reached(150-90)
        self.joint3.wait_until_goal_reached(150-90)
        time.sleep(2.0)


    def cubic_spline_trajectory(self, joint_angles_initial, joint_angles_final, time_points):
            spline_trajectory = []
            for initial, final in zip(joint_angles_initial, joint_angles_final):
                times = [0, max(time_points)]
                angles = [initial, final]
                spline = CubicSpline(times, angles, bc_type='clamped')
                spline_trajectory.append(spline(time_points))
            return np.array(spline_trajectory)

    def move_to_initial_position_again(self, velocity = 30):
        self.get_logger().info("--- Moving to initial position again ---")
        self.joint1.move(179, velocity)
        self.joint1.wait_until_goal_reached(179)
        self.base.move(212, velocity)
        # self.joint1.move(179, velocity)
        self.joint2.move(150-40, velocity)
        self.joint3.move(150-90, velocity)
        self.base.wait_until_goal_reached(212)
        # self.joint1.wait_until_goal_reached(179)
        self.joint2.wait_until_goal_reached(150-40)
        self.joint3.wait_until_goal_reached(150-90)

    def move_to_joint(self, base_angle=212, joint1_angle=179, joint2_angle=150, joint3_angle=150):
        self.get_logger().info("--- Moving to  joint--")
        self.base.move(base_angle)
        self.joint1.move(joint1_angle)
        self.joint2.move(joint2_angle)
        self.joint3.move(joint3_angle)
        self.base.wait_until_goal_reached(base_angle)
        self.joint1.wait_until_goal_reached(joint1_angle)
        self.joint2.wait_until_goal_reached(joint2_angle)
        self.joint3.wait_until_goal_reached(joint3_angle)
        time.sleep(1.0)

    def move_to_xyz_rpy(self, x, y, z, target_orientation=None, mode='trajectory',velocity=50, orientation=None):
        """
        ikpy를 사용하여 목표 위치 및 방향으로 로봇 팔을 이동시킵니다.
        """
        def convert_to_motor_angles(joint_angles_rad):
            """ikpy에서 계산된 라디안 각도를 모터 제어 각도로 변환"""
            joint_angles_deg = np.degrees(joint_angles_rad)
            motor_base = 210 - joint_angles_deg[0] # Joint 0 -> Base
            motor_joint1 = 179 - (joint_angles_deg[1] - 90) # Joint 1 -> Joint1
            motor_joint2 = 150 + joint_angles_deg[2] # Joint 2 -> Joint2
            motor_joint3 = 150 + joint_angles_deg[3] # Joint 3 -> Joint3
            return motor_base, motor_joint1, motor_joint2, motor_joint3
        
        target_position = np.array([x, y, z])
        # target_orientation = euler_to_rotation_matrix(roll_deg, pitch_deg, yaw_deg)
        previent_joint_angles = self.read_current_angle()
        print('previent_joint_angles', previent_joint_angles)
        if previent_joint_angles is not None:
            previent_joint_angles_rad = np.deg2rad(previent_joint_angles[0:6])
        else:
            previent_joint_angles_rad = np.array([0, 0, np.pi/2, 0, 0,  0.1]) # 초기값 설정
        
        #previent_joint_angles_rad = np.array([0, 0, np.pi/2, 0, 0,  0.1])
        joint_angles_rad = inverse_kinematics_ikpy(
            target_position=target_position,
            target_orientation=target_orientation,
            initial_guess=previent_joint_angles_rad,
            orientation_mode=orientation
        )
        self.get_logger().info(f"Calculated Joint Radian (IKPY): {joint_angles_rad}")
        # self.previous_joint_angles_rad = joint_angles_rad
        # joint_angles_rad = [-2.04684122e-04, -5.04204366e-14,  1.59002439e+00,  2.07496181e+00,
        #                     -5.23393547e-01,  3.74827733e-03]

        degrees = np.rad2deg(joint_angles_rad)
        self.get_logger().info(f"(degrees): {degrees}")
        motor_angles = convert_to_motor_angles(joint_angles_rad[1:5]) # 처음 4개 관절만 사용
        self.get_logger().info(f"Motor angles (degrees): {motor_angles}")

        if mode == 'trajectory':
            current_motor_angles = self.read_current_motor_angle()
            if all(angle is not None for angle in current_motor_angles):
                time_points = np.linspace(0, 5, 20)
                spline_trajectory = self.cubic_spline_trajectory(current_motor_angles, motor_angles, time_points)
                for t in range(len(time_points)):
                    self.base.move(spline_trajectory[0][t], velocity)
                    self.joint1.move(spline_trajectory[1][t], velocity)
                    self.joint2.move(spline_trajectory[2][t], velocity)
                    self.joint3.move(spline_trajectory[3][t], velocity)
                    time.sleep(0.1)
            else:
                self.get_logger().warn("현재 모터 각도를 읽을 수 없어 궤적 생성을 건너뜁니다.")

        elif mode == 'direct':
            if target_orientation == [0,0,-1]:
                self.joint3.move(motor_angles[3], velocity)
                finish_joint_3_flag = self.joint3.wait_until_goal_reached(motor_angles[3])
                if finish_joint_3_flag:
                    self.base.move(motor_angles[0], velocity)
                    self.joint1.move(motor_angles[1], velocity)
                    self.joint2.move(motor_angles[2], velocity)
                    self.base.wait_until_goal_reached(motor_angles[0])
                    self.joint1.wait_until_goal_reached(motor_angles[1])
                    self.joint2.wait_until_goal_reached(motor_angles[2])
                    time.sleep(0.1)
                time.sleep(0.1)
            elif target_orientation == [1,0,0]:
                self.joint3.move(motor_angles[3], velocity)
                finish_joint_3_flag = self.joint3.wait_until_goal_reached(motor_angles[3])
                if finish_joint_3_flag:
                    self.base.move(motor_angles[0], velocity)
                    self.joint1.move(motor_angles[1], velocity)
                    self.joint2.move(motor_angles[2], velocity)
                # self.base.wait_until_goal_reached(motor_angles[0])
                # self.joint1.wait_until_goal_reached(motor_angles[1])
                # self.joint3.wait_until_goal_reached(motor_angles[2])
                    self.base.wait_until_goal_reached(motor_angles[0])
                    self.joint1.wait_until_goal_reached(motor_angles[1])
                    self.joint2.wait_until_goal_reached(motor_angles[2])
                    time.sleep(0.1)
        elif mode == 'linear':
            num_steps = 10
            current_motor_angles = [self.base.get_present_position(),
                                    self.joint1.get_present_position(),
                                    self.joint2.get_present_position(),
                                    self.joint3.get_present_position()]
            if all(angle is not None for angle in current_motor_angles):
                motor_angles_diff = np.array(motor_angles) - np.array(current_motor_angles)
                for step in range(num_steps):
                    interpolated_angles = np.array(current_motor_angles) + (motor_angles_diff / num_steps) * step
                    self.base.move(interpolated_angles[0], velocity)
                    self.joint1.move(interpolated_angles[1], velocity)
                    self.joint2.move(interpolated_angles[2], velocity)
                    self.joint3.move(interpolated_angles[3], velocity)
                    time.sleep(0.05)
                self.base.wait_until_goal_reached(motor_angles[0])
                self.joint1.wait_until_goal_reached(motor_angles[1])
                self.joint2.wait_until_goal_reached(motor_angles[2])
                self.joint3.wait_until_goal_reached(motor_angles[3])
            else:
                self.get_logger().warn("현재 모터 각도를 읽을 수 없어 선형 보간을 수행할 수 없습니다.")
        elif mode == 'under_first':
            self.joint1.move(motor_angles[1], velocity)
            if target_orientation == [0,0,-1]:
                self.joint3.move(motor_angles[3], velocity)
                finish_joint_3_flag = self.joint3.wait_until_goal_reached(motor_angles[3])
                if finish_joint_3_flag:
                    self.base.move(motor_angles[0], velocity)
                    self.joint1.move(motor_angles[1], velocity)
                    self.joint2.move(motor_angles[2], velocity)
                    self.base.wait_until_goal_reached(motor_angles[0])
                    self.joint1.wait_until_goal_reached(motor_angles[1])
                    self.joint2.wait_until_goal_reached(motor_angles[2])
                    time.sleep(0.1)
                time.sleep(0.1)
            elif target_orientation == [1,0,0]:
                self.joint3.move(motor_angles[3], velocity)
                finish_joint_3_flag = self.joint3.wait_until_goal_reached(motor_angles[3])
                if finish_joint_3_flag:
                    self.base.move(motor_angles[0], velocity)
                    self.joint1.move(motor_angles[1], velocity)
                    self.joint2.move(motor_angles[2], velocity)
                    self.base.wait_until_goal_reached(motor_angles[0])
                    self.joint1.wait_until_goal_reached(motor_angles[1])
                    self.joint2.wait_until_goal_reached(motor_angles[2])
                    time.sleep(0.1)

        elif mode == 'place':
            if target_orientation == [0,0,-1]:
                self.joint1.move(motor_angles[1], velocity)
                self.joint1.wait_until_goal_reached(motor_angles[1])
                self.joint2.move(motor_angles[2], velocity)
                self.base.move(motor_angles[0], velocity)
                self.joint2.wait_until_goal_reached(motor_angles[2])
                self.base.wait_until_goal_reached(motor_angles[0])
                self.joint3.move(motor_angles[3], velocity)
                self.joint3.wait_until_goal_reached(motor_angles[3])
  
                time.sleep(0.1)
            elif target_orientation == [1,0,0]:
                self.joint3.move(motor_angles[3], velocity)
                finish_joint_3_flag = self.joint3.wait_until_goal_reached(motor_angles[3])
                if finish_joint_3_flag:
                    self.base.move(motor_angles[0], velocity)
                    self.joint1.move(motor_angles[1], velocity)
                    self.joint2.move(motor_angles[2], velocity)
                    self.base.wait_until_goal_reached(motor_angles[0])
                    self.joint1.wait_until_goal_reached(motor_angles[1])
                    self.joint2.wait_until_goal_reached(motor_angles[2])
                    time.sleep(0.1)
    def open_box(self, grip_x, grip_y, grip_z, angle):

        target_x_axis_1 = [1, 0, 0]
        target_x_axis_2 = [0, 0, -1]
        self.get_logger().info("초기 위치로 이동")
        self.move_to_initial_position()
        self.rotate_joint('base',angle)
        if self.angle2 is not None and self.distance2 is not None:
            print('111111111111111111111111111111111111111111111111')
            self.move_to_xyz_rpy(grip_x - 0.05, grip_y, grip_z + 0.10, mode='direct', velocity=50)
            
            self.rotate_joint('joint3',-30)

            self.move_to_xyz_rpy(grip_x, grip_y, grip_z + 0.04,
                                target_x_axis_2, mode='linear',  velocity=50, orientation='X')                

            self.move_to_xyz_rpy(grip_x, grip_y, grip_z-0.01,
                                target_x_axis_2, mode='linear', velocity=20, orientation='X')

            # self.get_logger().info("그리퍼 열기")
            # self.end_effector.move(330, velocity=10)
            # self.end_effector.wait_until_goal_reached(330)

            self.rotate_joint('joint3', -60, velocity=20)

            time.sleep(1.0)
        else:
            self.get_logger().info("픽업대로 이동")
            self.move_to_xyz_rpy(grip_x - 0.05, grip_y, grip_z + 0.2,
                            target_x_axis_1, mode='direct', velocity=50, orientation='X')
            
            self.move_to_xyz_rpy(grip_x, grip_y, grip_z,
                                target_x_axis_2, mode='direct', velocity=50, orientation='X')

            self.get_logger().info("그리퍼 열기")
            self.end_effector.move(270, velocity=10)
            self.end_effector.wait_until_goal_reached(270)
            self.move_to_xyz_rpy(grip_x-0.1, grip_y, grip_z,
                                target_x_axis_2, mode='linear', velocity=50, orientation='X')
            time.sleep(1.0)


    def close_box(self, grip_x, grip_y, grip_z, angle):

            target_x_axis_1 = [1, 0, 0]
            target_x_axis_2 = [0, 0, -1]
            self.get_logger().info("초기 위치로 이동")
            self.move_to_initial_position()
            if self.angle2 is not None and self.distance2 is not None:
                self.rotate_joint('base', angle)
                print('111111111111111111111111111111111111111111111111')
                # self.move_to_xyz_rpy(grip_x - 0.05, grip_y, grip_z + 0.10, mode='direct', velocity=50)
                
                self.rotate_joint('joint3',-30)
                # self.move_to_xyz_rpy(grip_x-0.05, grip_y, grip_z + 0.10,
                #                     target_x_axis_2, mode='direct', velocity=50, orientation='X')
            
                self.move_to_xyz_rpy(grip_x, grip_y, grip_z + 0.04,
                                    target_x_axis_2, mode='linear',  velocity=50, orientation='X')                

                self.move_to_xyz_rpy(grip_x, grip_y, grip_z-0.01,
                                    target_x_axis_2, mode='linear', velocity=20, orientation='X')

                self.get_logger().info("그리퍼 열기")
                self.end_effector.move(330, velocity=10)
                self.end_effector.wait_until_goal_reached(330)
                # self.move_to_xyz_rpy(grip_x-0.06, grip_y, grip_z-0.01,
                #                     target_x_axis_2, mode='linear', velocity=50, orientation='X')
                self.rotate_joint('joint1', +5, velocity=20)
                self.rotate_joint('joint3', +30, velocity=20)
                time.sleep(1.0)
                self.rotate_joint('joint1', -20, velocity=20)

                time.sleep(1.0)
            else:
                self.get_logger().info("픽업대로 이동")
                self.move_to_xyz_rpy(grip_x - 0.05, grip_y, grip_z + 0.2,
                                target_x_axis_1, mode='direct', velocity=50, orientation='X')
                
                self.move_to_xyz_rpy(grip_x, grip_y, grip_z,
                                    target_x_axis_2, mode='direct', velocity=50, orientation='X')

                self.get_logger().info("그리퍼 열기")
                self.end_effector.move(270, velocity=10)
                self.end_effector.wait_until_goal_reached(270)
                self.move_to_xyz_rpy(grip_x-0.1, grip_y, grip_z,
                                    target_x_axis_2, mode='linear', velocity=50, orientation='X')
                time.sleep(1.0)

    def block_stack(self, pick_x, pick_y, pick_z_grasp, place_x, place_y, place_z):

        target_x_axis_1 = [1, 0, 0]
        target_x_axis_2 = [0, 0, -1]
        self.get_logger().info("초기 위치로 이동")
        self.move_to_initial_position()

        if self.angle1 is not None and self.distance1 is not None and \
           self.angle2 is not None and self.distance2 is not None:

            self.rotate_joint('base', self.angle1)

            self.get_logger().info("픽업대로 이동")
            self.move_to_xyz_rpy(pick_x, pick_y, pick_z_grasp + 0.06, mode='linear', velocity=30)
            self.move_to_xyz_rpy(pick_x, pick_y, pick_z_grasp + 0.02, target_x_axis_2, mode='linear', velocity=20, orientation='X')
            self.move_to_xyz_rpy(pick_x, pick_y, pick_z_grasp,  target_x_axis_2, mode='linear', velocity=30, orientation='X')

            self.get_logger().info("그리퍼 닫기")
            self.end_effector.move(330, velocity=10)
            self.end_effector.wait_until_goal_reached(330)
            time.sleep(1.0)
            self.move_to_initial_position_again()
          

            self.get_logger().info("놓을 위치로 이동")
            #self.move_to_xyz_rpy(place_x, place_y, place_z + 0.08, mode='direct', velocity=10)
            # self.move_to_xyz_rpy(place_x, place_y, place_z + 0.05, mode='linear', velocity=20)
            self.move_to_xyz_rpy(place_x+0.02, place_y, place_z + 0.08, mode='direct', velocity=10)
            #self.move_to_xyz_rpy(place_x, place_y, place_z + 0.05, mode='direct', velocity=30)
            #self.move_to_xyz_rpy(place_x, place_y, place_z + 0.04, target_x_axis_2, mode='place', velocity=30, orientation='X')
            # theta = np.deg2rad(-45)  # x축이 아래로 꺾이게 하려면 y축 기준 -45도 회전
            # # target_orientation_45 = self.rotation_matrix_y(theta)
            # self.move_to_xyz_rpy(place_x, place_y, place_z+0.05, mode='direct', velocity=10)
            # self.move_to_xyz_rpy(place_x, place_y, place_z + 0.03, mode='linear', velocity=20)
            self.move_to_xyz_rpy(place_x, place_y, place_z, target_x_axis_2, mode='direct', velocity=10, orientation='X')
            
            # ty=10,  orientation='X')
            time.sleep(1.0)
            self.get_logger().info("그리퍼 열기")
            self.end_effector.move(270, velocity=5)
            self.end_effector.wait_until_goal_reached(270)
            #self.move_to_xyz_rpy(place_x, place_y, place_z + 0.02, target_x_axis_2, mode='linear', velocity=10, orientation='X')
            #time.sleep(1.0)
            self.rotate_joint('joint1', -20)
            self.rotate_joint('joint3', 40)
            self.get_logger().info("초기 위치로 복귀")
            self.move_to_initial_position(velocity=30)
            time.sleep(1.0)
        else:
            self.get_logger().error("Angle or distance is not set. Cannot perform block stacking.")
    
    def block_in(self, pick_x, pick_y, pick_z_grasp, place_x, place_y, place_z):
        target_x_axis_1 = [1, 0, 0]
        target_x_axis_2 = [0, 0, -1]
        self.get_logger().info("초기 위치로 이동")
        self.move_to_initial_position()

        if self.angle1 is not None and self.distance1 is not None and \
           self.angle2 is not None and self.distance2 is not None:

            self.rotate_joint('base', self.angle1)

            self.get_logger().info("픽업대로 이동")
            self.move_to_xyz_rpy(pick_x, pick_y, pick_z_grasp + 0.06, mode='linear', velocity=30)
            self.move_to_xyz_rpy(pick_x, pick_y, pick_z_grasp+ 0.02, target_x_axis_2, mode='linear', velocity=20, orientation='X')
            self.move_to_xyz_rpy(pick_x, pick_y, pick_z_grasp,  target_x_axis_2, mode='linear', velocity=30, orientation='X')

            self.get_logger().info("그리퍼 닫기")
            self.end_effector.move(330, velocity=10)
            self.end_effector.wait_until_goal_reached(330)
            time.sleep(1.0)
            self.move_to_initial_position_again()
            #self.move_to_xyz_rpy(pick_x, pick_y, pick_z_grasp + 0.2, mode='direct', velocity=60)

            # self.get_logger().info("Waypoint 경유")
            # self.move_to_xyz_rpy(pick_x, pick_y, 0.7, mode = 'under_first', velocity = 30)
            #self.move_to_xyz_rpy(pick_x, pick_y, 0.5, mode='direct', velocity=30)                                                        
            #self.move_to_xyz_rpy(waypoint[0], waypoint[1], waypoint[2] + 0.1, mode='direct', velocity=60)
            # self.move_to_xyz_rpy(pick_x, pick_y, 0.5, mode='direct', velocity=30)         # >>>>> waypoint 경유하는 부분 추가함 <<<<<

            self.get_logger().info("놓을 위치로 이동")
            self.rotate_joint('base', self.angle2)
            self.move_to_xyz_rpy(place_x, place_y, place_z + 0.1, mode='direct', velocity=30)
            #self.move_to_xyz_rpy(place_x, place_y, place_z + 0.05, mode='direct', velocity=30)
            #self.move_to_xyz_rpy(place_x, place_y, place_z + 0.04, target_x_axis_2, mode='place', velocity=30, orientation='X')
            # theta = np.deg2rad(-45)  # x축이 아래로 꺾이게 하려면 y축 기준 -45도 회전
            # # target_orientation_45 = self.rotation_matrix_y(theta)
            # self.move_to_xyz_rpy(place_x, place_y, place_z+0.05, mode='direct', velocity=10)
            self.move_to_xyz_rpy(place_x, place_y, place_z + 0.05, mode='linear', velocity=20)
            self.move_to_xyz_rpy(place_x, place_y, place_z, target_x_axis_2, mode='linear', velocity=20,  orientation='X')
            
            # self.move_to_xyz_rpy(place_x, place_y, place_z, target_x_axis_2, mode='direct',  veloci

            # ty=10,  orientation='X')
            time.sleep(1.0)
            self.get_logger().info("그리퍼 열기")
            self.end_effector.move(270, velocity=5)
            self.end_effector.wait_until_goal_reached(270)
            #self.move_to_xyz_rpy(place_x, place_y, place_z + 0.02, target_x_axis_2, mode='linear', velocity=10, orientation='X')
            #time.sleep(1.0)
            self.rotate_joint('joint1', -20)
            self.rotate_joint('joint3', 40)
            self.get_logger().info("초기 위치로 복귀")
            self.move_to_initial_position(velocity=30)
            time.sleep(1.0)
        else:
            self.get_logger().error("Angle or distance is not set. Cannot perform block stacking.")

    def process(self):
        time.sleep(1.0)
        # 1) 픽업 위치 계산
        pick_x = self.distance1 * math.cos(math.radians(self.angle1))
        pick_y = -self.distance1 * math.sin(math.radians(self.angle1))
        pick_z = 0.090
        
        # 2) 상자(혹은 놓기 기준) 중심 계산
        place_origin_x = (self.distance2 + 0.02) * math.cos(math.radians(self.angle2))
        place_origin_y = -(self.distance2 + 0.02)* math.sin(math.radians(self.angle2))
        place_z = 0.07

        #case1
        ###place : 15deg
        # block_stack_plan = [
        #     (0.0300, 0.0, 0.070, 0.01,  -0.08, -0.01),
        #     (0.0500, 0.0, 0.050, -0.03,  -0.03,  0.00),
        #     (0.0250, 0.0, 0.050, -0.08,  0.04,  0.00),
        #     (0.0600, 0.0, 0.030,  0.00,  0.00,  0.00), 
        #     (0.0355, 0.0, 0.030,  0.025, 0.00,  0.00), 
        #     (0.0118, 0.0, 0.030,  0.050, 0.00,  0.00), 
        #     (0.0450, 0.0, 0.000,  0.0125,0.00,  0.034),
        #     (0.0272, 0.0, 0.000,  0.0375,0.00,  0.034),
        #     (0.0000, 0.0, -0.01,  0.025, 0.00,  0.060)
        # ]


#####0.160(distance:1)
        # angle1: -15deg, angle1: -30deg 
        # block_stack_plan = [
        #     (0.0300, 0.0, 0.070, 0.01,  -0.08, -0.01),
        #     (0.0500, 0.0, 0.050, -0.03,  -0.03,  0.00),
        #     (0.0250, 0.0, 0.050, -0.05,  0.04,  0.00),
        #     (0.0600, 0.0, 0.030,  0.00,  0.00,  0.00), 
        #     (0.0355, 0.0, 0.030,  0.025, 0.00,  0.00), 
        #     (0.0118, 0.0, 0.030,  0.050, 0.00,  0.00), 
        #     (0.0450, 0.0, 0.000,  0.0125,0.00,  0.034),
        #     (0.0272, 0.0, 0.000,  0.0375,0.00,  0.034),
        #     (0.0000, 0.0, -0.01,  0.025, 0.00,  0.060)
        # ]



        # block_stack_plan = [
        #     (0.0300, 0.0, 0.070, 0.0,  -0.05, -0.01),
        #     (0.0500, 0.0, 0.050, 0.0,  -0.02,  0.00),
        #     (0.0250, 0.0, 0.050, 0.0,  0.02, -0.01),
        #     (0.055, 0.0, 0.030,  0.00,  0.00,  0.00), #  little back
        #     (0.033, 0.0, 0.030,  0.025, 0.00,  0.00), # little front
        #     (0.011, 0.0, 0.030,  0.050, 0.00,  0.00), # good
        #     (0.044, 0.0, 0.000,  0.0125,0.00,  0.031),# g
        #     (0.022, 0.0, 0.000,  0.0375,0.00,  0.031),
        #     (0.0000, 0.0, -0.01,  0.025, 0.00,  0.060)
        # ]

        # # 45, 60deg execept
        # block_stack_plan = [
        #     (0.0290, 0.0, 0.072, 0.01,  -0.08, -0.01),
        #     (0.0450, 0.0, 0.050, -0.03,  -0.02,  0.02),
        #     (0.0220, 0.0, 0.050, -0.10,  0.04,  0.00),
        #     (0.0620, 0.0, 0.025,  0.000,  0.00,  0.00), 
        #     (0.0375, 0.0, 0.025,  0.025, 0.00,  0.00), 
        #     (0.0125, 0.0, 0.025,  0.050, 0.00,  0.00), 
        #     (0.050, 0.0, 0.00,  0.0125,0.00,  0.034),# good
        #     (0.028, 0.0, 0.00,  0.0375,0.00,  0.034), #good
        #     (0.0000, 0.0, 0.00,  0.025, 0.00,  0.068)
        # ]

 
        # # 45(+0.5:distance1),
        # block_stack_plan = [
        #     (0.0290, 0.0, 0.072, -0.05,  -0.06, -0.01),
        #     (0.0450, 0.0, 0.050, -0.03,  -0.03,  0.00),
        #     (0.0220, 0.0, 0.050, -0.05,  0.04,  0.00),
        #     (0.0620, 0.0, 0.025,  0.000,  0.00,  0.00), #  little back
        #     (0.0370, 0.0, 0.025,  0.025, 0.00,  0.00), # little front
        #     (0.0125, 0.0, 0.025,  0.050, 0.00,  0.00), # good
        #     (0.048, 0.0, 0.00,  0.0125,0.00,  0.034),# good
        #     (0.025, 0.0, 0.00,  0.0375,0.00,  0.034), #good
        #     (0.0000, 0.0, 0.00,  0.020, 0.00,  0.063)
        # ]

        # 60(+0.1: distance1)
        block_stack_plan = [
            (0.0290, 0.0, 0.072, -0.05,  -0.07, -0.02),
            (0.0450, 0.0, 0.050, -0.03,  -0.03,  -0.02),
            (0.0220, 0.0, 0.050, 0.0,  0.01,  0.00),
            (0.0620, 0.0, 0.025,  0.000,  0.00,  0.00), #  little back
            (0.0370, 0.0, 0.025,  0.026, 0.00,  0.01), # little front
            (0.0125, 0.0, 0.025,  0.046, 0.00,  0.00), # good
            (0.048, 0.0, 0.00,  0.0125,0.00,  0.034),# good
            (0.028, 0.0, 0.00,  0.0372,0.00,  0.035), #good
            (0.0000, 0.0, 0.00,  0.020, 0.00,  0.065)
        ]

        pick_x = self.distance1 * math.cos(math.radians(self.angle1))
        pick_y = -self.distance1 * math.sin(math.radians(self.angle1))
        pick_z = 0.11

        open_grip_x = (self.distance2 - 0.040) * math.cos(math.radians(self.angle2))
        open_grip_y = -(self.distance2 - 0.040) * math.sin(math.radians(self.angle2))
        open_grip_z = 0.055
        
        # 45 30deg
        place_box_x = (self.distance2 - 0.01) * math.cos(math.radians(self.angle2-5))
        place_box_y = -(self.distance2 - 0.01) * math.sin(math.radians(self.angle2-5))
        place_box_z = 0.12

        #15deg 
        # place_box_x = (self.distance2 + 0.030) * math.cos(math.radians(self.angle2-5))
        # place_box_y = -(self.distance2 + 0.030) * math.sin(math.radians(self.angle2-5))
        # place_box_z = 0.12


        close_grip_x = (self.distance2 - 0.08) * math.cos(math.radians(self.angle2-5))
        close_grip_y = -(self.distance2 - 0.08) * math.sin(math.radians(self.angle2-5))
        close_grip_z = 0.080
        # 상자 열기
        
        # self.open_box(open_grip_x, open_grip_y, open_grip_z, self.angle2)
        # # # self.move_to_box_position()
        
        # # 1~3: 상자 안에 넣기
        # for i in range(3):
        #     dx_p, dy_p, dz_p, dx_pl, dy_pl, dz_pl = block_stack_plan[i]
        #     px, py = local_to_global_pick(pick_x, pick_y, dx_p, dy_p, self.angle1)
        #     pz = pick_z + dz_p
        #     plx, ply = local_to_global_box(place_box_x, place_box_y, dx_pl, dy_pl, self.angle2)
        #     plz = place_box_z + dz_pl
        #     self.block_in(px, py, pz, plx, ply, plz)

        # #상자 닫기
        # self.close_box(close_grip_x, close_grip_y, close_grip_z,self.angle2)

        #4~9: 블록 쌓기
        
        # 2) 상자(혹은 놓기 기준) 중심 계산
        ##factor
        place_origin_x = (0.13 + 0.02) * math.cos(math.radians(self.angle2))
        place_origin_y = -(0.13 + 0.02)* math.sin(math.radians(self.angle2))
        place_z = 0.07

        # modify -> angle2: -45
        # modify -> angle1: 30
        for i in range(3, 9):
            dx_p, dy_p, dz_p, dx_pl, dy_pl, dz_pl = block_stack_plan[i]
            px, py = local_to_global_pick(pick_x, pick_y, dx_p, dy_p,-60)
            pz = pick_z + dz_p
            plx, ply = local_to_global_place(place_origin_x, place_origin_y, dx_pl, dy_pl, 60)
            plz = 0.105 + dz_pl
            self.block_stack(px, py, pz, plx, ply, plz)

            
# # === main 함수 (기존과 동일) ===
# def main(args=None):
#     rclpy.init(args=args)
#     node = DynamixelControlNode(angle=-45, distance=0.180)
#     try:
#         node.process()
#         rclpy.spin(node)
#     except KeyboardInterrupt
#         node.get_logger().info('Keyboard Interrupt (SIGINT) detected, shutting down.')
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = DynamixelControlNode(angle1=-60, angle2=60, distance1=0.161, distance2=0.130)
    try:
        node.process()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt detected, shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


#14cm -60
#13cm 60