import rclpy
import numpy as np
from rclpy.node import Node
from dynamixel_sdk import *
import time
from scipy.interpolate import CubicSpline
from ikpy.chain import Chain
from ikpy.link import URDFLink
serial  
import time
from dynamixel_sdk import *

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

# DH_params = [
#     [0,        0.040, 0,       0],       
#     [0,        0.050, 0,  np.pi/2],        
#     [np.pi/2,  0,     0.125,   0],         
#     [0,        0,     0.13,    0],        
#     [0,   -0.033,     0,    np.pi/2],         
# ]

DH_params = [
    [0,        0.065, 0,       0],         # Joint 0
    [0,        0.052, 0,  np.pi/2],         # Joint 1
    [np.pi/2,  0,     0.123994,   0],         # Joint 2
    [0,        0,     0.126296,    0],         # Joint 3
    [np.pi/2,  0,     0.08,    np.pi/2],         # Joint 4 
]


num_joints = len(DH_params)


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

def create_ikpy_chain(dh_params):
    links = [
        URDFLink(
            name="base_link",
            origin_translation=[0, 0, 0],
            origin_orientation=[0, 0, 0],
            rotation=[0, 0, 0] 
        )
    ]

    for i, params in enumerate(dh_params):
        theta_offset, d, a, alpha = params

        if i == 0:  
            links.append(
                URDFLink(
                    name=f"joint_{i}",
                    origin_translation=[0, 0, d],
                    origin_orientation=[0, 0, 0],
                    rotation=[0, 0, 1],
                    bounds=(-np.radians(180), np.radians(180))
                )
            )
        elif i == 1: 
            links.append(
                URDFLink(
                    name=f"joint_{i}",
                    origin_translation=[0, 0, d],
                    origin_orientation=[alpha, 0, 0],
                    rotation=[0, 0, 1],
                    bounds=(-np.radians(180), np.radians(180))
                )
            )
        elif i == 2:
            links.append(
                URDFLink(
                    name=f"joint_{i}",
                    origin_translation=[a, 0, 0],
                    origin_orientation=[0, 0, 0],
                    rotation=[0, 0, 1],
                    bounds=(np.radians(-180), np.radians(180)
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
                    bounds=(-np.radians(180), np.radians(180))
                )
            )
        elif i == 4:  # Joint 4: 프리즘 조인트 (x축 방향 슬라이더)
            links.append(
                URDFLink(
                    name=f"joint_{i}",
                    origin_translation=[a, 0, 0],
                    
                    origin_orientation=[alpha, 0, 0],
                    rotation=[0, 0, 0], # 회전 없음
                    bounds=(-np.radians(0.1), np.radians(0.1)))
                )
        else:
            raise ValueError("Unexpected joint index")

    return Chain(name="arm", links=links)

ikpy_chain = create_ikpy_chain(DH_params)

initial_joints = [0, 0, np.pi/2, 0, 0,  np.deg2rad(270)]
fk_results = ikpy_chain.forward_kinematics(initial_joints)

def inverse_kinematics_ikpy(target_position, target_orientation=None, initial_guess=None, orientation_mode=None):
    ik_results = ikpy_chain.inverse_kinematics(
        target_position=target_position,
        target_orientation=target_orientation,  
        initial_position=initial_guess,
        orientation_mode=orientation_mode,
        regularization_parameter=0.01

    )
    return ik_results

class DynamixelMotor:
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

        self.goal_angle_deg = None
        self.goal_position = None
        self.timeout_sec = 0.0
        self.start_time = 0.0
        self.waiting_active = False
        self.goal_reached = False
        self.tolerance_deg = 5.0

    def setting(self):
        if not self.port_handler.setBaudRate(self.baudrate):
            self.logger.error(f'{self.name} Baudrate 설정 실패')
            raise RuntimeError(f'{self.name} Baudrate 설정 실패')
        if self.is_ax:
            self.packet_handler.write1ByteTxRx(self.port_handler, self.dxl_id, AX_ADDR_TORQUE_ENABLE, AX_TORQUE_ENABLE)
        elif self.is_xc:
            self.packet_handler.write1ByteTxRx(self.port_handler, self.dxl_id, XC_ADDR_TORQUE_ENABLE, XC_TORQUE_ENABLE)
        else:
            self.packet_handler.write1ByteTxRx(self.port_handler, self.dxl_id, ADDR_TORQUE_ENABLE, XM_TORQUE_ENABLE)

    def move(self, angle_deg, velocity=400):
        self.setting()
        protocol = AX_PROTOCOL_VERSION if self.is_ax else XC_PROTOCOL_VERSION if self.is_xc else XM_PROTOCOL_VERSION
        position = angle_to_position(angle_deg, protocol)
        if self.is_ax:
            self.packet_handler.write2ByteTxRx(self.port_handler, self.dxl_id, AX_ADDR_MOVING_SPEED, velocity)
            self.packet_handler.write2ByteTxRx(self.port_handler, self.dxl_id, AX_ADDR_GOAL_POSITION, position)
        elif self.is_xc:
            self.packet_handler.write4ByteTxRx(self.port_handler, self.dxl_id, ADDR_PROFILE_VELOCITY, velocity)
            self.packet_handler.write4ByteTxRx(self.port_handler, self.dxl_id, XC_ADDR_GOAL_POSITION, position)
        else:
            self.packet_handler.write4ByteTxRx(self.port_handler, self.dxl_id, ADDR_PROFILE_VELOCITY, velocity)
            self.packet_handler.write4ByteTxRx(self.port_handler, self.dxl_id, ADDR_GOAL_POSITION, position)

        self.goal_angle_deg = angle_deg
        self.goal_position = position
        self.start_time = time.time()
        self.waiting_active = True
        self.goal_reached = False

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

    def update(self):
        if not self.waiting_active:
            return

        present_position_deg = self.get_present_position()
        if present_position_deg is not None:
            protocol = AX_PROTOCOL_VERSION if self.is_ax else XC_PROTOCOL_VERSION if self.is_xc else XM_PROTOCOL_VERSION
            present_position = angle_to_position(present_position_deg, protocol)
            tolerance = angle_to_position(self.tolerance_deg, protocol)
            if abs(present_position - self.goal_position) <= tolerance:
                self.goal_reached = True
                self.waiting_active = False
            elif time.time() - self.start_time > self.timeout_sec:
                self.goal_reached = False
                self.waiting_active = False
        else:
            if time.time() - self.start_time > self.timeout_sec:
                self.goal_reached = False
                self.waiting_active = False



class DynamixelControlNode(Node):
    def __init__(self):
        super().__init__('dynamixel_control_node')
        self.port_handler = PortHandler(DEVICENAME)
        if not self.port_handler.openPort():
            return
        self.base = DynamixelMotor(self.port_handler, 1, "base", 1000000,  XM_PROTOCOL_VERSION, self.get_logger())
        self.joint1 = DynamixelMotor(self.port_handler, 2, "joint1", 1000000,  XM_PROTOCOL_VERSION, self.get_logger())
        self.joint2 = DynamixelMotor(self.port_handler, 3, "joint2", 1000000,  AX_PROTOCOL_VERSION, self.get_logger(),is_ax=True)
        self.joint3 = DynamixelMotor(self.port_handler, 4,  "joint3", 1000000,  AX_PROTOCOL_VERSION, self.get_logger(), is_ax=True)
        self.previous_joint_angles_rad = np.array([0, 0, np.pi/2, 0, 0,  0.1]) 
        self.timer = self.create_timer(0.0001, self.timer_callback)

    def timer_callback(self):
        return

    def read_current_angle(self):
            current_angles = []
            current_angles_final = [0,0,0,0,0,0]
            for motor in [self.base, self.joint1, self.joint2, self.joint3]:
                position_deg = motor.get_present_position()
                if position_deg is None:
                    return None
                current_angles.append(position_deg)
            print("current_angles", current_angles)
            current_angles_final[0] = 0
            current_angles_final[1] = 210 - current_angles[0] # Joint 0 -> Base
            current_angles_final[2] = 179 + 90 - current_angles[1] # Joint 1 -> Joint1
            current_angles_final[3] = -150 + current_angles[2] # Joint 2 -> Joint2
            current_angles_final[4] = -150 + current_angles[3] # Joint 3 -> Joint3
            current_angles_final[5] = 0.1 # Joint 4 -> Joint4 (프리즘 조인트)

            self.get_logger().info(f"현재 각도: {current_angles}")
            return current_angles_final


    def move_to_initial_position(self, velocity=50):
        self.get_logger().info("--- Moving to initial position ---")
        self.base.move(212, velocity)
        self.joint1.move(209, velocity)
        self.joint2.move(60,velocity)  
        self.joint3.move(90, velocity)
        # self.base.wait_until_goal_reached(212)
        # self.joint1.wait_until_goal_reached(179)
        # self.joint2.wait_until_goal_reached(150)
        # self.joint3.wait_until_goal_reached(150)

    def cubic_spline_trajectory(self, joint_angles_initial, joint_angles_final, time_points):
            spline_trajectory = []
            for initial, final in zip(joint_angles_initial, joint_angles_final):
                times = [0, max(time_points)]
                angles = [initial, final]
                spline = CubicSpline(times, angles, bc_type='clamped')
                spline_trajectory.append(spline(time_points))
            return np.array(spline_trajectory)

    def move_to_initial_position_again(self):
        self.get_logger().info("--- Moving to initial position again ---")
        self.base.move(212)
        self.joint1.move(179)
        self.joint2.move(150)
        self.joint3.move(150)

        # 중심에서 가장 멀고 가까운 점일 때를 np,linspace로 나누기, 40개 점만 move함수에서 joint3 -15도 빼기
        # move to xyz rpy에서 orientation 바꿔서 테스트.

    def draw_circle(self, center, radius, plane='xy', z_fixed=None, num_points=300, velocity=200):
        """
            distance-param  |radius(real) |  calibration param  |  real-distance-to-center  |  z_fixed   |   radius-parameter 
             15             |5            |0.003                |16.8                       |0.067       |0.048
             15             |6            |0.0042               |16.8                       |0.065       |0.058
             15             |7            |0.0046               |17.1                       |0.065       |0.068
             15             |8            |                     |                           |            |
             16             |5            |0.002                |17.9                       |0.07        |0.048
             16             |6            |0.0042               |18.1                       |0.065       |0.058
             16             |7            |0.0046               |18.2                       |0.0065      |0.067
             16             |8            |                     |                           |            |
             17             |5            |0.002                |18.7                       |0.07        |0.048
             17             |6            |0.0035               |19.2                       |0.065       |0.058
             17             |7            |0.0042               |19.0                       |0.065       |0.067
             17             |8            |                     |                           |            |
             18             |5            |0.002                |19.8                       |0.07        |0.048
             18             |6            |0.0032               |20.1                       |0.065       |0.057
             18             |7            |0.0042               |19.9                       |0.067       |0.067
             18             |8            |X                    |X                          |X           |X
        """
        cx, cy, cz = center
        z_base = z_fixed
        total_points = num_points

        # 각도 배열 (2바퀴: 2π ~ 4π)
        thetas = np.linspace(np.pi + np.pi / 6, 3 * np.pi + np.pi / 6, total_points)

        """ z plane calibration factor
        """
        # z 보정 함수
        def z_wave_offset(theta):
            return -0.003 * np.sin(2 * theta) 
        #2일 때 -0.000
        #3일 때 -0.001                                                                                                     #@@@@@@@ 바꿔야 함 @@@@@@@
        #4일 때 -0.003
        #5일 때 -0.000
        #6일 때 -0.000
        #7일 때 -0.000
        
        """ z plane offset
        """
        # z_offsets = [0.000, 0.000 , 0.000 , 0.000, -0.003, -0.003, -0.002, 0.000] #반지름 0.02일 때
        # z_offsets = [0.000, 0.000 , 0.000 , 0.000, -0.003, -0.003, -0.003, 0.000] #반지름 0.03일 때
        z_offsets = [-0.002, 0.000 , 0.000 , -0.002, -0.002, -0.002, -0.004, -0.002] #반지름 0.04일 때                        #@@@@@@@ 바꿔야 함 @@@@@@@
        # z_offsets = [0.000, 0.002 , 0.000 , 0.000, -0.002, -0.002, 0.000, 0.002] #반지름 0.05일 때
        # z_offsets = [0.000, 0.002 , 0.000 , 0.000, -0.002, -0.002, 0.000, 0.002] #반지름 0.06일 때
        # z_offsets = [-0.001, -0.001, -0.004, -0.004, -0.008, 0.002, 0.001, -0.001] #반지름 0.07일 때

        """ radius calibration factor
        """
        # cos 기반 반지름 보정 함수
        def radius_offset(theta):
            return -0.003 * np.cos(2 * theta)
        #2일 때 -0.001
        #3일 때 -0.002
        #4일 때 -0.003
        #5일 때 -0.003
        #6일 때 -0.003
        #7일 때 -0.003

        """ radius offset
        """
        # radius_offsets = [0.001, 0.001, 0.00, 0.00, 0.00, 0.00, -0.002, 0.00] #반지름 0.02일 때 -> 0.022
        # radius_offsets = [0.001, 0.001, 0.00, 0.00, 0.00, 0.001, 0.001, 0.00] #반지름 0.03일 때 -> 0.032
        radius_offsets = [0.001, 0.002, 0.00, 0.00, 0.00, 0.00, 0.00, -0.001] #반지름 0.04일 때 -> 0.042
        # radius_offsets = [0.001, 0.001, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00] #반지름 0.05일 때 -> 0.052                     #@@@@@@@ 바꿔야 함 @@@@@@@
        # radius_offsets = [0.001, 0.001, 0.00, 0.00, 0.00, 0.00, -0.001, -0.001] #반지름 0.06일 때 -> 0.062
        # radius_offsets = [-0.001, -0.001, -0.004, -0.004, -0.002, 0.002, 0.001, -0.001] #반지름 0.07일 때 -> 0.072

        # 첫 점 기준 위치 계산
        theta0 = thetas[0]
        segment_index = int((theta0 % (2 * np.pi)) / (np.pi / 4)) % 8
        dynamic_radius = radius + radius_offset(theta0) + radius_offsets[segment_index] 
        
        dynamic_z = z_base

        if plane == 'xy':
            initial_x = cx + dynamic_radius * np.cos(theta0)
            initial_y = cy + dynamic_radius * np.sin(theta0)
            initial_z = dynamic_z
            pre_z = initial_z + 0.03
            pre_x, pre_y = initial_x + 0.03, initial_y
        elif plane == 'yz':
            initial_x = cx
            initial_y = cy + dynamic_radius * np.cos(theta0)
            initial_z = cz + dynamic_radius * np.sin(theta0)
            pre_z = initial_z + 0.03
            pre_x, pre_y = initial_x, initial_y
        elif plane == 'xz':
            initial_x = cx + dynamic_radius * np.cos(theta0)
            initial_y = cy
            initial_z = cz + dynamic_radius * np.sin(theta0)
            pre_z = initial_z + 0.03
            pre_x, pre_y = initial_x, initial_y
        else:
            print(f"the plane '{plane}' is not correct")
            return

        self.move_to_initial_position()

        time.sleep(1.0)

        initial_flag = True
        start_flag = True

        for theta in thetas:
            segment_index = int((theta % (2 * np.pi)) / (np.pi / 4)) % 8
            print(f"{segment_index + 1}" * 4)
            dynamic_radius = radius + radius_offset(theta) + radius_offsets[segment_index]
            dynamic_z = z_base + z_wave_offset(theta) + z_offsets[segment_index]
            print(f" ------- theta:{theta} ------- ")
            if plane == 'xy':
                if initial_flag == True:
                    x = cx + dynamic_radius * np.cos(theta)
                    y = cy + dynamic_radius * np.sin(theta)
                    z = dynamic_z
                    self.move_to_xyz_rpy(x, y, z+0.02, target_orientation=[0, 0, -1],mode='direct',velocity=80,orientation="X") 
                    initial_flag = False
                elif theta > 4: 
                        """ 
                            radius 2일 때 6
                            radius 3일 때 5
                            radius 4일 때 4
                            radius 5일 때 4
                            radius 6일 때 4
                            radius 7일 때 4
                        """                                                                                       # @@@@@@ theta 범위 수정해야 함 @@@@@@
                        x = cx + dynamic_radius * np.cos(theta)
                        y = cy + dynamic_radius * np.sin(theta)
                        z = dynamic_z
                        if start_flag == True:
                            self.move_to_xyz_rpy(x, y, z+0.01, target_orientation=[0, 0, -1], mode='direct', velocity=80, orientation="X")  
                            self.move_to_xyz_rpy(x, y, z+0.005, target_orientation=[0, 0, -1], mode='linear', velocity=80, orientation="X")     
                            start_flag = False
                        else:                                                 
                            self.move_to_xyz_rpy(
                                x, y, z, target_orientation=[0, 0, -1],
                                mode='direct', 
                                velocity=80, orientation="X"
                            )
                else:
                        x = cx + dynamic_radius * np.cos(theta)
                        y = cy + dynamic_radius * np.sin(theta)
                        z = dynamic_z
                        self.move_to_xyz_rpy(
                            x, y, z + 0.015,target_orientation=[0, 0, -1],
                            mode='direct',
                            velocity=80, orientation="X"
                            )
                    
        for theta in thetas:
            segment_index = int((theta % (2 * np.pi)) / (np.pi / 4)) % 8
            print(f"{segment_index + 1}" * 4)
            dynamic_radius = radius + radius_offset(theta) + radius_offsets[segment_index]
            dynamic_z = z_base + z_wave_offset(theta) + z_offsets[segment_index]
            print(f"theta:{theta}")

            if plane == 'xy':
                x = cx + dynamic_radius * np.cos(theta)
                y = cy + dynamic_radius * np.sin(theta)
                z = dynamic_z

                if start_flag == True:
                    self.move_to_xyz_rpy(x, y, z+0.01, target_orientation=[0, 0, -1], mode='direct', velocity=80, orientation="X")
                    self.move_to_xyz_rpy(x, y, z+0.005, target_orientation=[0, 0, -1], mode='linear', velocity=80, orientation="X")
                    start_flag = False
                else:
                    self.move_to_xyz_rpy(
                        x, y, z, target_orientation=[0, 0, -1],
                        mode='direct',
                        velocity=80, orientation="X"                                                                       # @@@@@@ 여기도 수정해야 함 @@@@@@
                    )
                
                if theta >= 4.2:  
                    return
        
                """ 
                    radius 2일 때 6
                    radius 3일 때 5
                    radius 4일 때 42
                    radius 5일 때 4
                    radius 6일 때 4
                    radius 7일 때 4
                """      

    def draw_circle2(self, center, radius, plane='xy', z_fixed=None, num_points=300, velocity=200):
        """
            distance-param  |radius(real) |  calibration param  |  real-distance-to-center  |  z_fixed   |   radius-parameter 
             15             |5            |0.003                |16.8                       |0.067       |0.048
             15             |6            |0.0042               |16.8                       |0.065       |0.058
             15             |7            |0.0046               |17.1                       |0.065       |0.068
             15             |8            |                     |                           |            |
             16             |5            |0.002                |17.9                       |0.07        |0.048
             16             |6            |0.0042               |18.1                       |0.065       |0.058
             16             |7            |0.0046               |18.2                       |0.0065      |0.067
             16             |8            |                     |                           |            |
             17             |5            |0.002                |18.7                       |0.07        |0.048
             17             |6            |0.0035               |19.2                       |0.065       |0.058
             17             |7            |0.0042               |19.0                       |0.065       |0.067
             17             |8            |                     |                           |            |
             18             |5            |0.002                |19.8                       |0.07        |0.048
             18             |6            |0.0032               |20.1                       |0.065       |0.057
             18             |7            |0.0042               |19.9                       |0.067       |0.067
             18             |8            |X                    |X                          |X           |X
        """
        cx, cy, cz = center
        z_base = z_fixed
        total_points = num_points

        # 각도 배열 (2바퀴: 2π ~ 4π)
        thetas = np.linspace(np.pi + np.pi / 6, 3 * np.pi + np.pi / 6, total_points)

        """ z plane calibration factor
        """
        # z 보정 함수
        def z_wave_offset(theta):
            return -0.000 * np.sin(2 * theta) 
        #2일 때 -0.000
        #3일 때 -0.001                                                                                                     #@@@@@@@ 바꿔야 함 @@@@@@@
        #4일 때 -0.003
        #5일 때 -0.000
        #6일 때 -0.000
        #7일 때 -0.000
        
        """ z plane offset
        """
        z_offsets = [0.000, 0.000 , 0.000 , 0.000, -0.003, -0.003, -0.002, 0.000] #반지름 0.02일 때
        # z_offsets = [0.000, 0.000 , 0.000 , 0.000, -0.003, -0.003, -0.003, 0.000] #반지름 0.03일 때
        # z_offsets = [-0.002, 0.000 , 0.000 , -0.002, -0.002, -0.002, -0.004, -0.002] #반지름 0.04일 때                        #@@@@@@@ 바꿔야 함 @@@@@@@
        # z_offsets = [0.000, 0.002 , 0.000 , 0.000, -0.002, -0.002, 0.000, 0.002] #반지름 0.05일 때
        # z_offsets = [0.000, 0.002 , 0.000 , 0.000, -0.002, -0.002, 0.000, 0.002] #반지름 0.06일 때
        # z_offsets = [-0.001, -0.001, -0.004, -0.004, -0.008, 0.002, 0.001, -0.001] #반지름 0.07일 때

        """ radius calibration factor
        """
        # cos 기반 반지름 보정 함수
        def radius_offset(theta):
            return -0.001 * np.cos(2 * theta)
        #2일 때 -0.001
        #3일 때 -0.002
        #4일 때 -0.003
        #5일 때 -0.003
        #6일 때 -0.003
        #7일 때 -0.003

        """ radius offset
        """
        radius_offsets = [0.001, 0.001, 0.00, 0.00, 0.00, 0.00, -0.002, 0.00] #반지름 0.02일 때 -> 0.022
        # radius_offsets = [0.001, 0.001, 0.00, 0.00, 0.00, 0.001, 0.001, 0.00] #반지름 0.03일 때 -> 0.032
        # radius_offsets = [0.002, 0.002, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00] #반지름 0.04일 때 -> 0.042
        # radius_offsets = [0.001, 0.001, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00] #반지름 0.05일 때 -> 0.052                     #@@@@@@@ 바꿔야 함 @@@@@@@
        # radius_offsets = [0.001, 0.001, 0.00, 0.00, 0.00, 0.00, -0.001, -0.001] #반지름 0.06일 때 -> 0.062
        # radius_offsets = [-0.001, -0.001, -0.004, -0.004, -0.002, 0.002, 0.001, -0.001] #반지름 0.07일 때 -> 0.072

        # 첫 점 기준 위치 계산
        theta0 = thetas[0]
        segment_index = int((theta0 % (2 * np.pi)) / (np.pi / 4)) % 8
        dynamic_radius = radius + radius_offset(theta0) + radius_offsets[segment_index] 
        
        dynamic_z = z_base

        if plane == 'xy':
            initial_x = cx + dynamic_radius * np.cos(theta0)
            initial_y = cy + dynamic_radius * np.sin(theta0)
            initial_z = dynamic_z
            pre_z = initial_z + 0.03
            pre_x, pre_y = initial_x + 0.03, initial_y
        elif plane == 'yz':
            initial_x = cx
            initial_y = cy + dynamic_radius * np.cos(theta0)
            initial_z = cz + dynamic_radius * np.sin(theta0)
            pre_z = initial_z + 0.03
            pre_x, pre_y = initial_x, initial_y
        elif plane == 'xz':
            initial_x = cx + dynamic_radius * np.cos(theta0)
            initial_y = cy
            initial_z = cz + dynamic_radius * np.sin(theta0)
            pre_z = initial_z + 0.03
            pre_x, pre_y = initial_x, initial_y
        else:
            print(f"the plane '{plane}' is not correct")
            return

        self.move_to_initial_position()

        time.sleep(1.0)

        initial_flag = True
        start_flag = True

        for theta in thetas:
            segment_index = int((theta % (2 * np.pi)) / (np.pi / 4)) % 8
            print(f"{segment_index + 1}" * 4)
            dynamic_radius = radius + radius_offset(theta) + radius_offsets[segment_index]
            dynamic_z = z_base + z_wave_offset(theta) + z_offsets[segment_index]
            print(f" ------- theta:{theta} ------- ")
            if plane == 'xy':
                if initial_flag == True:
                    x = cx + dynamic_radius * np.cos(theta)
                    y = cy + dynamic_radius * np.sin(theta)
                    z = dynamic_z
                    self.move_to_xyz_rpy(x, y, z+0.02, target_orientation=[0, 0, -1],mode='direct',velocity=80,orientation="X") 
                    initial_flag = False
                elif theta > 6: 
                        """ 
                            radius 2일 때 6
                            radius 3일 때 5
                            radius 4일 때 4
                            radius 5일 때 4
                            radius 6일 때 4
                            radius 7일 때 4
                        """                                                                                       # @@@@@@ theta 범위 수정해야 함 @@@@@@
                        x = cx + dynamic_radius * np.cos(theta)
                        y = cy + dynamic_radius * np.sin(theta)
                        z = dynamic_z
                        if start_flag == True:
                            self.move_to_xyz_rpy(x, y, z+0.01, target_orientation=[0, 0, -1], mode='direct', velocity=80, orientation="X")  
                            self.move_to_xyz_rpy(x, y, z+0.005, target_orientation=[0, 0, -1], mode='linear', velocity=80, orientation="X")     
                            start_flag = False
                        else:                                                 
                            self.move_to_xyz_rpy(
                                x, y, z, target_orientation=[0, 0, -1],
                                mode='direct', 
                                velocity=80, orientation="X"
                            )
                else:
                        x = cx + dynamic_radius * np.cos(theta)
                        y = cy + dynamic_radius * np.sin(theta)
                        z = dynamic_z
                        self.move_to_xyz_rpy(
                            x, y, z + 0.015,target_orientation=[0, 0, -1],
                            mode='direct',
                            velocity=80, orientation="X"
                            )
                    
        for theta in thetas:
            segment_index = int((theta % (2 * np.pi)) / (np.pi / 4)) % 8
            print(f"{segment_index + 1}" * 4)
            dynamic_radius = radius + radius_offset(theta) + radius_offsets[segment_index]
            dynamic_z = z_base + z_wave_offset(theta) + z_offsets[segment_index]
            print(f"theta:{theta}")

            if plane == 'xy':
                x = cx + dynamic_radius * np.cos(theta)
                y = cy + dynamic_radius * np.sin(theta)
                z = dynamic_z

                if start_flag == True:
                    self.move_to_xyz_rpy(x, y, z+0.01, target_orientation=[0, 0, -1], mode='direct', velocity=80, orientation="X")
                    self.move_to_xyz_rpy(x, y, z+0.005, target_orientation=[0, 0, -1], mode='linear', velocity=80, orientation="X")
                    start_flag = False
                else:
                    self.move_to_xyz_rpy(
                        x, y, z, target_orientation=[0, 0, -1],
                        mode='direct',
                        velocity=80, orientation="X"                                                                       # @@@@@@ 여기도 수정해야 함 @@@@@@
                    )
                
                if theta >= 6.2:  # 이 지점에서 루프 종료
                    break

                """ 
                    radius 2일 때 6
                    radius 3일 때 5
                    radius 4일 때 4
                    radius 5일 때 4
                    radius 6일 때 4
                    radius 7일 때 4
                """      


    def move_to_xyz_rpy(self, x, y, z, target_orientation=None, mode='trajectory',velocity=50, orientation=None):

        def convert_to_motor_angles(joint_angles_rad):

            joint_angles_deg = np.degrees(joint_angles_rad)
            motor_base = 210 - joint_angles_deg[0] # Joint 0 -> Base
            motor_joint1 = 179 - (joint_angles_deg[1] - 90) # Joint 1 -> Joint1
            motor_joint2 = 150 + joint_angles_deg[2] # Joint 2 -> Joint2
            motor_joint3 = 150 + joint_angles_deg[3] # Joint 3 -> Joint3
            return motor_base, motor_joint1, motor_joint2, motor_joint3
        
        target_position = np.array([x, y, z])
        previent_joint_angles = self.read_current_angle()
        if previent_joint_angles is not None:
            previent_joint_angles_rad = np.deg2rad(previent_joint_angles[0:6])
        else:
            previent_joint_angles_rad = np.array([0, 0, np.pi/2, 0, 0,  0.1]) 

        joint_angles_rad = inverse_kinematics_ikpy(
            target_position=target_position,
            target_orientation=target_orientation,
            initial_guess=previent_joint_angles_rad,
            orientation_mode=orientation
        )

        degrees = np.rad2deg(joint_angles_rad)
        motor_angles = convert_to_motor_angles(joint_angles_rad[1:5]) 

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
                self.joint2.move(motor_angles[2], velocity)
                self.base.move(motor_angles[0], velocity)
                self.joint1.move(motor_angles[1], velocity)
                    
            elif target_orientation == [1,0,0]:
                self.joint3.move(motor_angles[3], velocity)
                self.joint2.move(motor_angles[2], velocity)
                self.base.move(motor_angles[0], velocity)
                self.joint1.move(motor_angles[1], velocity)
                    
            else:
                self.joint3.move(motor_angles[3], velocity)
                self.joint2.move(motor_angles[2], velocity)
                self.base.move(motor_angles[0], velocity)
                self.joint1.move(motor_angles[1], velocity)
                    

    # def process(self):                              # ** 이거 **                # ** 이거 **
    #     self.draw_circle(center=(0.18, 0.035, 0.0), radius=0.027, plane='xy', z_fixed=0.035, num_points=100, velocity=10)
    #     """z_fixed
    #     """   

    def process(self):                              # ** 이거 **                # ** 이거 **
        self.draw_circle(center=(0.191, -0.029,0), radius=0.042, plane='xy', z_fixed=0.033, num_points=100, velocity=10)
        """z_fixed
        """  
        #2일 때는 0.035                         
        #3일 때는 0.035
        #4일 때는 0.035                                                                                                            #@@@@@@@ 바꿔야 함 @@@@@@@
        #5일 때는 0.032
        #6일 때는 0.032
        #7일 때는 0.032
        #오른쪽이 (+), 왼쪽이 (-)
    
    def process2(self):                              # ** 이거 **                # ** 이거 **
        self.draw_circle2(center=(0.205, 0.038, 0.0), radius=0.027, plane='xy', z_fixed=0.035, num_points=100, velocity=10)
        """z_fixed
        """   

def main(args=None):
    rclpy.init(args=args)                                                                                                                                                                                                                                                  
    node = DynamixelControlNode()
    try:
        node.process2()
        node.process()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
