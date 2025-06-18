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

# Note on DXL_ID comments: The actual mapping in DynamixelControlNode's __init__
# determines which ID corresponds to which joint name.
XM_DXL_ID = [1, 2]  # As per DynamixelControlNode: ID 1 is base, ID 2 is joint2
AX_DXL_ID = [3, 4]  # As per DynamixelControlNode: ID 3 is joint1, ID 4 is joint3
XC_DXL_ID = [5]     # As per DynamixelControlNode: ID 5 is end-effector

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
# Assuming the order of DH parameters directly corresponds to
# Joint 0 (Base), Joint 1, Joint 2, Joint 3, Joint 4 (End-effector)
DH_params = [
    [0,                 0.059,    0,        0],     # Joint 0 (Base)
    [0,                 0.059,    0,  -np.pi/2],    # Joint 1
    [-np.pi/2,          0,    0.13,        0],     # Joint 2
    [0,                 0,    0.13,        0],     # Joint 3
    [0,                 0, 0.10,        0],       # Joint 4 (End-effector)
]

num_joints = len(DH_params)

# === 유틸 함수 ===
def angle_to_position(angle_deg, protocol_version):
    """
    Converts an angle in degrees to Dynamixel motor position units.
    Handles both Protocol 1.0 (AX series) and Protocol 2.0 (XM/XC series).
    """
    # Ensure angle is within a practical range for conversion
    # For AX series, 300 degrees = 1023 units
    # For XM/XC series, 360 degrees = 4095 units
    if protocol_version == 1.0: # AX series (300 degree range)
        # Map input angle_deg (which can be outside 0-300) into the motor's effective range if needed.
        # However, IK bounds should already constrain it.
        # Typical mapping: 0 degrees -> 0, 300 degrees -> 1023
        # If angle_deg represents a relative change from a center (e.g., 150 for AX),
        # ensure the final calculated motor position is within 0-1023.
        return int(angle_deg / 300.0 * 1023)
    else: # XM/XC series (360 degree range)
        # Typical mapping: 0 degrees -> 0, 360 degrees -> 4095
        return int(angle_deg / 360.0 * 4095)

def forward_kinematics(joints):
    """
    Calculates the end-effector position (and other joint positions)
    given the joint angles using DH parameters.
    """
    T = np.eye(4) # Transformation matrix from base to current link
    positions = [] # List to store the (x, y, z) position of each joint frame
    for i, (theta_offset, d, a, alpha) in enumerate(DH_params):
        # theta: joint variable from input + DH theta_offset
        theta = joints[i] + theta_offset
        
        # Transformation matrix from link i-1 to link i
        T_i = np.array([
            [np.cos(theta), -np.sin(theta)*np.cos(alpha),  np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
            [np.sin(theta),  np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
            [0,              np.sin(alpha),                np.cos(alpha),               d],
            [0,              0,                            0,                           1]
        ])
        T = T @ T_i # Accumulate transformation
        positions.append(T[:3, 3]) # Store the (x, y, z) position of the current joint frame origin
    return positions

def inverse_kinematics(target_pos, initial_guess=np.zeros(num_joints)):
    """
    Finds the joint angles required to reach a target 3D position
    using numerical optimization (minimize).
    """
    # Bounds for joint angles in radians
    # These bounds are critical for the IK solver to find a valid solution
    # and should reflect the physical limits of your robot's joints.
    bounds = [
        (math.radians(-180), math.radians(180)), # Joint 0 (Base)
        (math.radians(-90), math.radians(90)),   # Joint 1
        (math.radians(-180), math.radians(180)), # Joint 2
        (math.radians(-60), math.radians(0)),    # Joint 3 (This bound is quite restrictive)
        (math.radians(-180), math.radians(180)), # Joint 4 (End-effector orientation)
    ]

    def objective_function(joints):
        """
        The function to minimize: squared Euclidean distance between
        current end-effector position and the target position.
        """
        pos = forward_kinematics(joints)[-1] # Get only the end-effector position
        # The print statement is useful for debugging but can be verbose during operation
        # print(f"Current End-Effector Position: {pos}")
        return np.sum((pos - target_pos) ** 2)

    # Use L-BFGS-B method which supports bounds
    result = minimize(objective_function, initial_guess, method='L-BFGS-B', bounds=bounds)
    
    if not result.success:
        print(f"Warning: IK did not converge for target {target_pos}. Message: {result.message}")
        print(f"Final objective value: {result.fun}")
        print(f"Joint angles found (possibly not optimal): {np.degrees(result.x)}")

    return result.x if result.success else initial_guess # Return the optimized joint angles

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
        
        # Set initial baud rate for the port handler for this motor
        self.port_handler.setBaudRate(baudrate)

    def setting(self):
        """
        Ensures the baud rate is set and torque is enabled for the motor.
        """
        # This is redundant if set in __init__ and baudrate doesn't change.
        # Kept for consistency with original code's flow.
        if not self.port_handler.setBaudRate(self.baudrate):
            self.logger.error(f'{self.name} Baudrate 설정 실패')
            raise RuntimeError(f'{self.name} Baudrate 설정 실패')

        # Enable torque for the motor
        if self.is_ax:
            result, error = self.packet_handler.write1ByteTxRx(self.port_handler, self.dxl_id, AX_ADDR_TORQUE_ENABLE, AX_TORQUE_ENABLE)
        elif self.is_xc:
            result, error = self.packet_handler.write1ByteTxRx(self.port_handler, self.dxl_id, XC_ADDR_TORQUE_ENABLE, XC_TORQUE_ENABLE)
        else: # XM series
            result, error = self.packet_handler.write1ByteTxRx(self.port_handler, self.dxl_id, ADDR_TORQUE_ENABLE, XM_TORQUE_ENABLE)
        self._check_comm_result(result, error, 'Enable Torque')

    def move(self, angle_deg, velocity=50):
        """
        Commands the motor to move to a specified angle in degrees.
        Adjusts velocity for different motor types.
        """
        self.setting() # Ensure torque is enabled and baud rate is set

        protocol = AX_PROTOCOL_VERSION if self.is_ax else XC_PROTOCOL_VERSION if self.is_xc else XM_PROTOCOL_VERSION
        position = angle_to_position(angle_deg, protocol)

        # Write goal position and velocity based on motor type
        if self.is_ax:
            # AX series uses 2-byte for velocity and position
            self.packet_handler.write2ByteTxRx(self.port_handler, self.dxl_id, AX_ADDR_MOVING_SPEED, velocity)
            result, error = self.packet_handler.write2ByteTxRx(self.port_handler, self.dxl_id, AX_ADDR_GOAL_POSITION, position)
        elif self.is_xc:
            # XC series uses 4-byte for velocity and position
            self.packet_handler.write4ByteTxRx(self.port_handler, self.dxl_id, XC_ADDR_GOAL_VELOCITY, velocity)
            result, error = self.packet_handler.write4ByteTxRx(self.port_handler, self.dxl_id, XC_ADDR_GOAL_POSITION, position)
        else: # XM series
            # XM series uses 4-byte for velocity and position
            self.packet_handler.write4ByteTxRx(self.port_handler, self.dxl_id, ADDR_PROFILE_VELOCITY, velocity)
            result, error = self.packet_handler.write4ByteTxRx(self.port_handler, self.dxl_id, ADDR_GOAL_POSITION, position)
        self._check_comm_result(result, error, f'Move to {angle_deg}°')

    def _check_comm_result(self, result, error, operation):
        """
        Helper to check Dynamixel communication results and log them.
        """
        if result != COMM_SUCCESS:
            self.logger.error(f'{self.name} {operation} 실패: {self.packet_handler.getTxRxResult(result)}')
        elif error != 0:
            self.logger.warn(f'{self.name} {operation} 에러 발생: {self.packet_handler.getRxPacketError(error)}')
        # else:
            # self.logger.info(f'{self.name} {operation} 성공') # Commented out to reduce log verbosity during circle drawing

# === ROS2 노드 ===
class DynamixelControlNode(Node):
    def __init__(self):
        super().__init__('dynamixel_control_node')
        self.port_handler = PortHandler(DEVICENAME)
        if not self.port_handler.openPort():
            self.get_logger().error('포트 열기 실패')
            return
        self.get_logger().info('포트 열기 성공')

        # 모터 인스턴스 생성 (Mapping from logical joint names to physical DXL IDs)
        # ID 1: Base motor (XM)
        # ID 3: Joint 1 motor (AX)
        # ID 2: Joint 2 motor (XM)
        # ID 4: Joint 3 motor (AX)
        # ID 5: End-effector motor (XC)
        self.base = DynamixelMotor(self.port_handler, 1, "base", 1000000, XM_PROTOCOL_VERSION, self.get_logger())
        self.joint1 = DynamixelMotor(self.port_handler, 3, "joint1", 1000000, AX_PROTOCOL_VERSION, self.get_logger(), is_ax=True)
        self.joint2 = DynamixelMotor(self.port_handler, 2, "joint2", 1000000, XM_PROTOCOL_VERSION, self.get_logger())
        self.joint3 = DynamixelMotor(self.port_handler, 4, "joint3", 1000000, AX_PROTOCOL_VERSION, self.get_logger(), is_ax=True)
        self.end_effector = DynamixelMotor(self.port_handler, 5, "end_effector", 1000000, XC_PROTOCOL_VERSION, self.get_logger(), is_xc=True)

        # Store initial positions of the motors (in degrees of their range)
        self.initial_motor_positions = {
            "base": 180,
            "joint1": 150,
            "joint2": 180,
            "joint3": 150,
            "end_effector": 270,
        }

    def move_to_initial_position(self):
        """
        Moves all robot joints to their predefined initial/home positions.
        These positions often correspond to the DH parameter's 'zero' angle.
        """
        self.get_logger().info('로봇을 초기 위치로 이동 중...')
        self.base.move(self.initial_motor_positions["base"])
        self.joint1.move(self.initial_motor_positions["joint1"])
        self.joint2.move(self.initial_motor_positions["joint2"])
        self.joint3.move(self.initial_motor_positions["joint3"])
        self.end_effector.move(self.initial_motor_positions["end_effector"])
        time.sleep(2.0) # Give motors time to reach position

    def process(self):
        """
        Main process to control the robot: moves to initial position,
        prompts for circle parameters, draws the circle, then returns to an initial state.
        """
        # Move to a safe initial position before any task
        self.move_to_initial_position()
        self.get_logger().info('초기 위치 이동 완료. 원 그리기 준비 중.')

        # --- Get user input for circle parameters ---
        try:
            center_x = float(input("원의 중심 X 좌표를 입력하세요 (미터, 예: -0.1): "))
            center_y = float(input("원의 중심 Y 좌표를 입력하세요 (미터, 예: 0.0): "))
            center_z = float(input("원의 중심 Z 좌표를 입력하세요 (미터, 예: 0.15): "))
            radius = float(input("원의 반지름을 입력하세요 (미터, 예: 0.05): "))
            num_points = int(input("원을 그릴 점의 개수를 입력하세요 (예: 50): "))
            if num_points < 2:
                self.get_logger().error("점의 개수는 2개 이상이어야 합니다.")
                return
        except ValueError:
            self.get_logger().error("잘못된 입력입니다. 숫자를 입력해주세요.")
            return

        self.get_logger().info(f"원을 그립니다: 중심 ({center_x}, {center_y}, {center_z}), 반지름 {radius} m, 점 {num_points}개.")
        
        # --- Generate circle points and move the robot ---
        # Use a fixed Z-coordinate for the circle points (assuming a horizontal circle)
        current_initial_guess = np.zeros(num_joints) # Initial guess for IK, updated with previous solution

        for i in range(num_points + 1): # +1 to close the circle
            angle = 2 * np.pi * i / num_points
            
            # Calculate target point on the circle
            target_x = center_x + radius * np.cos(angle)
            target_y = center_y + radius * np.sin(angle)
            target_z = center_z # Z remains constant for a horizontal circle
            
            target_pos = np.array([target_x, target_y, target_z])
            self.get_logger().info(f"현재 목표 위치: ({target_x:.3f}, {target_y:.3f}, {target_z:.3f})")

            # Perform inverse kinematics
            joint_angles_rad = inverse_kinematics(target_pos, initial_guess=current_initial_guess)
            
            if np.array_equal(joint_angles_rad, current_initial_guess):
                self.get_logger().warn(f"목표 위치에 대한 IK 솔루션을 찾을 수 없거나 수렴하지 않았습니다. {target_pos}")
                # Optionally, skip this point or try a different strategy
                continue 
            
            # Update initial guess for next iteration to speed up convergence
            current_initial_guess = joint_angles_rad 

            joint_angles_deg = np.degrees(joint_angles_rad)
            self.get_logger().info(f"계산된 조인트 각도 (deg): {joint_angles_deg}")

            # --- Command motors based on IK results ---
            # IMPORTANT: Verify these signs and offsets with your physical robot.
            # 180, 150, 180, 150, 270 are assumed to be the motor's "home" positions
            # where the DH joint angle is 0.
            # The sign (+ or -) depends on the motor's physical mounting direction.
            try:
                self.base.move(self.initial_motor_positions["base"] + joint_angles_deg[0])
                self.joint1.move(self.initial_motor_positions["joint1"] + joint_angles_deg[1])
                self.joint2.move(self.initial_motor_positions["joint2"] + joint_angles_deg[2])
                self.joint3.move(self.initial_motor_positions["joint3"] + joint_angles_deg[3])
                self.end_effector.move(self.initial_motor_positions["end_effector"] + joint_angles_deg[4])
                time.sleep(0.1) # Small delay for smooth motion
            except RuntimeError as e:
                self.get_logger().error(f"모터 제어 중 오류 발생: {e}. 다음 지점으로 넘어갑니다.")
                continue

        self.get_logger().info("원 그리기 완료!")
        time.sleep(2.0) # Wait after drawing

        # --- Return to a safe home position ---
        self.move_to_initial_position()
        self.get_logger().info("로봇이 초기 위치로 돌아왔습니다. 프로그램 종료.")

# === main ===
def main(args=None):
    rclpy.init(args=args)
    node = DynamixelControlNode()
    try:
        node.process()
    except KeyboardInterrupt:
        node.get_logger().info("프로그램이 수동으로 종료되었습니다.")
    finally:
        rclpy.spin_once(node, timeout_sec=1.0) # Spin once to process any remaining messages
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()