#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from dynamixel_sdk import *
import time

# 공통 설정
DEVICENAME = '/dev/ttyUSB0'

# 모터 ID 설정 (XM, AX, XC)
XM_DXL_ID = [1, 2]  # XM 모터 (XM1, XM2)
AX_DXL_ID = [1, 4]  # AX 모터 (AX1, AX2)
XC_DXL_ID = [5]     # XC 모터 (XC1)

# 포트 설정 1000000 baudrate 통일

# Baudrate 설정
XM1_BAUDRATE = 57600
XM2_BAUDRATE = 1000000

AX1_BAUDRATE = 115200 
AX2_BAUDRATE = 115200

XC_BAUDRATE = 115200

# 프로토콜 버전 설정
XM_PROTOCOL_VERSION = 2.0
AX_PROTOCOL_VERSION = 1.0
XC_PROTOCOL_VERSION = 2.0

# 주소값 설정 (각각의 모터에 대한 설정)
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

def angle_to_position(angle_deg, protocol_version):
    if protocol_version == 1.0:
        return int(angle_deg / 300.0 * 1023)
    else:
        return int(angle_deg / 360.0 * 4095)

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
        # Set Baudrate
        self.port_handler.setBaudRate(self.baudrate)
        if not self.port_handler.setBaudRate(self.baudrate):
            self.logger.error(f'{self.name} Baudrate 설정 실패')
            raise RuntimeError(f'{self.name} Baudrate 설정 실패')

        # Enable Torque
        if self.is_ax:
            result, error = self.packet_handler.write1ByteTxRx(
                self.port_handler, self.dxl_id, AX_ADDR_TORQUE_ENABLE, AX_TORQUE_ENABLE
            )
        elif self.is_xc:
            result, error = self.packet_handler.write1ByteTxRx(
                self.port_handler, self.dxl_id, XC_ADDR_TORQUE_ENABLE, XC_TORQUE_ENABLE
            )
        else:
            result, error = self.packet_handler.write1ByteTxRx(
                self.port_handler, self.dxl_id, ADDR_TORQUE_ENABLE, XM_TORQUE_ENABLE
            )

        self._check_comm_result(result, error, 'Enable Torque')


    def move(self, angle_deg, velocity=50):
        self.setting()
        if self.is_ax:
            position = angle_to_position(angle_deg, AX_PROTOCOL_VERSION)
        elif self.is_xc:
            position = angle_to_position(angle_deg, XC_PROTOCOL_VERSION)
        else:
            position = angle_to_position(angle_deg, XM_PROTOCOL_VERSION)
        
        # Set Velocity
        if self.is_ax:
            result, error = self.packet_handler.write2ByteTxRx(
                self.port_handler, self.dxl_id, AX_ADDR_MOVING_SPEED, velocity)
        elif self.is_xc:
            result, error = self.packet_handler.write4ByteTxRx(
                self.port_handler, self.dxl_id, XC_ADDR_GOAL_VELOCITY, velocity)
        else:
            result, error = self.packet_handler.write4ByteTxRx(
                self.port_handler, self.dxl_id, ADDR_PROFILE_VELOCITY, velocity)

        self._check_comm_result(result, error, 'Set Profile Velocity')

        if self.is_ax:
            result, error = self.packet_handler.write2ByteTxRx(
                self.port_handler, self.dxl_id, AX_ADDR_GOAL_POSITION, position
            )
        elif self.is_xc:
            result, error = self.packet_handler.write4ByteTxRx(
                self.port_handler, self.dxl_id, XC_ADDR_GOAL_POSITION, position
            )
        else:
            result, error = self.packet_handler.write4ByteTxRx(
                self.port_handler, self.dxl_id, ADDR_GOAL_POSITION, position
            )
        self._check_comm_result(result, error, f'Move to {angle_deg}°')


    def _check_comm_result(self, result, error, operation):
        if result != COMM_SUCCESS:
            self.logger.error(f'{self.name} {operation} 실패: {self.packet_handler.getTxRxResult(result)}')
        elif error != 0:
            self.logger.warn(f'{self.name} {operation} 에러 발생: {self.packet_handler.getRxPacketError(error)}')
        else:
            self.logger.info(f'{self.name} {operation} 성공')

class DynamixelControlNode(Node):
    def __init__(self):
        super().__init__('dynamixel_control_node')

        self.port_handler = PortHandler(DEVICENAME)
        if not self.port_handler.openPort():
            self.get_logger().error('포트 열기 실패')
            return

        self.get_logger().info('포트 열기 성공')

        # XM 모터 : joint2
        self.motor1 = DynamixelMotor(
            port_handler=self.port_handler,
            dxl_id=1,
            baudrate=57600,
            motor_name='XM1',
            protocol_version=XM_PROTOCOL_VERSION,
            node_logger=self.get_logger()
        )
        # XM 모터 : base
        self.motor2 = DynamixelMotor(
            port_handler=self.port_handler,
            dxl_id=2,
            baudrate=1000000,
            motor_name='XM2',
            protocol_version=XM_PROTOCOL_VERSION,
            node_logger=self.get_logger()
        )
        # AX 모터 : joint1
        self.ax_motor1 = DynamixelMotor(
            port_handler=self.port_handler,
            dxl_id=1,
            baudrate=AX1_BAUDRATE,
            motor_name='AX1',
            protocol_version=AX_PROTOCOL_VERSION,
            node_logger=self.get_logger(),
            is_ax=True
        )
        # AX 모터 : joint4
        self.ax_motor2 = DynamixelMotor(
            port_handler=self.port_handler,
            dxl_id=4,
            baudrate=AX2_BAUDRATE,
            motor_name='AX2',
            protocol_version=AX_PROTOCOL_VERSION,
            node_logger=self.get_logger(),
            is_ax=True
        )

        # XC 모터 인스턴스 생성
        self.xc_motor1 = DynamixelMotor(
            port_handler=self.port_handler,
            dxl_id=5,
            baudrate=XC_BAUDRATE,
            motor_name='XC1',
            protocol_version=XC_PROTOCOL_VERSION,
            node_logger=self.get_logger(),
            is_xc=True
        )

    def process(self):
        # XM, AX, XC 모터 모두 제어

        #base
        #self.motor2.move(13)
        
        #joint1
        self.ax_motor1.move(190)
        
        #joint2
        #self.motor1.move(170)
        
        #joint3
        self.ax_motor2.move(200)
        
        #end
        #self.xc_motor1.move(300)
        time.sleep(1.0)


def main(args=None):
    rclpy.init(args=args)
    node = DynamixelControlNode()
    node.process()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
