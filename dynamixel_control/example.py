#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from dynamixel_sdk import *
import time

# 공통 설정
DEVICENAME = '/dev/ttyUSB0'
PROTOCOL_VERSION = 2.0
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116
ADDR_PROFILE_VELOCITY = 112
ADDR_PROFILE_ACCELERATION = 108
TORQUE_ENABLE = 1

def angle_to_position(angle_deg, protocol_version):
    if protocol_version == 1.0:
        return int(angle_deg / 300.0 * 1023)
    else:
        return int(angle_deg / 360.0 * 4095)


class DynamixelMotor:
    def __init__(self, port_handler, dxl_id, baudrate, motor_name, node_logger):
        self.dxl_id = dxl_id
        self.baudrate = baudrate
        self.name = motor_name
        self.port_handler = port_handler
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)
        self.logger = node_logger
        self.port_handler.setBaudRate(baudrate)  

    def setting(self):
        # Set Baudrate
        self.port_handler.setBaudRate(self.baudrate)
        if not self.port_handler.setBaudRate(self.baudrate):
            self.logger.error(f'{self.name} Baudrate 설정 실패')
            raise RuntimeError(f'{self.name} Baudrate 설정 실패')
        # Enable Torque
        result, error = self.packet_handler.write1ByteTxRx(
            self.port_handler, self.dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE
        )
        self._check_comm_result(result, error, 'Enable Torque')

        # # Set Profile Acceleration
        # result, error = self.packet_handler.write4ByteTxRx(
        #     self.port_handler, self.dxl_id, ADDR_PROFILE_ACCELERATION, 5)
        # self._check_comm_result(result, error, 'Set Profile Acceleration')

    def move(self, angle_deg, velocity=5):
        self.setting()
        position = angle_to_position(angle_deg, PROTOCOL_VERSION)
        
        # Set Velocity
        result, error = self.packet_handler.write4ByteTxRx(
            self.port_handler, self.dxl_id, ADDR_PROFILE_VELOCITY, 5)
        self._check_comm_result(result, error, 'Set Profile Velocity')

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

        # 모터 인스턴스 생성
        self.motor1 = DynamixelMotor(
            port_handler=self.port_handler,
            dxl_id=1,
            baudrate=57600,
            motor_name='XM1',
            node_logger=self.get_logger()
        )

        self.motor2 = DynamixelMotor(
            port_handler=self.port_handler,
            dxl_id=2,
            baudrate=1000000,
            motor_name='XM2',
            node_logger=self.get_logger()
        )
    def process(self):
        # 원하는 각도만 넣어 제어
        self.motor1.move(190)
        time.sleep(1.0)
        self.motor2.move(180)


def main(args=None):
    rclpy.init(args=args)
    node = DynamixelControlNode()
    node.process()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()



# import rclpy
# from rclpy.node import Node
# from dynamixel_sdk import *

# # 모터 ID
# XM_DXL_ID = [1, 2]  # XM1, XM2 연결

# # 포트 이름
# DEVICENAME = '/dev/ttyUSB0'

# # Baudrate
# XM1_BAUDRATE = 57600  # XM1 Baudrate
# XM2_BAUDRATE = 1000000  # XM2 Baudrate

# # 프로토콜 버전
# XM_PROTOCOL_VERSION = 2.0

# # 주소값
# XM_ADDR_TORQUE_ENABLE = 64
# XM_ADDR_GOAL_POSITION = 116
# XM_ADDR_PROFILE_ACCELERATION = 108

# XM_TORQUE_ENABLE = 1

# # 각도 → 포지션 변환
# def angle_to_position(angle_deg, protocol_version):
#     if protocol_version == 1.0:
#         return int(angle_deg / 300.0 * 1023)
#     else:
#         return int(angle_deg / 360.0 * 4095)

# class DynamixelControlNode(Node):
#     def __init__(self):
#         super().__init__('dynamixel_control_node')

#         self.port_handler = PortHandler(DEVICENAME)

#         if not self.port_handler.openPort():
#             self.get_logger().error('포트 열기 실패')
#             return
#         else:
#             self.get_logger().info('포트 열기 성공')

#         # XM1 모터 PacketHandler
#         self.port_handler.setBaudRate(XM1_BAUDRATE)  # XM1 Baudrate
#         self.packet_handler_xm1 = PacketHandler(XM_PROTOCOL_VERSION)
#         # # XM2 모터 PacketHandler
#         self.port_handler.setBaudRate(XM2_BAUDRATE)  # XM2 Baudrate
#         self.packet_handler_xm2 = PacketHandler(XM_PROTOCOL_VERSION)

#         self.enable_and_move_all()

#     def check_comm_result(self, result, error, motor_name, operation):
#         if result != COMM_SUCCESS:
#             self.get_logger().error(f'{motor_name} {operation} 실패: {self.packet_handler_xm1.getTxRxResult(result)}')
#         elif error != 0:
#             self.get_logger().warn(f'{motor_name} {operation} 에러 발생: {self.packet_handler_xm1.getRxPacketError(error)}')
#         else:
#             self.get_logger().info(f'{motor_name} {operation} 성공')


#     def enable_and_move_all(self):
#         # XM1
#         dxl_id = XM_DXL_ID[0]
#         self.port_handler.setBaudRate(XM1_BAUDRATE)  # XM1 Baudrate
#         self.packet_handler_xm1 = PacketHandler(XM_PROTOCOL_VERSION)
#         result, error = self.packet_handler_xm1.write1ByteTxRx(self.port_handler, dxl_id, XM_ADDR_TORQUE_ENABLE, XM_TORQUE_ENABLE)
#         self.check_comm_result(result, error, f'XM1 ID {dxl_id}', 'Torque Enable')
#         result, error = self.packet_handler_xm1.write4ByteTxRx(self.port_handler, dxl_id, XM_ADDR_PROFILE_ACCELERATION, 5)  # 4바이트로 설정
#         self.check_comm_result(result, error, f'XM1 ID {dxl_id}', 'Set Acceleration')
#         target_pos = angle_to_position(190, XM_PROTOCOL_VERSION)
#         result, error = self.packet_handler_xm1.write4ByteTxRx(self.port_handler, dxl_id, XM_ADDR_GOAL_POSITION, target_pos)
#         self.check_comm_result(result, error, f'XM1 ID {dxl_id}', 'Set Goal Position')



#         time.sleep(3)  # 포트 열기 후 1초 대기
#         # XM2
#         self.port_handler.setBaudRate(XM2_BAUDRATE)  # XM2 Baudrate
#         self.packet_handler_xm2 = PacketHandler(XM_PROTOCOL_VERSION)
#         dxl_id = XM_DXL_ID[1]
#         result, error = self.packet_handler_xm2.write1ByteTxRx(self.port_handler, dxl_id, XM_ADDR_TORQUE_ENABLE, XM_TORQUE_ENABLE)
#         self.check_comm_result(result, error, f'XM2 ID {dxl_id}', 'Torque Enable')
#         result, error = self.packet_handler_xm2.write4ByteTxRx(self.port_handler, dxl_id, XM_ADDR_PROFILE_ACCELERATION, 5)  # 4바이트로 설정
#         self.check_comm_result(result, error, f'XM2 ID {dxl_id}', 'Set Acceleration')
#         target_pos = angle_to_position(190, XM_PROTOCOL_VERSION)
#         result, error = self.packet_handler_xm2.write4ByteTxRx(self.port_handler, dxl_id, XM_ADDR_GOAL_POSITION, target_pos)
#         self.check_comm_result(result, error, f'XM2 ID {dxl_id}', 'Set Goal Position')

# def main(args=None):
#     rclpy.init(args=args)
#     node = DynamixelControlNode()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
