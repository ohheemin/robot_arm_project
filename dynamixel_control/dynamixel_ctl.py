#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from dynamixel_sdk import *

# 모터 ID
XM_DXL_ID = [1, 2]
AX_DXL_ID = [1, 4]
XC_DXL_ID = 5

# 포트 이름
DEVICENAME = '/dev/ttyUSB0'

# Baudrate
XM1_BAUDRATE = 57600
XM2_BAUDRATE = 1000000
AX1_BAUDRATE = 115200
AX2_BAUDRATE = 115200
XC_BAUDRATE = 115200

# 프로토콜 버전
XM_PROTOCOL_VERSION = 2.0
AX_PROTOCOL_VERSION = 1.0
XC_PROTOCOL_VERSION = 2.0

# 주소값
XM_ADDR_TORQUE_ENABLE = 64
XM_ADDR_GOAL_POSITION = 116
XM_ADDR_PROFILE_ACCELERATION = 108

AX_ADDR_TORQUE_ENABLE = 24
AX_ADDR_MOVING_SPEED = 32
AX_ADDR_GOAL_POSITION = 30
AX_ADDR_CW_COMPLIANCE_SLOPE = 28
AX_ADDR_CCW_COMPLIANCE_SLOPE = 29

XC_ADDR_TORQUE_ENABLE = 64
XC_ADDR_GOAL_POSITION = 116

XM_TORQUE_ENABLE = 1
AX_TORQUE_ENABLE = 1
XC_TORQUE_ENABLE = 1

# 각도 → 포지션 변환
def angle_to_position(angle_deg, protocol_version):
    if protocol_version == 1.0:
        return int(angle_deg / 300.0 * 1023)
    else:
        return int(angle_deg / 360.0 * 4095)

class DynamixelControlNode(Node):
    def __init__(self):
        super().__init__('dynamixel_control_node')

        self.port_handler = PortHandler(DEVICENAME)

        if not self.port_handler.openPort():
            self.get_logger().error('포트 열기 실패')
            return
        else:
            self.get_logger().info('포트 열기 성공')

        # 각 모터별 PacketHandler
        self.port_handler.setBaudRate(XM1_BAUDRATE)
        self.packet_handler_xm1 = PacketHandler(XM_PROTOCOL_VERSION)

        self.port_handler.setBaudRate(XM2_BAUDRATE)
        self.packet_handler_xm2 = PacketHandler(XM_PROTOCOL_VERSION)

        self.port_handler.setBaudRate(AX1_BAUDRATE)
        self.packet_handler_ax1 = PacketHandler(AX_PROTOCOL_VERSION)

        self.port_handler.setBaudRate(AX2_BAUDRATE)
        self.packet_handler_ax2 = PacketHandler(AX_PROTOCOL_VERSION)

        self.port_handler.setBaudRate(XC_BAUDRATE)
        self.packet_handler_xc = PacketHandler(XC_PROTOCOL_VERSION)

        self.enable_and_move_all()

    def check_comm_result(self, result, error, motor_name, operation):
        if result != COMM_SUCCESS:
            self.get_logger().error(f'{motor_name} {operation} 실패: {self.packet_handler_xm1.getTxRxResult(result)}')
        elif error != 0:
            self.get_logger().warn(f'{motor_name} {operation} 에러 발생: {self.packet_handler_xm1.getRxPacketError(error)}')
        else:
            self.get_logger().info(f'{motor_name} {operation} 성공')

    def enable_and_move_all(self):
        # XM1
        dxl_id = XM_DXL_ID[0]
        result, error = self.packet_handler_xm1.write1ByteTxRx(self.port_handler, dxl_id, XM_ADDR_TORQUE_ENABLE, XM_TORQUE_ENABLE)
        self.check_comm_result(result, error, f'XM1 ID {dxl_id}', 'Torque Enable')
        result, error = self.packet_handler_xm1.write1ByteTxRx(self.port_handler, dxl_id, XM_ADDR_PROFILE_ACCELERATION, 5)
        self.check_comm_result(result, error, f'XM1 ID {dxl_id}', 'Set Acceleration')
        target_pos = angle_to_position(190, XM_PROTOCOL_VERSION)
        result, error = self.packet_handler_xm1.write4ByteTxRx(self.port_handler, dxl_id, XM_ADDR_GOAL_POSITION, target_pos)
        self.check_comm_result(result, error, f'XM1 ID {dxl_id}', 'Set Goal Position')

        # XM2
        dxl_id = XM_DXL_ID[1]
        result, error = self.packet_handler_xm2.write1ByteTxRx(self.port_handler, dxl_id, XM_ADDR_TORQUE_ENABLE, XM_TORQUE_ENABLE)
        self.check_comm_result(result, error, f'XM2 ID {dxl_id}', 'Torque Enable')
        result, error = self.packet_handler_xm2.write1ByteTxRx(self.port_handler, dxl_id, XM_ADDR_PROFILE_ACCELERATION, 5)
        self.check_comm_result(result, error, f'XM2 ID {dxl_id}', 'Set Acceleration')
        target_pos = angle_to_position(190, XM_PROTOCOL_VERSION)
        result, error = self.packet_handler_xm2.write4ByteTxRx(self.port_handler, dxl_id, XM_ADDR_GOAL_POSITION, target_pos)
        self.check_comm_result(result, error, f'XM2 ID {dxl_id}', 'Set Goal Position')

        # AX1
        dxl_id = AX_DXL_ID[0]
        result, error = self.packet_handler_ax1.write1ByteTxRx(self.port_handler, dxl_id, AX_ADDR_TORQUE_ENABLE, AX_TORQUE_ENABLE)
        self.check_comm_result(result, error, f'AX1 ID {dxl_id}', 'Torque Enable')
        result, error = self.packet_handler_ax1.write2ByteTxRx(self.port_handler, dxl_id, AX_ADDR_MOVING_SPEED, 100)
        self.check_comm_result(result, error, f'AX1 ID {dxl_id}', 'Set Speed')
        result, error = self.packet_handler_ax1.write1ByteTxRx(self.port_handler, dxl_id, AX_ADDR_CW_COMPLIANCE_SLOPE, 65)
        self.check_comm_result(result, error, f'AX1 ID {dxl_id}', 'CW Compliance Slope')
        result, error = self.packet_handler_ax1.write1ByteTxRx(self.port_handler, dxl_id, AX_ADDR_CCW_COMPLIANCE_SLOPE, 65)
        self.check_comm_result(result, error, f'AX1 ID {dxl_id}', 'CCW Compliance Slope')
        target_pos = angle_to_position(163, AX_PROTOCOL_VERSION)
        result, error = self.packet_handler_ax1.write2ByteTxRx(self.port_handler, dxl_id, AX_ADDR_GOAL_POSITION, target_pos)
        self.check_comm_result(result, error, f'AX1 ID {dxl_id}', 'Set Goal Position')

        # AX2
        dxl_id = AX_DXL_ID[1]
        result, error = self.packet_handler_ax2.write1ByteTxRx(self.port_handler, dxl_id, AX_ADDR_TORQUE_ENABLE, AX_TORQUE_ENABLE)
        self.check_comm_result(result, error, f'AX2 ID {dxl_id}', 'Torque Enable')
        result, error = self.packet_handler_ax2.write2ByteTxRx(self.port_handler, dxl_id, AX_ADDR_MOVING_SPEED, 100)
        self.check_comm_result(result, error, f'AX2 ID {dxl_id}', 'Set Speed')
        result, error = self.packet_handler_ax2.write1ByteTxRx(self.port_handler, dxl_id, AX_ADDR_CW_COMPLIANCE_SLOPE, 65)
        self.check_comm_result(result, error, f'AX2 ID {dxl_id}', 'CW Compliance Slope')
        result, error = self.packet_handler_ax2.write1ByteTxRx(self.port_handler, dxl_id, AX_ADDR_CCW_COMPLIANCE_SLOPE, 65)
        self.check_comm_result(result, error, f'AX2 ID {dxl_id}', 'CCW Compliance Slope')
        target_pos = angle_to_position(163, AX_PROTOCOL_VERSION)
        result, error = self.packet_handler_ax2.write2ByteTxRx(self.port_handler, dxl_id, AX_ADDR_GOAL_POSITION, target_pos)
        self.check_comm_result(result, error, f'AX2 ID {dxl_id}', 'Set Goal Position')

        # XC
        dxl_id = XC_DXL_ID
        result, error = self.packet_handler_xc.write1ByteTxRx(self.port_handler, dxl_id, XC_ADDR_TORQUE_ENABLE, XC_TORQUE_ENABLE)
        self.check_comm_result(result, error, f'XC ID {dxl_id}', 'Torque Enable')
        target_pos = angle_to_position(280, XC_PROTOCOL_VERSION)
        result, error = self.packet_handler_xc.write4ByteTxRx(self.port_handler, dxl_id, XC_ADDR_GOAL_POSITION, target_pos)
        self.check_comm_result(result, error, f'XC ID {dxl_id}', 'Set Goal Position')

def main(args=None):
    rclpy.init(args=args)
    node = DynamixelControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
