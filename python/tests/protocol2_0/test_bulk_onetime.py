import time
from dynamixel_sdk import *

# 기본 설정
DXL_ID_LIST = [1, 3, 7, 4]
ADDR_PRESENT_POSITION = 132
LEN_PRESENT_POSITION = 4
ADDR_LED_RED = 65
LEN_LED_RED = 1
BAUDRATE = 1000000
ADDR_PRESENT_VELOCITY = 128
LEN_PRESENT_VELOCITY = 4
SERIAL_PORT = '/dev/ttyUSB0'

# 포트 및 패킷 핸들러 초기화
portHandler = PortHandler(SERIAL_PORT)
packetHandler = PacketHandler(2.0)

# 포트 열기 및 속도 설정
if not portHandler.openPort():
    print("포트 열기 실패")
    exit()
if not portHandler.setBaudRate(BAUDRATE):
    print("보드레이트 설정 실패")
    exit()

# GroupBulkRead 초기화
groupBulkRead = GroupBulkRead(portHandler, packetHandler)

# ID별로 정확한 주소를 추가
groupBulkRead.addParam(1, ADDR_LED_RED, LEN_LED_RED)
groupBulkRead.addParam(3, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
groupBulkRead.addParam(7, ADDR_LED_RED, LEN_LED_RED)
groupBulkRead.addParam(4, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)

# 📌 Fast Bulk Read 성능 테스트
if groupBulkRead.fastBulkRead() != COMM_SUCCESS:
    print("Fast Bulk Read 통신 실패")
    exit()
    dxl1_led_value_read = group_bulk_read.getData(DXL_ID_LIST[0], ADDR_LED_RED, LEN_LED_RED)
    dxl2_present_position = groupBulkRead.getData(DXL_ID_LIST[1], ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
    dxl3_led_value_read = groupBulkRead.getData(DXL_ID_LIST[2], ADDR_LED_RED, LEN_LED_RED)
    dxl4_present_velocity = groupBulkRead.getData(DXL_ID_LIST[3], ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY)


# 포트 닫기
portHandler.closePort()
