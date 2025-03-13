import time
from dynamixel_sdk import *

# 기본 설정
DXL_ID_LIST = [3, 7, 4]
ADDR_PRESENT_POSITION = 132
LEN_PRESENT_POSITION = 4
BAUDRATE = 1000000
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

# GroupSyncRead 초기화
groupSyncRead = GroupSyncRead(portHandler, packetHandler, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)

# Dynamixel ID 등록
for dxl_id in DXL_ID_LIST:
    if not groupSyncRead.addParam(dxl_id):
        print(f"[ID:{dxl_id}] Sync Read addParam 실패")
        exit()

# 📌 Sync Read 성능 테스트

if groupSyncRead.txRxPacket() != COMM_SUCCESS:
    print("Sync Read 통신 실패")
    exit()

for dxl_id in DXL_ID_LIST:
    if not groupSyncRead.isAvailable(dxl_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION):
        print(f"[ID:{dxl_id}] 데이터 획득 실패 (Sync)")
        exit()
    position = groupSyncRead.getData(dxl_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
    print("position: ", position)


# 📌 Fast Sync Read 성능 테스트
if groupSyncRead.fastSyncRead() != COMM_SUCCESS:
    print("Fast Sync Read 통신 실패")
    exit()

for dxl_id in DXL_ID_LIST:
    if not groupSyncRead.isAvailable(dxl_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION):
        print(f"[ID:{dxl_id}] 데이터 획득 실패 (Fast Sync)")
        exit()
    position = groupSyncRead.getData(dxl_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
    print("position: ", position)


# 포트 닫기
portHandler.closePort()
