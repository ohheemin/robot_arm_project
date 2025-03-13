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

# GroupBulkRead 초기화
groupBulkRead = GroupBulkRead(portHandler, packetHandler)

# Dynamixel ID 등록 및 주소 설정
for dxl_id in DXL_ID_LIST:
    if not groupBulkRead.addParam(dxl_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION):
        print(f"[ID:{dxl_id}] Bulk Read addParam 실패")
        exit()

# 📌 Bulk Read 성능 테스트
start_time = time.perf_counter()
for _ in range(100):
    if groupBulkRead.txRxPacket() != COMM_SUCCESS:
        print("Bulk Read 통신 실패")
        exit()

    for dxl_id in DXL_ID_LIST:
        if not groupBulkRead.isAvailable(dxl_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION):
            print(f"[ID:{dxl_id}] 데이터 획득 실패 (Bulk)")
            exit()
        position = groupBulkRead.getData(dxl_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
end_time = time.perf_counter()
bulk_read_time = (end_time - start_time) / 100

# 📌 Fast Bulk Read 성능 테스트
start_time = time.perf_counter()
for _ in range(100):
    if groupBulkRead.fastBulkRead() != COMM_SUCCESS:
        print("Fast Bulk Read 통신 실패")
        exit()

    for dxl_id in DXL_ID_LIST:
        if not groupBulkRead.isAvailable(dxl_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION):
            print(f"[ID:{dxl_id}] 데이터 획득 실패 (Fast Bulk)")
            exit()
        position = groupBulkRead.getData(dxl_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
end_time = time.perf_counter()
fast_bulk_read_time = (end_time - start_time) / 100

# 결과 출력
print("\n🚀 Fast Bulk Read 성능 평가 결과 🚀")
print(f"📌 Bulk Read 평균 실행 시간: {bulk_read_time:.8f} 초")
print(f"📌 Fast Bulk Read 평균 실행 시간: {fast_bulk_read_time:.8f} 초")
print(f"📌 속도 향상 비율: {bulk_read_time / fast_bulk_read_time:.2f} 배 빠름")

# 포트 닫기
portHandler.closePort()