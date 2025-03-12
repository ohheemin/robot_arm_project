#!/usr/bin/env python
# -*- coding: utf-8 -*-

################################################################################
# Copyright 2017 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
################################################################################

# Author: Ryu Woon Jung (Leon)

from .robotis_def import *


class GroupSyncRead:
    def __init__(self, port, ph, start_address, data_length):
        self.port = port
        self.ph = ph
        self.start_address = start_address
        self.data_length = data_length

        self.last_result = False
        self.is_param_changed = False
        self.param = []
        self.data_dict = {}

        self.clearParam()

    def makeParam(self):
        if self.ph.getProtocolVersion() == 1.0:
            return

        if not self.is_param_changed:  # ✅ 추가된 최적화
            return

        if not self.data_dict:
            return

        self.param = list(self.data_dict.keys())  # ✅ 불필요한 반복문 제거

        self.is_param_changed = False  # ✅ 변경 이후 다시 실행되지 않도록 설정


    def addParam(self, dxl_id):
        if self.ph.getProtocolVersion() == 1.0:
            return False

        if dxl_id in self.data_dict:  # dxl_id already exist
            return False

        self.data_dict[dxl_id] = []  # [0] * self.data_length

        self.is_param_changed = True
        return True

    def removeParam(self, dxl_id):
        if self.ph.getProtocolVersion() == 1.0:
            return

        if dxl_id not in self.data_dict:  # NOT exist
            return

        del self.data_dict[dxl_id]

        self.is_param_changed = True

    def clearParam(self):
        if self.ph.getProtocolVersion() == 1.0:
            return

        self.data_dict.clear()

    def txPacket(self):
        if self.ph.getProtocolVersion() == 1.0 or len(self.data_dict.keys()) == 0:
            return COMM_NOT_AVAILABLE

        if self.is_param_changed is True or not self.param:
            self.makeParam()

        return self.ph.syncReadTx(
            self.port,
            self.start_address,
            self.data_length,
            self.param,
            len(self.data_dict.keys()) * 1,
            False)

    def fastSyncReadTxPacket(self):
        if self.ph.getProtocolVersion() == 1.0 or len(self.data_dict.keys()) == 0:
            return COMM_NOT_AVAILABLE

        if self.is_param_changed is True or not self.param:
            self.makeParam()

        return self.ph.syncReadTx(
            self.port,
            self.start_address,
            self.data_length,
            self.param,
            len(self.data_dict.keys()) * 1,
            True)

    def rxPacket(self):
        self.last_result = False

        if self.ph.getProtocolVersion() == 1.0:
            return COMM_NOT_AVAILABLE

        result = COMM_RX_FAIL

        if len(self.data_dict.keys()) == 0:
            return COMM_NOT_AVAILABLE

        for dxl_id in self.data_dict:
            self.data_dict[dxl_id], result, _ = self.ph.readRx(self.port, dxl_id, self.data_length)
            if result != COMM_SUCCESS:
                return result

        if result == COMM_SUCCESS:
            self.last_result = True

        return result

    def fastSyncReadRxPacket(self):
        self.last_result = False

        if self.ph.getProtocolVersion() == 1.0:
            return COMM_NOT_AVAILABLE

        if not self.data_dict:
            return COMM_NOT_AVAILABLE

        num_devices = len(self.data_dict)
        rx_param_length = (self.data_length + 4) * num_devices  # 각 ID별 (Error(1) + ID(1) + Data(N) + CRC(2))
        
        # 데이터 수신
        raw_data, result, _ = self.ph.fastSyncReadRx(self.port, BROADCAST_ID, rx_param_length)
        if result != COMM_SUCCESS:
            return result

        print(f"[DEBUG] 실제 수신된 데이터 길이: {len(raw_data)}, 결과: {result}")
        print(f"[DEBUG] 수신된 전체 패킷: {raw_data}")

        # 바이트 배열로 변환 (빠른 인덱싱을 위해)
        raw_data = bytearray(raw_data)

        start_index = 0
        # for _ in range(num_devices):
        #     # 패킷 구조: Error(1) + ID(1) + Data(N) + CRC(2)
        #     error = raw_data[start_index]
        #     dxl_id = raw_data[start_index + 1]

        #     if dxl_id not in self.data_dict:
        #         print(f"[ERROR] Unexpected ID received: {dxl_id}")
        #         return COMM_RX_CORRUPT

        valid_ids = set(self.data_dict.keys())  # ✅ 미리 ID를 `set`으로 변환
        for _ in range(num_devices):
            dxl_id = raw_data[start_index + 1]
            if dxl_id not in valid_ids:
                print(f"[ERROR] Unexpected ID received: {dxl_id}")
                return COMM_RX_CORRUPT


            # 데이터 저장 (불필요한 리스트 연산 최소화)
            self.data_dict[dxl_id] = bytearray(raw_data[start_index + 2 : start_index + 2 + self.data_length])

            print(f"[DEBUG] ID {dxl_id} 데이터 저장됨: {self.data_dict[dxl_id]}")

            # 다음 데이터 위치 갱신 (Error(1) + ID(1) + Data(N) + CRC(2))
            start_index += self.data_length + 4

        self.last_result = True
        print("[DEBUG] Fast Sync Read 성공 완료")
        return COMM_SUCCESS


    # def txRxPacket(self):
    #     if self.ph.getProtocolVersion() == 1.0:
    #         return COMM_NOT_AVAILABLE

    #     result = self.txPacket()
    #     if result != COMM_SUCCESS:
    #         return result

    #     return self.rxPacket()

    def txRxPacket(self):
        if self.ph.getProtocolVersion() == 1.0:
            return COMM_NOT_AVAILABLE

        if (result := self.txPacket()) != COMM_SUCCESS:  # ✅ 단일 라인 최적화
            return result

        return self.rxPacket()  # ✅ 바로 호출하여 딜레이 최소화




    def fastSyncRead(self):
        if self.ph.getProtocolVersion() == 1.0:
            return COMM_NOT_AVAILABLE

        result = self.fastSyncReadTxPacket()
        if result != COMM_SUCCESS:
            return result

        return self.fastSyncReadRxPacket()

    def isAvailable(self, dxl_id, address, data_length):
        if self.ph.getProtocolVersion() == 1.0 or self.last_result is False or dxl_id not in self.data_dict:
            return False

        if (address < self.start_address) or (self.start_address + self.data_length - data_length < address):
            return False

        return True

    # def getData(self, dxl_id, address, data_length):
    #     if not self.isAvailable(dxl_id, address, data_length):
    #         return 0

    #     if data_length == 1:
    #         return self.data_dict[dxl_id][address - self.start_address]
    #     elif data_length == 2:
    #         return DXL_MAKEWORD(self.data_dict[dxl_id][address - self.start_address],
    #                             self.data_dict[dxl_id][address - self.start_address + 1])
    #     elif data_length == 4:
    #         return DXL_MAKEDWORD(DXL_MAKEWORD(self.data_dict[dxl_id][address - self.start_address + 0],
    #                                           self.data_dict[dxl_id][address - self.start_address + 1]),
    #                              DXL_MAKEWORD(self.data_dict[dxl_id][address - self.start_address + 2],
    #                                           self.data_dict[dxl_id][address - self.start_address + 3]))
    #     else:
    #         return 0

    def getData(self, dxl_id, address, data_length):
        if not self.isAvailable(dxl_id, address, data_length):
            return 0

        start_idx = address - self.start_address
        data = self.data_dict[dxl_id]

        if data_length == 1:
            return data[start_idx]
        elif data_length == 2:
            return (data[start_idx] | (data[start_idx + 1] << 8))  # ✅ 함수 호출 없이 연산 처리
        elif data_length == 4:
            return ((data[start_idx] | (data[start_idx + 1] << 8)) |
                    ((data[start_idx + 2] | (data[start_idx + 3] << 8)) << 16))  # ✅ 최적화된 4바이트 연산
        else:
            return 0



