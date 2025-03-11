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

PARAM_NUM_DATA = 0
PARAM_NUM_ADDRESS = 1
PARAM_NUM_LENGTH = 2


class GroupBulkRead:
    def __init__(self, port, ph):
        self.port = port
        self.ph = ph

        self.last_result = False
        self.is_param_changed = False
        self.param = []
        self.data_dict = {}

        self.clearParam()

    def makeParam(self):
        if not self.data_dict:
            return

        self.param = []

        for dxl_id in self.data_dict:
            if self.ph.getProtocolVersion() == 1.0:
                self.param.append(self.data_dict[dxl_id][2])  # LEN
                self.param.append(dxl_id)  # ID
                self.param.append(self.data_dict[dxl_id][1])  # ADDR
            else:
                self.param.append(dxl_id)  # ID
                self.param.append(DXL_LOBYTE(self.data_dict[dxl_id][1]))  # ADDR_L
                self.param.append(DXL_HIBYTE(self.data_dict[dxl_id][1]))  # ADDR_H
                self.param.append(DXL_LOBYTE(self.data_dict[dxl_id][2]))  # LEN_L
                self.param.append(DXL_HIBYTE(self.data_dict[dxl_id][2]))  # LEN_H

    def addParam(self, dxl_id, start_address, data_length):
        if dxl_id in self.data_dict:  # dxl_id already exist
            return False

        data = []  # [0] * data_length
        self.data_dict[dxl_id] = [data, start_address, data_length]

        self.is_param_changed = True
        return True

    def removeParam(self, dxl_id):
        if dxl_id not in self.data_dict:  # NOT exist
            return

        del self.data_dict[dxl_id]

        self.is_param_changed = True

    def clearParam(self):
        self.data_dict.clear()
        return

    def txPacket(self):
        if len(self.data_dict.keys()) == 0:
            return COMM_NOT_AVAILABLE

        if self.is_param_changed is True or not self.param:
            self.makeParam()

        if self.ph.getProtocolVersion() == 1.0:
            return self.ph.bulkReadTx(self.port, self.param, len(self.data_dict.keys()) * 3, False)
        else:
            return self.ph.bulkReadTx(self.port, self.param, len(self.data_dict.keys()) * 5, False)

    def fastBulkReadTxPacket(self):
        if self.ph.getProtocolVersion() == 1.0 or len(self.data_dict.keys()) == 0:
            return COMM_NOT_AVAILABLE

        if self.is_param_changed is True or not self.param:
            self.makeParam()

        return self.ph.bulkReadTx(self.port, self.param, len(self.data_dict.keys()) * 5, True)

    def rxPacket(self):
        self.last_result = False

        result = COMM_RX_FAIL

        if len(self.data_dict.keys()) == 0:
            return COMM_NOT_AVAILABLE

        for dxl_id in self.data_dict:
            self.data_dict[dxl_id][PARAM_NUM_DATA], result, _ = self.ph.readRx(self.port, dxl_id,
                                                                               self.data_dict[dxl_id][PARAM_NUM_LENGTH])
            if result != COMM_SUCCESS:
                return result

        if result == COMM_SUCCESS:
            self.last_result = True

        return result

    def fastBulkReadRxPacket(self):
        self.last_result = False

        if self.ph.getProtocolVersion() == 1.0:
            return COMM_NOT_AVAILABLE

        # 예상 수신 패킷 길이 계산 (각 ID별 Error(1) + ID(1) + DATA(N) + CRC(2))
        rx_length = sum(self.data_dict[dxl_id][PARAM_NUM_LENGTH] + 4 for dxl_id in self.data_dict)

        # 패킷 수신 (Broadcast 응답이므로 True)
        rxpacket, result = self.ph.rxPacket(self.port, True)
        if result != COMM_SUCCESS:
            print(f"[ERROR] RX Packet failed: {result}")
            return result

        index = 8
        packet_end = len(rxpacket) - 2  # CRC 2bytes 제외

        while index < packet_end:
            if index + 2 > packet_end:
                print(f"[ERROR] Incomplete packet data at index {index}")
                return COMM_RX_CORRUPT

            error = rxpacket[index]
            dxl_id = rxpacket[index + 1]

            if dxl_id not in self.data_dict:
                print(f"[ERROR] Unexpected ID received: {dxl_id}")
                return COMM_RX_CORRUPT

            data_length = self.data_dict[dxl_id][PARAM_NUM_LENGTH]

            if index + 2 + data_length > packet_end:
                print(f"[ERROR] Data length mismatch for ID {dxl_id}")
                return COMM_RX_CORRUPT

            # 받은 데이터를 정확히 저장
            self.data_dict[dxl_id][PARAM_NUM_DATA] = rxpacket[index + 2 : index + 2 + data_length]

            # 다음 데이터 세그먼트로 이동 (Error(1) + ID(1) + Data(N) + CRC(2))
            index += data_length + 4

        self.last_result = True
        return COMM_SUCCESS




    def txRxPacket(self):
        result = self.txPacket()
        if result != COMM_SUCCESS:
            return result

        return self.rxPacket()

    def fastBulkRead(self):
        if self.ph.getProtocolVersion() == 1.0:
            return COMM_NOT_AVAILABLE
        
        result = self.fastBulkReadTxPacket()
        if result != COMM_SUCCESS:
            return result
        
        return self.fastBulkReadRxPacket()

    def isAvailable(self, dxl_id, address, data_length):
        if self.last_result is False or dxl_id not in self.data_dict:
            return False

        start_addr = self.data_dict[dxl_id][PARAM_NUM_ADDRESS]

        if (address < start_addr) or (start_addr + self.data_dict[dxl_id][PARAM_NUM_LENGTH] - data_length < address):
            return False

        return True

    def getData(self, dxl_id, address, data_length):
        if not self.isAvailable(dxl_id, address, data_length):
            return 0

        start_addr = self.data_dict[dxl_id][PARAM_NUM_ADDRESS]

        if data_length == 1:
            return self.data_dict[dxl_id][PARAM_NUM_DATA][address - start_addr]
        elif data_length == 2:
            return DXL_MAKEWORD(self.data_dict[dxl_id][PARAM_NUM_DATA][address - start_addr],
                                self.data_dict[dxl_id][PARAM_NUM_DATA][address - start_addr + 1])
        elif data_length == 4:
            return DXL_MAKEDWORD(DXL_MAKEWORD(self.data_dict[dxl_id][PARAM_NUM_DATA][address - start_addr + 0],
                                              self.data_dict[dxl_id][PARAM_NUM_DATA][address - start_addr + 1]),
                                 DXL_MAKEWORD(self.data_dict[dxl_id][PARAM_NUM_DATA][address - start_addr + 2],
                                              self.data_dict[dxl_id][PARAM_NUM_DATA][address - start_addr + 3]))
        else:
            return 0
