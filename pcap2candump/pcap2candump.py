#!/bin/python3
#
# (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.
#
# This file is part of CANboat.
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
#

import sys
import struct

def errprint(*args, **kwargs):
    print(*args, file=sys.stderr, **kwargs)

for fileName in sys.argv[1:]:
    with open(fileName, mode='rb') as file: 
        fileContent = file.read()
        # print struct.unpack("i" * ((len(fileContent) -24) // 4), fileContent[20:-4])
        magic, = struct.unpack("<I", fileContent[0:4])
        byte_order = '<'
        if magic == 0xd4c3b2a1:
            byte_order = '>'
        elif magic != 0xa1b2c3d4:
            errprint("File", fileName + ": invalid magic")
            exit(1)
        offset = 0x18
        # A record needs at least its 16-byte header and the 8-byte SocketCAN
        # header; the captured length check below bounds the rest.
        while offset + 16 + 8 <= len(fileContent):
            timestamp, = struct.unpack_from(byte_order + "L", fileContent, offset)
            nano, = struct.unpack_from(byte_order + "I", fileContent, offset + 4)
            captured_length, = struct.unpack_from(byte_order + "I", fileContent, offset + 8)
            if captured_length < 8 or offset + 16 + captured_length > len(fileContent):
                break  # truncated or not a SocketCAN record
            canid, = struct.unpack_from(">I", fileContent, offset + 16)
            datalen, = struct.unpack_from("B", fileContent, offset + 20)
            # Only the captured bytes belong to this record; a short snaplen
            # cuts the payload below the DLC.
            datalen = min(datalen, captured_length - 8, 8)
            b1, = struct.unpack_from("B", fileContent, offset + 21)
            b2, = struct.unpack_from("B", fileContent, offset + 22)
            b3, = struct.unpack_from("B", fileContent, offset + 23)
            data = struct.unpack_from("B" * datalen, fileContent, offset + 24)

            #
            # Print in candump format 3:
            #
            # (1502984883.726457) slcan0 09F119CC#FF765ACBF7FA04FF
            #
            print(f"({timestamp}.{nano:06d}) slcan0 {canid:x}#", end = "")
            for b in data:
                print(f"{b:02x}", end = "")
            print()
            # A SocketCAN record includes padding up to CAN_MTU even when
            # the CAN payload has fewer than eight bytes. Advance by the
            # PCAP captured length, not by the CAN DLC.
            offset += 16 + captured_length



