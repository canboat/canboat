#!/bin/python3

import sys
import struct

def errprint(*args, **kwargs):
    print(*args, file=sys.stderr, **kwargs)

for fileName in sys.argv[1:]:
    with open(fileName, mode='rb') as file: 
        fileContent = file.read()
        # print struct.unpack("i" * ((len(fileContent) -24) // 4), fileContent[20:-4])
        magic, = struct.unpack("<I", fileContent[0:4])
        if magic != 0xa1b2c3d4:
            errprint("File", fileName + ": invalid magic")
            exit(1)
        offset = 0x18
        while (offset < len(fileContent) - 0x18):
            timestamp, = struct.unpack_from("<L", fileContent, offset)
            nano, = struct.unpack_from("<I", fileContent, offset + 4)
            canid, = struct.unpack_from(">I", fileContent, offset + 16)
            datalen, = struct.unpack_from("B", fileContent, offset + 20)
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
            offset += 24 + datalen



