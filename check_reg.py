import serial
import struct
import time

def crc16(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if (crc & 1) else (crc >> 1)
    return crc.to_bytes(2, 'little')

def req(slave, reg, cnt):
    f = struct.pack(">B B H H", slave, 0x03, reg, cnt)
    return f + crc16(f)

ser = serial.Serial("/dev/ttyUSB0", 9600, timeout=0.2)
SLAVE = 0x50

print("Scanning registers...")
for reg in range(0x30, 0x80):
    ser.write(req(SLAVE, reg, 1))
    time.sleep(0.01)
    r = ser.read(7)
    if len(r) == 7:
        print(f"Register 0x{reg:02X} -> {int.from_bytes(r[3:5],'big',signed=True)}")

ser.close()