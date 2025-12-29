#!/usr/bin/env python3
"""
WT901C485 ROS 2 IMU Driver (RS-485 / Modbus RTU)

Publishes:
  /imu/data        -> sensor_msgs/Imu
  /imu/mag         -> sensor_msgs/MagneticField
  /imu/temperature -> sensor_msgs/Temperature
  /imu/pressure    -> sensor_msgs/FluidPressure
  /imu/altitude    -> std_msgs/Float64 (derived from pressure)
"""

import time
import math
import struct
from typing import Optional

import serial
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu, MagneticField, Temperature, FluidPressure
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Float64


# ---------------- CRC16 ----------------
def crc16(data: bytes) -> bytes:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if (crc & 1) else (crc >> 1)
    return crc.to_bytes(2, "little")


class WT901CImu(Node):
    def __init__(self):
        super().__init__("wt901c_imu")

        # ---------------- Parameters ----------------
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baud", 9600)
        self.declare_parameter("rate", 10.0)
        self.declare_parameter("frame_id", "imu_link")
        self.declare_parameter("slave_id", 0x50)

        self.port = self.get_parameter("port").value
        self.baud = int(self.get_parameter("baud").value)
        self.rate = float(self.get_parameter("rate").value)
        self.frame_id = self.get_parameter("frame_id").value
        self.slave_id = int(self.get_parameter("slave_id").value)

        # ---------------- Registers (verified) ----------------
        self.acc_reg = 0x34
        self.gyro_reg = 0x37
        self.angle_reg = 0x3A
        self.mag_reg = 0x3D
        self.temp_reg = 0x40
        self.baro_high_reg = 0x3F
        self.baro_low_reg = 0x63

        # ---------------- Scales ----------------
        self.acc_scale_g = 16.0
        self.gyro_scale_dps = 2000.0
        self.angle_scale_deg = 180.0
        self.mag_scale = 1e-6        # Tesla
        self.temp_scale = 0.01       # °C
        self.baro_scale = 0.1        # Pa (correct for your unit)

        # ---------------- Serial ----------------
        self.ser = serial.Serial(self.port, self.baud, timeout=0.5)

        # ---------------- Publishers ----------------
        self.pub_imu = self.create_publisher(Imu, "imu/data", 10)
        self.pub_mag = self.create_publisher(MagneticField, "imu/mag", 10)
        self.pub_temp = self.create_publisher(Temperature, "imu/temperature", 10)
        self.pub_pressure = self.create_publisher(FluidPressure, "imu/pressure", 10)
        self.pub_altitude = self.create_publisher(Float64, "imu/altitude", 10)

        self.timer = self.create_timer(1.0 / self.rate, self.update)

        self.get_logger().info(
            f"WT901C485 running on {self.port} @ {self.baud} baud"
        )

    # ---------------- Modbus helpers ----------------
    def build_request(self, reg: int, count: int) -> bytes:
        frame = struct.pack(">B B H H", self.slave_id, 0x03, reg, count)
        return frame + crc16(frame)

    def read_registers(self, reg: int, count: int) -> Optional[list]:
        self.ser.reset_input_buffer()
        self.ser.write(self.build_request(reg, count))
        time.sleep(0.01)

        resp = self.ser.read(5 + count * 2)
        if len(resp) != (5 + count * 2):
            return None
        if resp[-2:] != crc16(resp[:-2]):
            return None

        payload = resp[3:-2]
        return [
            struct.unpack(">h", payload[i:i + 2])[0]
            for i in range(0, len(payload), 2)
        ]

    def read_pressure_raw_u32(self) -> Optional[int]:
        hi = self.read_registers(self.baro_high_reg, 1)
        lo = self.read_registers(self.baro_low_reg, 1)
        if hi is None or lo is None:
            return None
        return ((hi[0] & 0xFFFF) << 16) | (lo[0] & 0xFFFF)

    # ---------------- Math ----------------
    def euler_to_quaternion(self, r, p, y) -> Quaternion:
        cy, sy = math.cos(y / 2), math.sin(y / 2)
        cp, sp = math.cos(p / 2), math.sin(p / 2)
        cr, sr = math.cos(r / 2), math.sin(r / 2)
        q = Quaternion()
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy
        return q

    # ---------------- Main loop ----------------
    def update(self):
        acc = self.read_registers(self.acc_reg, 3)
        gyro = self.read_registers(self.gyro_reg, 3)
        ang = self.read_registers(self.angle_reg, 3)

        if not acc or not gyro or not ang:
            return

        ax, ay, az = [v / 32768.0 * self.acc_scale_g * 9.80665 for v in acc]
        gx, gy, gz = [math.radians(v / 32768.0 * self.gyro_scale_dps) for v in gyro]
        roll, pitch, yaw = [math.radians(v / 32768.0 * self.angle_scale_deg) for v in ang]

        imu = Imu()
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = self.frame_id
        imu.linear_acceleration.x = ax
        imu.linear_acceleration.y = ay
        imu.linear_acceleration.z = az
        imu.angular_velocity.x = gx
        imu.angular_velocity.y = gy
        imu.angular_velocity.z = gz
        imu.orientation = self.euler_to_quaternion(roll, pitch, yaw)
        self.pub_imu.publish(imu)

        # Magnetometer
        mag = self.read_registers(self.mag_reg, 3)
        if mag:
            m = MagneticField()
            m.header = imu.header
            m.magnetic_field.x = mag[0] * self.mag_scale
            m.magnetic_field.y = mag[1] * self.mag_scale
            m.magnetic_field.z = mag[2] * self.mag_scale
            self.pub_mag.publish(m)

        # Temperature
        temp = self.read_registers(self.temp_reg, 1)
        if temp:
            t = Temperature()
            t.header = imu.header
            t.temperature = temp[0] * self.temp_scale
            self.pub_temp.publish(t)

        # Pressure + Altitude
        raw_p = self.read_pressure_raw_u32()
        if raw_p is not None:
            pressure_pa = raw_p * self.baro_scale

            pmsg = FluidPressure()
            pmsg.header = imu.header
            pmsg.fluid_pressure = pressure_pa
            self.pub_pressure.publish(pmsg)

            ratio = pressure_pa / 101325.0 if pressure_pa > 0 else 0.0
            altitude = 44330.0 * (1.0 - pow(ratio, 0.1903)) if ratio > 0 else 0.0

            amsg = Float64()
            amsg.data = altitude
            self.pub_altitude.publish(amsg)

    def destroy_node(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WT901CImu()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
