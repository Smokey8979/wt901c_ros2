#!/usr/bin/env python3
"""
WT901C485 final ROS2 node — quaternion-based, calibration + offsets, sensor->robot offset.

Publishes:
  /imu/data        -> sensor_msgs/Imu
  /imu/mag         -> sensor_msgs/MagneticField
  /imu/temperature -> sensor_msgs/Temperature
  /imu/pressure    -> sensor_msgs/FluidPressure
  /imu/altitude    -> std_msgs/Float64

Defaults assume IMU axes: +X forward, +Y left, +Z up (standard ROS mobile robot).
If your IMU is rotated relative to the robot, set `orientation_offset` (roll,pitch,yaw in degrees).

remove gravity from linear_acceleration is optional.
"""

import time
import math
import struct
from typing import Optional, Tuple, List

import serial
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu, MagneticField, Temperature, FluidPressure
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Float64

GRAVITY = 9.80665


# ===== CRC16 (Modbus) =====
def crc16(data: bytes) -> bytes:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if (crc & 1) else (crc >> 1)
    return crc.to_bytes(2, "little")


# ===== quaternion helpers (tuples) =====
def quat_mul(a: Tuple[float, float, float, float], b: Tuple[float, float, float, float]) -> Tuple[float, float, float, float]:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw*bw - ax*bx - ay*by - az*bz,
        aw*bx + ax*bw + ay*bz - az*by,
        aw*by - ax*bz + ay*bw + az*bx,
        aw*bz + ax*by - ay*bx + az*bw,
    )


def quat_conjugate(q: Tuple[float, float, float, float]) -> Tuple[float, float, float, float]:
    w, x, y, z = q
    return (w, -x, -y, -z)


def rotate_vector_by_quat(v: Tuple[float, float, float], q: Tuple[float, float, float, float]) -> Tuple[float, float, float]:
    # v' = q * (0,v) * q_conj
    vq = (0.0, v[0], v[1], v[2])
    tmp = quat_mul(q, vq)
    res = quat_mul(tmp, quat_conjugate(q))
    return (res[1], res[2], res[3])


def quaternion_from_euler(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return (w, x, y, z)


def normalize_quat_tuple(q: Tuple[float, float, float, float]) -> Tuple[float, float, float, float]:
    w, x, y, z = q
    n = math.sqrt(w*w + x*x + y*y + z*z)
    if n == 0:
        return (1.0, 0.0, 0.0, 0.0)
    return (w/n, x/n, y/n, z/n)


# ===== Node =====
class WT901CFinal(Node):
    def __init__(self):
        super().__init__("wt901c_final")

        # --- parameters ---
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baud", 9600)
        self.declare_parameter("rate", 20.0)
        self.declare_parameter("frame_id", "imu_link")
        self.declare_parameter("slave_id", 0x50)

        # calibration & offsets usage
        self.declare_parameter("calibrate_on_start", True)
        self.declare_parameter("cal_mode", 0x01)
        self.declare_parameter("cal_time_s", 3.5)
        self.declare_parameter("save_after_cal", True)

        # remove gravity from linear_acceleration (False by default)
        self.declare_parameter("remove_gravity", True)

        # orientation offset (sensor->robot) in degrees [roll, pitch, yaw]
        # default [0,0,0] because you told me X=front, Y=left, Z=up
        self.declare_parameter("orientation_offset", [0.0, 0.0, 0.0])

        # scales & register defaults based on your register map
        self.declare_parameter("acc_scale_g", 16.0)
        self.declare_parameter("gyro_scale_dps", 2000.0)
        self.declare_parameter("angle_scale_deg", 180.0)
        self.declare_parameter("mag_scale", 1e-6)
        self.declare_parameter("temp_scale", 0.01)
        self.declare_parameter("baro_scale", 0.01)  # adjust if your pressure is off

        # read params
        self.port = self.get_parameter("port").value
        self.baud = int(self.get_parameter("baud").value)
        self.rate = float(self.get_parameter("rate").value)
        self.frame_id = self.get_parameter("frame_id").value
        self.slave_id = int(self.get_parameter("slave_id").value)

        self.calibrate_on_start = bool(self.get_parameter("calibrate_on_start").value)
        self.cal_mode = int(self.get_parameter("cal_mode").value)
        self.cal_time_s = float(self.get_parameter("cal_time_s").value)
        self.save_after_cal = bool(self.get_parameter("save_after_cal").value)

        self.remove_gravity = bool(self.get_parameter("remove_gravity").value)
        off_deg = list(self.get_parameter("orientation_offset").value)
        self.orientation_offset_q = quaternion_from_euler(math.radians(off_deg[0]), math.radians(off_deg[1]), math.radians(off_deg[2]))

        self.acc_scale_g = float(self.get_parameter("acc_scale_g").value)
        self.gyro_scale_dps = float(self.get_parameter("gyro_scale_dps").value)
        self.angle_scale_deg = float(self.get_parameter("angle_scale_deg").value)
        self.mag_scale = float(self.get_parameter("mag_scale").value)
        self.temp_scale = float(self.get_parameter("temp_scale").value)
        self.baro_scale = float(self.get_parameter("baro_scale").value)

        # --- register addresses (per your table) ---
        self.REG_SAVE = 0x00
        self.REG_CALSW = 0x01
        self.REG_AXOFFSET = 0x05
        # ... offsets consecutive to 0x0D
        self.acc_reg = 0x34     # AX
        self.gyro_reg = 0x37    # GX
        self.mag_reg = 0x3a     # HX in table at 0x3a..0x3c (but some units place mag elsewhere)
        self.angle_reg = 0x3d   # roll@0x3d, pitch@0x3e, yaw@0x3f (signed)
        self.temp_reg = 0x40    # temperature
        # pressure and height (low/high bytes per table)
        self.pressure_lo = 0x45
        self.pressure_hi = 0x46
        self.height_lo = 0x47
        self.height_hi = 0x48
        # quaternion registers (most robust) per the table
        self.q0_reg = 0x51
        self.q1_reg = 0x52
        self.q2_reg = 0x53
        self.q3_reg = 0x54

        # offsets (populated after read)
        self.ax_offset = self.ay_offset = self.az_offset = 0
        self.gx_offset = self.gy_offset = self.gz_offset = 0
        self.hx_offset = self.hy_offset = self.hz_offset = 0

        # serial
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.5)
        except Exception as e:
            self.get_logger().error(f"Failed to open serial {self.port}: {e}")
            raise

        # publishers
        self.pub_imu = self.create_publisher(Imu, "imu/data", 10)
        self.pub_mag = self.create_publisher(MagneticField, "imu/mag", 5)
        self.pub_temp = self.create_publisher(Temperature, "imu/temperature", 2)
        self.pub_pressure = self.create_publisher(FluidPressure, "imu/pressure", 2)
        self.pub_altitude = self.create_publisher(Float64, "imu/altitude", 2)

        # timer
        period = 1.0 / max(1.0, self.rate)
        self.timer = self.create_timer(period, self.update)

        self.get_logger().info(f"WT901 final: port={self.port} baud={self.baud} rate={self.rate}")

        # optionally do calibration sequence and read offsets
        if self.calibrate_on_start:
            self.get_logger().info("Starting calibration via CALSW (register 0x01)")
            if self.write_register(self.REG_CALSW, self.cal_mode):
                self.get_logger().info(f"CALSW set to 0x{self.cal_mode:02X}, waiting {self.cal_time_s}s")
                time.sleep(self.cal_time_s)
                self.write_register(self.REG_CALSW, 0)
                self.get_logger().info("CALSW cleared to 0")
                if self.save_after_cal:
                    self.write_register(self.REG_SAVE, 0)
                    self.get_logger().info("Saved calibration to device (REG 0x00)")
            else:
                self.get_logger().warning("Failed to set CALSW; skipping calibrate_on_start")

        self._read_offsets()

    # ===== Modbus helpers =====
    def build_read_request(self, reg: int, count: int) -> bytes:
        frame = struct.pack(">B B H H", self.slave_id, 0x03, reg, count)
        return frame + crc16(frame)

    def build_write_request(self, reg: int, value: int) -> bytes:
        frame = struct.pack(">B B H H", self.slave_id, 0x06, reg, value & 0xFFFF)
        return frame + crc16(frame)

    def read_registers(self, reg: int, count: int = 1, retries: int = 2) -> Optional[List[int]]:
        expected = 5 + count * 2
        for _ in range(retries):
            try:
                self.ser.reset_input_buffer()
                self.ser.write(self.build_read_request(reg, count))
                time.sleep(0.02)
                resp = self.ser.read(expected)
                if len(resp) < expected:
                    resp += self.ser.read(expected - len(resp))
                if len(resp) != expected:
                    continue
                if resp[-2:] != crc16(resp[:-2]):
                    self.get_logger().debug("CRC mismatch on read")
                    continue
                payload = resp[3:-2]
                return [struct.unpack(">h", payload[i:i+2])[0] for i in range(0, len(payload), 2)]
            except Exception as e:
                self.get_logger().debug(f"read_registers exception: {e}")
                time.sleep(0.01)
        return None

    def write_register(self, reg: int, value: int, retries: int = 2) -> bool:
        for _ in range(retries):
            try:
                self.ser.reset_input_buffer()
                self.ser.write(self.build_write_request(reg, value))
                time.sleep(0.02)
                resp = self.ser.read(8)
                if len(resp) < 8:
                    resp += self.ser.read(8 - len(resp))
                if len(resp) != 8:
                    continue
                if resp[-2:] != crc16(resp[:-2]):
                    continue
                return True
            except Exception:
                time.sleep(0.01)
        return False

    def read_signed32_pair(self, reg_hi:int, reg_lo:int) -> Optional[int]:
        hi = self.read_registers(reg_hi, 1)
        lo = self.read_registers(reg_lo, 1)
        if hi is None or lo is None:
            return None
        hi_u = hi[0] & 0xFFFF
        lo_u = lo[0] & 0xFFFF
        raw_u = (hi_u << 16) | lo_u
        if raw_u & 0x80000000:
            return raw_u - 0x100000000
        return raw_u

    # ===== Offsets =====
    def _read_offsets(self):
        regs = self.read_registers(self.REG_AXOFFSET, 9)
        if regs and len(regs) >= 9:
            self.ax_offset, self.ay_offset, self.az_offset = regs[0], regs[1], regs[2]
            self.gx_offset, self.gy_offset, self.gz_offset = regs[3], regs[4], regs[5]
            self.hx_offset, self.hy_offset, self.hz_offset = regs[6], regs[7], regs[8]
            self.get_logger().info(f"Offsets read AX,AY,AZ = {self.ax_offset},{self.ay_offset},{self.az_offset}")
        else:
            self.get_logger().info("No hw offsets read; using zeros")

    # ===== main update loop =====
    def update(self):
        # read accel/gyro/angle
        acc_regs = self.read_registers(self.acc_reg, 3)
        gyro_regs = self.read_registers(self.gyro_reg, 3)
        # try quaternion registers first (preferred)
        q_regs = self.read_registers(self.q0_reg, 4)

        if acc_regs is None or gyro_regs is None:
            # not enough data
            return

        # apply offsets (raw counts)
        ax_raw = acc_regs[0] - self.ax_offset
        ay_raw = acc_regs[1] - self.ay_offset
        az_raw = acc_regs[2] - self.az_offset

        gx_raw = gyro_regs[0] - self.gx_offset
        gy_raw = gyro_regs[1] - self.gy_offset
        gz_raw = gyro_regs[2] - self.gz_offset

        # scale to SI
        ax = ax_raw / 32768.0 * self.acc_scale_g * GRAVITY
        ay = ay_raw / 32768.0 * self.acc_scale_g * GRAVITY
        az = az_raw / 32768.0 * self.acc_scale_g * GRAVITY

        gx = math.radians(gx_raw / 32768.0 * self.gyro_scale_dps)
        gy = math.radians(gy_raw / 32768.0 * self.gyro_scale_dps)
        gz = math.radians(gz_raw / 32768.0 * self.gyro_scale_dps)

        # orientation: prefer quaternion registers if valid
        if q_regs and len(q_regs) >= 4:
            # device likely stores Q0..Q3 as signed int16 scaled by 32768
            qw = q_regs[0] / 32768.0
            qx = q_regs[1] / 32768.0
            qy = q_regs[2] / 32768.0
            qz = q_regs[3] / 32768.0
            q_raw = normalize_quat_tuple((qw, qx, qy, qz))
        else:
            # fallback to Euler angles (registers 0x3d..0x3f contain Roll/Pitch/Yaw in degrees scaled)
            ang_regs = self.read_registers(self.angle_reg, 3)
            if ang_regs is None:
                return
            roll = math.radians(ang_regs[0] / 32768.0 * self.angle_scale_deg)
            pitch = math.radians(ang_regs[1] / 32768.0 * self.angle_scale_deg)
            yaw = math.radians(ang_regs[2] / 32768.0 * self.angle_scale_deg)
            q_raw = quaternion_from_euler(roll, pitch, yaw)

        # apply orientation offset (sensor->robot) by quaternion multiplication:
        # q_robot = q_offset * q_raw
        q_offset = self.orientation_offset_q
        q_robot = quat_mul(q_offset, q_raw)
        q_robot = normalize_quat_tuple(q_robot)

        # rotate accel & gyro into robot frame by same offset (so vectors align)
        accel_robot = rotate_vector_by_quat((ax, ay, az), q_offset)
        gyro_robot = rotate_vector_by_quat((gx, gy, gz), q_offset)

        # optionally remove gravity component from linear acceleration
        if self.remove_gravity:
            gvec = rotate_vector_by_quat((0.0, 0.0, GRAVITY), q_robot)  # gravity in robot frame
            accel_robot = (accel_robot[0] - gvec[0], accel_robot[1] - gvec[1], accel_robot[2] - gvec[2])

        # build IMU msg
        imu = Imu()
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = self.frame_id
        imu.linear_acceleration.x = float(accel_robot[0])
        imu.linear_acceleration.y = float(accel_robot[1])
        imu.linear_acceleration.z = float(accel_robot[2])
        imu.angular_velocity.x = float(gyro_robot[0])
        imu.angular_velocity.y = float(gyro_robot[1])
        imu.angular_velocity.z = float(gyro_robot[2])
        imu.orientation.w = float(q_robot[0])
        imu.orientation.x = float(q_robot[1])
        imu.orientation.y = float(q_robot[2])
        imu.orientation.z = float(q_robot[3])
        self.pub_imu.publish(imu)

        # magnetometer (best-effort)
        mag_regs = self.read_registers(self.mag_reg, 3)
        if mag_regs:
            mx = (mag_regs[0] - self.hx_offset) * self.mag_scale
            my = (mag_regs[1] - self.hy_offset) * self.mag_scale
            mz = (mag_regs[2] - self.hz_offset) * self.mag_scale
            m_robot = rotate_vector_by_quat((mx, my, mz), q_offset)
            mm = MagneticField()
            mm.header = imu.header
            mm.magnetic_field.x = float(m_robot[0])
            mm.magnetic_field.y = float(m_robot[1])
            mm.magnetic_field.z = float(m_robot[2])
            self.pub_mag.publish(mm)

        # temperature
        t_regs = self.read_registers(self.temp_reg, 1)
        if t_regs:
            t = Temperature()
            t.header = imu.header
            t.temperature = float(t_regs[0] * self.temp_scale)
            self.pub_temp.publish(t)

        # pressure / altitude
        pres_raw = self.read_signed32_pair(self.pressure_hi, self.pressure_lo)
        if pres_raw is not None:
            pressure_pa = float(pres_raw) * self.baro_scale
            pmsg = FluidPressure()
            pmsg.header = imu.header
            pmsg.fluid_pressure = pressure_pa
            pmsg.variance = 0.0
            self.pub_pressure.publish(pmsg)
            # altitude estimate (barometric formula)
            try:
                altitude = 44330.0 * (1.0 - (pressure_pa / 101325.0) ** 0.1903)
            except Exception:
                altitude = 0.0
            am = Float64()
            am.data = float(altitude)
            self.pub_altitude.publish(am)

    def destroy_node(self):
        try:
            if hasattr(self, "ser") and self.ser and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WT901CFinal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
