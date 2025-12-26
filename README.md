# WT901C485 ROS 2 Driver (RS-485 Modbus IMU)

This repository contains a **ROS 2 package for setting up and using the WT901C485 RS-485 IMU in Modbus mode**.

The package provides **plug-and-play ROS 2 nodes** that poll the WT901C485 IMU over **RS-485 (Modbus RTU)** and publish **standard ROS sensor messages**, making it suitable for:

- robot_localization
- Nav2
- General mobile robotics, mapping, and localization applications

This driver follows the **industrial-correct Modbus polling model** required by the WT901C485 (no UART streaming).

---

## 🔌 Hardware Setup

### Required Hardware
- WT901C485 IMU (RS-485 variant)
- USB → RS-485 converter  
  (CH340 / FTDI based, auto-direction recommended)
- External **5V power supply** for the IMU

### Wiring

| WT901C485 | RS-485 Adapter |
|----------|---------------|
| A / A+ | A / A+ |
| B / B- | B / B- |
| GND | GND |
| VCC | External 5V |

⚠️ **Do NOT power the IMU from TTL VCC pins on the converter**  
⚠️ **Do NOT use UART / TTL mode — this IMU works only in RS-485 Modbus mode**

---

## 🖥️ Software Requirements

- Ubuntu 22.04
- ROS 2 Humble
- Python ≥ 3.10
- pyserial

Install the Python dependency:
```bash
pip3 install pyserial
```

---

## 📦 Installation & Setup

### Clone and install the package
```bash
cd ~/ros2_ws/src
git clone https://github.com/<your-username>/wt901c_imu.git
cd wt901c_imu
chmod +x install.sh
./install.sh
sudo reboot
```

### What the install script does
- Installs required system packages
- Installs USB–RS485 kernel modules (CH340 support)
- Removes conflicting services (brltty)
- Adds udev rules for stable device naming
- Adds the user to the dialout group

---

## 🔨 Build the Package

```bash
cd ~/ros2_ws
colcon build --packages-select wt901c_imu
source install/setup.bash
```

---

## 🚀 Running the Nodes

> ⚠️ **IMPORTANT NOTE**  
> Temperature readings are currently **NOT available** and will be supported in a future update.

### 1️⃣ Basic IMU Only
(Accelerometer + Gyroscope + Orientation)

```bash
ros2 run wt901c_imu wit_basic_imu_node \
  --ros-args \
  -p port:=/dev/ttyUSB0 \
  -p frame_id:=imu_link
```

---

### 2️⃣ IMU + Magnetometer
(Temperature placeholder – not working yet)

```bash
ros2 run wt901c_imu wit_imu_node_mag_tem \
  --ros-args \
  -p port:=/dev/ttyUSB0 \
  -p frame_id:=imu_link \
  -p rate:=20.0
```

---

### 3️⃣ Full IMU
(IMU + Magnetometer + Barometer)

```bash
ros2 run wt901c_imu full_imu_node \
  --ros-args \
  -p port:=/dev/ttyUSB0 \
  -p rate:=20.0 \
  -p mag_reg:=61 \
  -p baro_reg:=63
```

---

## 📡 Published Topics

| Topic | Message Type | Description |
|------|-------------|------------|
| /imu/data | sensor_msgs/Imu | Accelerometer, Gyroscope, Orientation |
| /imu/mag | sensor_msgs/MagneticField | Magnetometer (optional) |
| /imu/temperature | sensor_msgs/Temperature | Temperature (currently not working) |
| /imu/pressure | sensor_msgs/FluidPressure | Barometric pressure (optional) |

---

## ✨ Features

- RS-485 Modbus RTU polling (industrial-grade & reliable)
- Accelerometer
- Gyroscope
- Orientation (Euler → Quaternion)
- Magnetometer
- Barometer / Pressure
- ROS-standard message types
- Parameter-driven configuration
- Compatible with robot_localization and Nav2
- Works on any ROS 2 Humble system

---

## 📌 Important Notes

- WT901C485 does NOT support continuous UART streaming
- This driver correctly uses poll-based Modbus communication
- Register addresses may vary by firmware
- USB–RS485 adapters must support automatic TX/RX direction control

---

## 🆘 Support

For help, questions, or issues:

sdhudu@gmail.com

---

## 📄 License

MIT License
