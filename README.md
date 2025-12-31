# WT901C485 ROS 2 Driver (RS-485 Modbus IMU)

This repository provides a **ROS 2 driver for the WT901C485 RS-485 IMU operating in Modbus RTU mode**.

The package offers **plug-and-play ROS 2 nodes** that communicate with the WT901C485 over **RS-485 (Modbus RTU)** and publish **standard ROS sensor messages**, making it suitable for:

- `robot_localization`
- Nav2
- Mobile robots
- Mapping, localization, and sensor fusion pipelines

This driver follows the **industrial-correct Modbus polling model** required by the WT901C485.

⚠️ **Important**  
The WT901C485 **does NOT support UART streaming**.  
All data must be acquired using **Modbus RTU polling**.

---

## 🔌 Hardware Setup

### Required Hardware
- WT901C485 IMU (RS-485 variant)
- USB → RS-485 converter  
  (CH340 / FTDI based, **auto-direction control recommended**)
- External **5V power supply** for the IMU

### Wiring

| WT901C485 | RS-485 Adapter |
|----------|----------------|
| A / A+   | A / A+         |
| B / B−   | B / B−         |
| GND      | GND            |
| VCC      | External 5V    |

⚠️ **Do NOT power the IMU from TTL VCC pins on the converter**  
⚠️ **Do NOT use UART / TTL mode — this IMU works only in RS-485 Modbus mode**

---

## 🖥️ Software Requirements

- Ubuntu 22.04 or Ubuntu 24.04
- ROS 2 Humble or ROS 2 Jazzy
- Python ≥ 3.10
- `pyserial`

---

## 📦 Installation & Setup

### Clone and Install

```bash
cd ~/ros2_ws/src
git clone https://github.com/Smokey8979/wt901c_ros2.git
cd wt901c_ros2
chmod +x install.sh
./install.sh
sudo reboot
```

### What the Install Script Does
- Installs required system dependencies
- Enables USB–RS485 kernel modules (CH340 support)
- Removes conflicting services (`brltty`)
- Adds udev rules for stable device naming
- Adds the user to the `dialout` group
- Installs the IMU plugin for `rviz2`

---

## 🔨 Build the Package

```bash
cd ~/ros2_ws
colcon build --packages-select wt901c_imu
source install/setup.bash
```

---

## 🚀 Running the Nodes

> ⚠️ **Important Notes**
>
> - Temperature data is **available**
> - Altitude data is **currently inaccurate** (derived from pressure)
> - Altitude improvements are planned in future releases

---

### 1️⃣ Basic IMU  
**Accelerometer + Gyroscope + Orientation**

```bash
ros2 run wt901c_imu wit_basic_imu_node \
  --ros-args \
  -p port:=/dev/ttyUSB0 \
  -p frame_id:=imu_link
```

---

### 2️⃣ IMU + Magnetometer

```bash
ros2 run wt901c_imu wit_imu_node_mag \
  --ros-args \
  -p port:=/dev/ttyUSB0 \
  -p frame_id:=imu_link \
  -p rate:=20.0
```

Temperature is currently a placeholder in this node.

---

### 3️⃣ Test / Diagnostic Node  
**IMU + Magnetometer + Barometer**

```bash
ros2 run wt901c_imu test_imu_node \
  --ros-args \
  -p port:=/dev/ttyUSB0 \
  -p rate:=20.0 \
  -p mag_reg:=61 \
  -p baro_reg:=63
```

Alternative:

```bash
ros2 run wt901c_imu wit_imu_node
```

---

### 4️⃣ Full IMU (Robot Deployment & Calibration)

This node is intended for **real-robot integration** and supports **on-device IMU calibration**.

> ⚠️ **Important Notes**
>
> - Altitude is derived from pressure and is currently inaccurate
> - Allow the IMU to complete calibration after startup
> - Gravity compensation can be optionally disabled
> - Recommended for EKF / real-world navigation setups

```bash
ros2 run wt901c_imu wit_imu_with_cal_node
```

---

## 🔍 Verification & Visualization

### Check Topics
```bash
ros2 topic list
```

### Visualize in RViz2
```bash
rviz2
```

Set the fixed frame to `imu_link` and add IMU-related topics.

---

## 🧪 Debug & Diagnostics

### Scan IMU Registers (Low-Level Debug)

Use this only if IMU data is not publishing correctly.

```bash
cd ~/ros2_ws/src/wt901c_ros2/
chmod +x check_reg.py
python3 check_reg.py
```

### Official Register Documentation

WT901C485 register map and functions:  
https://images-na.ssl-images-amazon.com/images/I/A1YLf4otHWL.pdf

---

## 📡 Published Topics

| Topic | Message Type | Description |
|------|-------------|------------|
| `/imu/data` | `sensor_msgs/Imu` | Acceleration, angular velocity, orientation |
| `/imu/mag` | `sensor_msgs/MagneticField` | Magnetometer |
| `/imu/temperature` | `sensor_msgs/Temperature` | Temperature |
| `/imu/pressure` | `sensor_msgs/FluidPressure` | Barometric pressure |
| `/imu/altitude` | `std_msgs/Float64` | Altitude (derived, inaccurate) |

---

## ✨ Features

- Industrial-grade RS-485 Modbus RTU polling
- Accelerometer, gyroscope, magnetometer support
- Orientation (Euler → Quaternion)
- Barometer / pressure data
- ROS-standard message interfaces
- Parameter-driven configuration
- Compatible with `robot_localization` and Nav2
- Works on ROS 2 Humble and Jazzy
- Designed for real-robot deployment
- Supports EKF fusion with wheel odometry and other sensors

---

## 📌 Important Notes

- WT901C485 does not support UART streaming
- Modbus polling is mandatory
- Register addresses may vary by firmware version
- USB–RS485 adapters must support automatic TX/RX direction switching

---

## 🆘 Support

For questions, issues, or contributions:

📧 **sdhudu@gmail.com**

---

## 📄 License

MIT License