# Security Policy

## Supported Versions

The following versions of **WT901C_ros2** are currently supported with security updates.

| Version / Branch | Supported |
| ---------------- | --------- |
| `main`           | ✅ Yes    |
| `humble`         | ✅ Yes    |
| `jazzy`          | ✅ Yes    |
| Older releases   | ❌ No     |

Only the latest commits on the active ROS 2 branches are supported.  
Security fixes will **not** be backported to archived or deprecated branches.

---

## Reporting a Vulnerability

If you discover a security vulnerability in this repository, please report it responsibly.

### 🔒 How to Report

**Do NOT open a public GitHub issue** for security-related problems.

Instead, report vulnerabilities using **one of the following methods**:

- **GitHub Security Advisories** (preferred):  
  Go to the repository → **Security** tab → **Report a vulnerability**

- **Direct contact** (if GitHub Security Advisories are unavailable):  
  Contact the repository maintainer via GitHub profile messaging or a private discussion.

---

### 📋 What to Include in Your Report

Please include as much detail as possible:

- A clear description of the vulnerability  
- Steps to reproduce the issue  
- Affected files, scripts, or configurations  
- Potential impact (e.g., unsafe robot motion, privilege escalation, data leakage)  
- Suggested mitigation or fix (if known)  

---

### ⏱ Response Timeline

You can expect the following response timeline:

- **Acknowledgement**: within **48–72 hours**
- **Initial assessment**: within **5–7 days**
- **Fix or mitigation plan**: as soon as reasonably possible

If the vulnerability is accepted:
- A fix will be developed and released
- Credit will be given to the reporter (unless anonymity is requested)

If the vulnerability is declined:
- You will receive an explanation with reasoning

---

## Scope of Security Considerations

This repository primarily interfaces with:

- Serial devices (USB / RS485 IMU)
- ROS 2 middleware and DDS communication
- Linux system services (udev rules, device permissions)
- Embedded IMU configuration and calibration commands

Security issues may include (but are not limited to):

- Unsafe or overly permissive udev rules  
- Excessive device permissions or privilege escalation  
- Unsafe shell scripts or install scripts  
- Remote access exposure via ROS topics, services, or parameters  
- Malicious or unintended calibration command execution  
- Dependency or supply-chain vulnerabilities  

---

## IMU Calibration and Real-Robot Deployment

This repository supports **on-device IMU calibration**, allowing the WT901C IMU to be calibrated and then deployed reliably in **real robotic systems**.

Supported calibration workflows may include:

- Accelerometer bias and offset calibration  
- Gyroscope bias calibration  
- Magnetometer alignment and hard/soft iron compensation (where supported)  
- Persistent storage of calibration parameters on the IMU device  

Once calibrated, the IMU can be safely used for:

- Odometry and localization pipelines  
- Sensor fusion (EKF / UKF / robot_localization)  
- Mobile robots, manipulators, and autonomous platforms  

---

### ⚠️ Calibration Security & Safety Considerations

Calibration directly affects motion estimation, navigation, and control behavior.  
Incorrect or malicious calibration data can result in **unsafe robot motion**.

Users and integrators should ensure:

- Calibration commands are executed **only by trusted operators**
- Calibration services, topics, or serial commands are not exposed to untrusted ROS nodes
- Device access permissions are restricted to intended users or groups
- Stored calibration parameters are validated before use in production systems
- Recalibration is performed after:
  - Mechanical shocks or impacts
  - Sensor remounting or orientation changes
  - Hardware replacement or wiring changes

Any vulnerability related to **calibration commands, parameter storage, or misuse in safety-critical robots** is considered **in-scope** and must be reported following this policy.

---

## Calibration Security Checklist (Recommended)

Before deploying a calibrated IMU in a real robot:

- [ ] Verify calibration in a controlled environment  
- [ ] Confirm correct axis alignment and frame conventions  
- [ ] Restrict access to calibration services and serial interfaces  
- [ ] Disable calibration commands during normal robot operation  
- [ ] Log calibration timestamps and parameters  
- [ ] Validate IMU output against ground truth or reference sensors  

---

## ROS 2 Safety and Best-Practice Alignment

This project follows general ROS 2 safety and deployment best practices, including:

- Principle of least privilege for device access  
- Clear separation between calibration, configuration, and runtime operation  
- Avoiding unsafe default permissions  
- Encouraging validation before deployment in safety-critical systems  

For robots operating in real environments, users are encouraged to align deployments with:

- ROS 2 lifecycle node concepts  
- Controlled parameter updates  
- Failsafe behavior in navigation and control stacks  

---

## Responsible Disclosure

Please allow adequate time for investigation and remediation **before publicly disclosing any vulnerability**.

We appreciate responsible disclosure and community contributions that improve the safety, reliability, and real-world usability of this project.

---

## Acknowledgements

Thank you for helping keep **WT901C_ros2** and the ROS ecosystem secure.
