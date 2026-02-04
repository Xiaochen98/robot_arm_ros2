# DOFBOT Pro ROS2 Workspace – Source Packages Overview

This directory (`dofbot_pro_ws/src`) contains ROS 2 packages and vendor components related to the DOFBOT Pro robotic arm.  
The packages include **core drivers**, **robot description**, **interfaces**, and a collection of **vendor-provided demo and perception modules**.

This document briefly explains the role of each folder.

---

## Core & Infrastructure Packages

### `dofbot_pro_driver`
**Type:** Core ROS2 driver  
**Description:**  
Low-level ROS 2 driver responsible for communicating with the DOFBOT Pro hardware.  
This package bridges ROS messages/services to the underlying vendor SDK.

> This is a **core package** and part of the main control stack.

---

### `dofbot_pro_interface`
**Type:** ROS2 interface definitions  
**Description:**  
Defines ROS 2 service and action interfaces used by the DOFBOT Pro system, such as motion commands and state queries.

> Used by drivers, planners, and higher-level applications.

---

### `dofbot_pro_msgs`
**Type:** ROS2 message package  
**Description:**  
Custom ROS 2 message definitions shared across multiple DOFBOT Pro packages.

---

### `dofbot_pro_description`
**Type:** Robot description (URDF/Xacro)  
**Description:**  
Contains the robot model, including URDF/Xacro files, meshes, and joint definitions.  
Used by RViz, MoveIt, and simulation tools.

---

### `dofbot_pro_moveit`
**Type:** Motion planning  
**Description:**  
MoveIt configuration package for DOFBOT Pro, including planning groups, controllers, and RViz configurations.

---

## Vendor SDK & External Dependencies

### `Arm_Lib`
**Type:** Vendor SDK (Python)  
**Description:**  
Manufacturer-provided Python SDK for low-level robotic arm control (servo, joint, communication).  
This is a **vendor dependency**, not developed in this repository.

> ⚠️ License may be unspecified. Recommended to keep private or treat as a vendor dependency.

---

### `OrbbecSDK_ROS2-main`
**Type:** Third-party ROS2 package  
**Description:**  
ROS 2 integration for Orbbec depth cameras, used for RGB-D perception and depth-based applications.

---

## Perception & Vision Demo Packages (Vendor-Provided)

> The following packages are primarily **demonstration or example applications** provided by the vendor.
> They are useful for reference but are **not core system components**.

---

### `dofbot_pro_vision`
**Type:** Vision framework / demos  
**Description:**  
General vision-related utilities and demos using camera input and OpenCV.

---

### `dofbot_pro_depth`
**Type:** Depth perception demo  
**Description:**  
Depth-based perception examples, typically relying on RGB-D sensors.

---

### `dofbot_pro_color`
**Type:** Color-based vision demos  
**Description:**  
Demonstration of color detection, tracking, and rule-based behaviors using OpenCV.

---

### `dofbot_pro_apriltag`
**Type:** AprilTag demo  
**Description:**  
AprilTag detection examples for localization and object referencing.

---

### `dofbot_pro_mediapipe`
**Type:** Vision / AI demo  
**Description:**  
Human pose, hand, or face tracking demos based on MediaPipe.

---

### `dofbot_pro_KCF`
**Type:** Tracking demo  
**Description:**  
KCF (Kernelized Correlation Filter) based object tracking examples.

---

### `dofbot_pro_yolov11`
**Type:** Deep learning demo  
**Description:**  
YOLOv11-based object detection demos (e.g., garbage classification).

> ⚠️ May include large model files; handle with care in public repositories.

---

## Interaction & Application-Level Demos

### `dofbot_pro_voice_ctrl`
**Type:** Voice control demo  
**Description:**  
Voice-command-based control examples, often relying on external speech recognition services.

---

### `dofbot_pro_info`
**Type:** Information & helper package  
**Description:**  
Utility package providing system or robot information display and helper scripts.

---

### `oled_yahboom`
**Type:** Peripheral demo  
**Description:**  
OLED display support for showing system status or startup information on Yahboom hardware.

---

## Archives & Miscellaneous

### `dofbot_pro_info.zip`
**Type:** Archive  
**Description:**  
Compressed vendor-provided documentation or resources.

> Recommended to extract externally or archive separately.

---

## Notes

- Only a **small subset** of these packages is required for core robotic arm control.
- Many packages are **vendor demos** intended for learning or showcasing functionality.
- For long-term development, it is recommended to:
  - Treat vendor SDKs as external dependencies
  - Keep demo packages archived or clearly marked as non-core
  - Focus development on driver, interface, and planning layers

---
