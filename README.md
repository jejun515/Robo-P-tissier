# 🦾 Autonomous Pick & Place System with Vision AI
> **Deep Learning(YOLOv8) 기반의 객체 인식과 Depth Sensing을 활용한 두산 로봇 자율 파지(Grasping) 시스템입니다.**

## 📖 Project Overview
이 프로젝트는 **RealSense D435i** 카메라로 작업 공간을 스캔하여 목표 물체를 인식하고, **Doosan Robotics (M-Series)** 로봇 팔을 제어하여 물체를 집어 올리는(Pick) 시스템의 제어 로직을 담고 있습니다.
Firebase를 통해 주문(Order)이 들어오면, 로봇은 대기 상태에서 벗어나 YOLO 모델이 탐지한 좌표로 이동하여 작업을 수행합니다.

---

## 🛠 Tech Stack

### Hardware
<img src="https://img.shields.io/badge/Robot-Doosan M0609-C70025?style=for-the-badge&logo=robot&logoColor=white"> <img src="https://img.shields.io/badge/Camera-Intel RealSense D435i-0071C5?style=for-the-badge&logo=intel&logoColor=white"> <img src="https://img.shields.io/badge/Gripper-OnRobot RG2-gray?style=for-the-badge&logo=arduino&logoColor=white">

### Software & Libraries
<img src="https://img.shields.io/badge/ROS 2-Humble-22314E?style=for-the-badge&logo=ros&logoColor=white"> <img src="https://img.shields.io/badge/YOLOv8-Ultralytics-1F425F?style=for-the-badge&logo=pytorch&logoColor=white"> <img src="https://img.shields.io/badge/OpenCV-Computer Vision-5C3EE8?style=for-the-badge&logo=opencv&logoColor=white"> <img src="https://img.shields.io/badge/Firebase-Realtime DB-FFCA28?style=for-the-badge&logo=firebase&logoColor=black">

---

## 🏗️ System Architecture

전체 시스템은 **Vision Node**와 **Control Node**가 토픽 통신을 주고받으며 협업하는 구조입니다.

```mermaid
graph TD
    User((Firebase Order)) -->|Trigger| A[Robot Control Node]
    B[RealSense Camera] -->|"RGB-D Image"| C[YOLO Detection Node]
    C -->|"Object Class & Pixel(u,v)"| D{"Coordinate Transform"}
    D -->|"Spatial Coord (x,y,z)"| A
    A -->|"Motion Command"| E[Doosan Robot Controller]
    A -->|"Grip Command"| F[OnRobot RG2]
    
    subgraph "Vision Processing"
    B
    C
    D
    end

    subgraph "Actuation"
    E
    F
    end
```
