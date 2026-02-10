# 🎂 Robo Pâtissier: AI 기반 협동로봇 케이크 자동화 시스템

![ROS2](https://img.shields.io/badge/Framework-ROS2%20Humble-3399FF?style=flat-square&logo=ros&logoColor=white) 
![Doosan Robotics](https://img.shields.io/badge/Robotics-Doosan%20M0609-0054A6?style=flat-square&logo=robotframework&logoColor=white) 
![YOLOv8](https://img.shields.io/badge/AI%20Vision-YOLOv8-FF6F00?style=flat-square&logo=ultralytics&logoColor=white) 
![FastAPI](https://img.shields.io/badge/Backend-FastAPI-009688?style=flat-square&logo=fastapi&logoColor=white)

> **AI와 협동로봇을 결합하여 고품질 커스텀 케이크 제작 공정을 자동화하는 솔루션입니다.**

---

## 1. Project Overview (프로젝트 개요)

급성장하는 커스텀 케이크 시장의 수작업 한계를 극복하기 위해 기획되었습니다. 작업자의 숙련도에 의존하던 기존 방식에서 벗어나, **AI 비전**과 **정밀 로봇 제어**를 통해 균일한 품질과 효율적인 생산 시스템을 제공합니다.

### 🚀 Core Functions (주요 기능)

* **멀티모달 주문 시스템**
    * **Web UI**: 직관적인 인터페이스를 통한 디자인 및 옵션 선택
    * **AI 음성 비서**: OpenAI Whisper 기반 NLU를 활용한 음성 상담 및 주문 지원
* **지능형 커스텀 드로잉**
    * 사용자 업로드 이미지나 생성형 AI(Pollinations.ai) 이미지를 좌표 데이터로 변환
    * 협동로봇을 이용한 정밀 시럽 드로잉 구현
* **AI Vision 품질 검증 (YOLOv8)**
    * 토핑 위치 실시간 인식 및 배치 정확도 확인
    * 드로잉 결과물을 원본과 비교하여 누락된 부분 자동 리터치
* **엔드 투 엔드 자동화 공정**
    * 시럽 드로잉 → 파우더 도포 → 토핑 배치 → 픽업대 이송 전 과정 자동화
* **안전 및 관제 시스템**
    * **안전 복구**: 비상 정지 시 Firebase 연동을 통한 현재 상태 저장 및 안전 위치 복귀
    * **관리자 대시보드**: 로봇 관절 좌표 모니터링 및 실시간 매출/소모품 통계 제공

---

## 2. Tech Stack (기술 스택)

### 🤖 Environment & Robotics
* **OS**: Ubuntu
* **Framework**: ROS2 (Robot Operating System 2)
* **Robotics**: Doosan Robot Language (DRL), MoveIt2

### 🧠 AI & Machine Learning
* **Computer Vision**: Ultralytics YOLOv8, OpenCV
* **Speech & NLP**: OpenAI Whisper, Natural Language Understanding
* **Generative AI**: Pollinations.ai (Image Generation)

### 💻 Web & Backend
* **Frontend**: React.js, Vite, Tailwind CSS, Three.js (3D 시뮬레이션)
* **Backend**: FastAPI (Python)
* **Database**: Firebase Realtime Database
* **API**: Google Drive API, CoolSMS API

### 🛠 Hardware
* **Robot**: Doosan Collaborative Robot (M0609)
* **Sensor**: Intel RealSense Camera

---

## 3. Core Technologies (핵심 기술)

### 👁️ AI Vision: YOLOv8 & Real-time Inspection
단순한 자동화를 넘어, **지능형 품질 검사**를 수행합니다.
* **Topping Recognition**: YOLOv8 모델을 사용하여 케이크 위 토핑의 종류와 위치를 실시간으로 파악합니다.
* **Drawing Retouching**: OpenCV를 이용해 실제 드로잉 결과와 원본 도안을 비교 분석하고, 선이 끊기거나 미흡한 부분을 로봇이 스스로 판단하여 보정(Retouching) 작업을 수행합니다.

### 🤖 Robotics: ROS2 & Precision Path Control
복잡한 도안을 로봇의 정밀한 움직임으로 구현합니다.
* **Image to Path**: 사용자가 입력한 이미지를 이진화 및 세선화(Skeletonization) 처리를 거쳐 로봇이 따라갈 수 있는 좌표(Cartesian Coordinates) 데이터로 변환합니다.
* **Multi-Layer Control**: ROS2 프레임워크와 Doosan Robot Language(DRL)를 결합하여 시럽 드로잉, 파우더 도포, 토핑 배치 등 서로 다른 특성의 작업을 하나의 공정으로 통합 제어합니다.

### 🎙️ Multi-modal AI: Whisper-based Voice Ordering
접근성을 높인 **차세대 주문 시스템**을 구축했습니다.
* **NLU (Natural Language Understanding)**: OpenAI Whisper를 통해 음성을 텍스트로 변환하고, 사용자의 의도를 분석하여 복잡한 주문 옵션을 자동으로 추출합니다.

---

## 4. Safety & Recovery System (안전 및 복구)

실제 운영 환경에서의 변수를 고려하여, 단순 중단이 아닌 **연속성을 확보하는 복구 시스템**을 구축했습니다.

* **State Persistence**: 공정 중 비상 정지나 오류 발생 시, Firebase를 통해 로봇의 마지막 작업 단계와 상태 값을 실시간으로 보존합니다.
* **Process-Specific Recovery (공정별 맞춤 후처리)**: 초기화 버튼 클릭 시, 단순히 홈으로 복귀하는 것이 아니라 현재 진행 중인 공정(시럽, 파우더, 토핑 등)을 식별하여 각 공정에 맞는 **전용 후처리 로직**을 우선 실행합니다.
* **Safe Home Return**: 모든 후처리 단계가 안전하게 완료된 것을 확인한 후, 최적의 경로를 통해 **홈 위치(Home Position)**로 복귀합니다.

---

## 👥 Team Members & Roles (팀원 및 역할)

| 이름 | 역할 | 주요 담당 업무 |
| :--- | :--- | :--- |
| **박제준** | **👑 Team Leader** | • 전체 프로세스 시나리오 통합 및 예외 처리 로직 설계<br>• 커스텀 도안용 이미지 변환 알고리즘 고도화<br>• AI Vision 기반 드로잉 품질 검사 및 로봇 보정 동작 구현 |
| **이승준** | **💻 Full-Stack Dev** | • React/FastAPI 기반 웹 서비스 및 실시간 DB(Firebase) 구축<br>• 관리자용 매출 통계 대시보드 및 로봇 상태 관제 시스템 개발<br>• 고객 알림 서비스 및 작업 타임랩스 기능 구현 |
| **안효원** | **🤖 Robotics & AI** | • ROS2 기반 로봇 공정 환경 구성 및 시럽 드로잉 제어<br>• YOLOv8 객체 인식 모델 학습 및 토핑 위치 데이터 추출<br>• 실시간 사용자 인식 및 휴먼-로봇 인터랙션(HRI) 구현 |
| **한지엽** | **🎙️ AI & Contents** | • OpenAI Whisper 기반 음성 주문 인터페이스 및 NLU 모델 구현<br>• 토핑 배치 로직 설계 및 하드웨어 공정 최적화<br>• 프로젝트 홍보 영상 제작 및 대외 콘텐츠 총괄 |
