# 1. ROS 2 Humble 데스크탑 풀 이미지 사용
FROM osrf/ros:humble-desktop-full

# 기본 쉘 설정
SHELL ["/bin/bash", "-c"]

# 2. 필수 도구 설치 (git, pip 등)
RUN apt-get update && apt-get install -y \
    git \
    python3-pip \
    python3-rosdep \
    ros-$ROS_DISTRO-dynamixel-sdk \
    ros-$ROS_DISTRO-pinocchio\
    && rm -rf /var/lib/apt/lists/*

# 3. 작업 디렉토리 설정 (컨테이너 접속 시 바로 여기로 들어감)
WORKDIR /root/colcon_ws

# 4. 환경 설정 (자동 source)
# 환경 변수와 기본 ROS 설정을 미리 넣어둡니다.
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# 5. rosdep 초기화 (의존성 자동 설치 도구)
RUN rosdep update