# 이지 컨트롤러 중력+마찰 보상 ROS2 패키지

## 사용 전 설정 필수
### 다이나믹셀
1. 보어레이트 4.5M
2. 응답지연시간 0ms
3. 토크 최대/최소 제한 1000/-1000
4. ID 설정 (왼팔 - 10~16, 오른팔 - 20~26 권장 -> 기본 설정은 왼팔만 지원 코드에서 ID 바꿔줘야함)

### 컴퓨터 
- USB 지연시간 1ms로 설정
  모든 USB-Serial 장치에 대해 latency를 1로 설정하는 규칙 생성
  ```
  echo 'ACTION=="add", SUBSYSTEM=="usb-serial", DRIVER=="ftdi_sio", ATTR{latency_timer}="1"' | sudo tee /etc/udev/rules.d/99-usb-latency.rules
  ```
  ```
  sudo udevadm control --reload-rules
  sudo udevadm trigger
  ```
  > 만약 영구 적용이 아니라 일시만 적용하고 싶으면 (다만 영구 적용을 강력 추천)
  ```
  echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
  ```
- 적용 확인
  ```
  cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
  ```
  숫자가 1로 나오면 성공
  
---

## 사용법

### 기본 노드만 키기
``` 
ros2 run gravity_compensation gravity_node
```

### RVIZ에서 로봇 확인하기 
``` 
ros2 launch gravity_compensation check_robot.launch.py
``` 

### 파라미터 튜닝하기
gravity_node가 켜진 상태에서 rqt로 수정가능
```
ros2 run rqt_reconfigure rqt_reconfigure
# 또는 그냥 rqt 입력 후 Plugins -> Configuration -> Dynamic Reconfigure
```


