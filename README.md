# 이지 컨트롤러 중력+마찰 보상 ROS2 패키지

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
