echo "Gravity_compensation 컨테이너를 시작합니다..."

# 호스트에서 GUI 권한 허용
xhost +local:docker

# 컨테이너 실행
docker run -it --rm \
    --name gravityCompensation_container \
    --privileged \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --env="ROS_DOMAIN_ID=51" \
    --env="RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="$(pwd):/root/colcon_ws" \
    --network host \
    gravity_compensation:latest
    
echo "컨테이너가 종료되었습니다."