#!/bin/bash
# 重启nav_slam容器并挂载推理所需的文件

echo "停止现有容器..."
docker stop nav_slam 2>/dev/null
docker rm nav_slam 2>/dev/null

echo "启动新容器，挂载Intern_g1和模型..."
docker run -d --name nav_slam \
    --net=host \
    --runtime=nvidia \
    --gpus all \
    -e ROS_DOMAIN_ID=0 \
    -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v $HOME/.Xauthority:/root/.Xauthority \
    -e XAUTHORITY=/root/.Xauthority \
    -v /home/spirit-ai/thor_livox_mid360s_installer:/home/spirit-ai/thor_livox_mid360s_installer \
    -v /home/spirit-ai/vln_ws:/workspace/vln_ws \
    -v /home/spirit-ai/Intern_g1:/workspace/Intern_g1 \
    -v /home/spirit-ai/model_vln:/workspace/model_vln \
    -e LD_LIBRARY_PATH=/home/spirit-ai/thor_livox_mid360s_installer/Livox_SDK2/build/sdk_core:$LD_LIBRARY_PATH \
    --privileged \
    harbor.i.spirit-ai.com:443/slam_nav/nav_release:jazzy-thor-vln-deps-fixed-20260714 \
    bash -lc 'while true; do sleep 1000; done'

echo ""
echo "容器已启动，挂载信息："
echo "  - Intern_g1项目: /workspace/Intern_g1"
echo "  - 推理模型: /workspace/model_vln"
echo ""
echo "可以使用以下命令进入容器："
echo "  docker exec -it nav_slam bash"
