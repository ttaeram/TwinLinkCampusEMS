#!/bin/bash
# conda activate metacom_nav_python310
export MODE=local
export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6
# export LD_PRELOAD=~/.local/share/ov/pkg/isaac-sim-4.2.0/kit/extscore/omni.isaac.core/lib/libspdlog.so:/usr/lib/x86_64-linux-gnu/libstdc++.so.6
# ---------- 환경 변수 설정 ----------
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/taeram/Desktop/Digital-Twin-IoT-Anomaly-Detection/simulation_app/config/fastdds.xml
export ISAAC_SIM_PATH="/home/taeram/Isaac-sim_4.2.0"
export ACCEPT_EULA=Y
export PRIVACY_CONSENT=Y
export NVIDIA_VISIBLE_DEVICES=all
export NVIDIA_DRIVER_CAPABILITIES=all
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export AMENT_PREFIX_PATH=${ISAAC_SIM_PATH}/exts/omni.isaac.ros2_bridge/humble
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:${ISAAC_SIM_PATH}/exts/omni.isaac.ros2_bridge/humble/lib
export PYTHONPATH=${ISAAC_SIM_PATH}/kit/python/lib/python3.10/site-packages:${ISAAC_SIM_PATH}/exts:${ISAAC_SIM_PATH}/extscache:.:./simulation_app:./simulation_app/codes
export USD_PLUGIN_PATH=${ISAAC_SIM_PATH}/exts/omni.usd.schema.isaac/plugins
export ROS_DISTRO=humble
export EXTENSIONS=omni.isaac.ros2_bridge
export ROS_DOMAIN_ID=0
# export FASTRTPS_DEFAULT_PROFILES_FILE=./config/fastdds.xml
export QT_X11_NO_MITSHM=1
export DISPLAY=$DISPLAY
export XAUTHORITY=$XAUTHORITY
export EXTENSION_PATHS=./resources/extensions

#   demo    aicenter    dongcheon   jiphyeon    gwanggaeto
export ENV_METASEJONG_SCENARIO=demo

# ---------- 캐시 디렉토리 & 데이터 경로 설정 ----------
# 볼륨 마운트한 경로들을 로컬에서도 동일하게 접근할 수 있게 설정
export KIT_CACHE_DIR=./.runtime-data/cache/kit
export OV_CACHE_DIR=./.runtime-data/cache/ov
export PIP_CACHE_DIR=./.runtime-data/cache/pip
export GLCACHE_DIR=./.runtime-data/cache/glcache
export COMPUTECACHE_DIR=./.runtime-data/cache/computecache
export LOG_DIR=./.runtime-data/logs
export DATA_DIR=./.runtime-data/data
export DOCUMENTS_DIR=./.runtime-data/documents

# 필요시 디렉토리 생성
mkdir -p $KIT_CACHE_DIR $OV_CACHE_DIR $PIP_CACHE_DIR $GLCACHE_DIR $COMPUTECACHE_DIR $LOG_DIR $DATA_DIR $DOCUMENTS_DIR

# ---------- 필요 패키지 설치 ----------
pip3 install --no-cache-dir -r ./simulation_app/requirements.txt

# ---------- 스크립트 실행 ----------
source ${ISAAC_SIM_PATH}/setup_python_env.sh
${ISAAC_SIM_PATH}/python.sh simulation_app/codes/run_simulation_app.py
