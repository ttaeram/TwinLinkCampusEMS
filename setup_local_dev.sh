#!/bin/bash

# 기준 디렉토리
SRC_DIR=$(pwd)/resources
TARGET_DIR=~/Documents

# # 1. meta-sejong
# mkdir -p $TARGET_DIR
# ln -sfn $SRC_DIR/models/meta-sejong $TARGET_DIR/meta-sejong
# chmod -R 777 $SRC_DIR/models/meta-sejong

# # 2. robot
# mkdir -p $TARGET_DIR/robot
# ln -sfn $SRC_DIR/models/robot $TARGET_DIR/robot
# chmod -R 777 $SRC_DIR/models/robot

# # 3. assets
# ln -sfn $SRC_DIR/models/assets $TARGET_DIR/assets
# chmod -R 777 $SRC_DIR/models/assets

# # 4. requirements.txt
# mkdir -p $TARGET_DIR/python
# ln -sfn $SRC_DIR/python/requirements.txt $TARGET_DIR/python/requirements.txt
# chmod 777 $SRC_DIR/python/requirements.txt

# # 5. config.yaml
# mkdir -p $TARGET_DIR/env_config
# ln -sfn $SRC_DIR/env_config/config.yaml $TARGET_DIR/env_config/config.yaml
# chmod 777 $SRC_DIR/env_config/config.yaml

# 6. pickandplace extension → ais.omnigraph.extension
mkdir -p $TARGET_DIR/Kit/shared/exts
ln -sfn $SRC_DIR/extensions/omni.metacom.pickandplace.extension $TARGET_DIR/Kit/shared/exts/aisl.omnigraph.extension
chmod -R 777 $SRC_DIR/extensions/omni.metacom.pickandplace.extension

# # 7. output (test only)
# ln -sfn $SRC_DIR/output $TARGET_DIR/output
# chmod -R 777 $SRC_DIR/output

# # 8. omni.isaac.sim.base.kit → Isaac Sim 내부 디렉토리
# ISAAC_SIM_DIR=~/.local/share/ov/pkg/isaac-sim-4.2.0
# mkdir -p $ISAAC_SIM_DIR/apps
# ln -sfn $SRC_DIR/extensions/omni.isaac.sim.base.kit $ISAAC_SIM_DIR/apps/omni.isaac.sim.base.kit
# chmod -R 777 $SRC_DIR/extensions/omni.isaac.sim.base.kit

echo "✅ 심볼릭 링크 + 권한 777 설정 완료!"
