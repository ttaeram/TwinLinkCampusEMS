

# 프로젝트 폴더 구성 ('local' 실행 환경)

Docker로 배포할 때 연관성을 고려해서 구성

```
metacom2025/
├── docs/                   # documentations (개발 진행과정에서 프로젝트에 대한 documentation 파일들을 저장)
├── resources/              # MetaSejong, 재활용 쓰레기 모델 파일들과 IsaacSim extension 파일들을 저장
|   ├── assets/                 # 재활용 쓰레기 모델 파일(USD)과 texture 파일들
|   ├── models/                 # 세종대 캠퍼스 모델 파일(USD)과 로봇 모델 파일(USD). Google Drive에서 다운로드 받아야 함
|   |   ├── playground/            # 세종대 캠퍼스 모델 파일들로, 훈련 환경별 USD 파일을 포함
|   |   |   ├── Assets              # -> 훈련 환경 USD 파일들이 공유하는 Assets 데이터
|   |   |   └── {demo_field, chumgmu_field, gunja_field, ...}.usd           # -> 훈련 환경 USD 파일들
|   |   └── robot/              # robot USD
|   └── extensions/         # 로봇 Arm 관련 omnigraph extension
├── simulation_app/         # 경진대회 서버 플랫폼 코드. 3D Model들을 로드하고, 사용자 application의 요청에 따라 로봇 구동 및 채점하는 기능 수행
|   ├── codes/                  # 경진대회 운용을 위한 simulation app code
|   └── config/                 # 경진대회 운용 관련 설정 파일들 
├── scenario-data/         # 문제 시나리오를 정의하는 파일들 
    ├── {demo, chumgmu, gunja, ...}.yaml
|   └── answer-sheets/          # 시나리오에서 random하게 배치되는 Trash에 대한 GT값을 파일로 생성하는 폴더
└── .runtime-data/          # 시뮬레이션 실행 중에 생성되는 임시 파일들 
    ├── cache/                  # 각종 캐시파일
    |   └── {.local, root, .nv, .nvidia-omniverse, isaac-sim}/
    ├── log/                    # 로그 파일 
    ├── data/                   # 임시 데이터 파일 
    └── scenario-data/          # 시나리오에서 random하게 배치되는 Trash에 대한 GT값을 파일로 생성하는 폴더
```

# 프로젝트 폴더 구성 ('docker' 실행 환경)

## 보안이 필요하지 않은 개발, 경진대회용 docker 환경은 "COPY from"으로 되어 있는 부분들을 host volume binding으로 처리 가능 
## 보안이 필요한 배포용 docker 환경은 "COPY"로 복사한 다음에 image만 배포  

copy로 이미지만 배포하는 docker 관련 파일들은 Dockerfile.release, docker-compose-release.yml을 이용
host volume binding으로 개발하는 경우 docker 관련 파일들은 Dockerfile, docker-compose.yml을 이용

image로 배포하는 경우 개발 과정에서 debug를 위한 데이터 확인용으로 .runtime-data는 host volume binding으로 처리

```
/
├── root/
|   ├── .cache/                             # 프로젝트 폴더의 .runtime-data/cache/ 하위 폴더로 binding (특별히 정한건 아니고, Cursor.app의 도움으로 최초 Dockerizing 구성할 때 mapping된 구조임)
|   |   └── {ov, pip, nvidia/GLCache}/          # .runtime-data/cache/root/{ov, pip,clcache}/ 폴더로 각각 binding 
|   ├── .nv/ComputeCache                    # 프로젝트 폴더의 .runtime-data/cache/nv/computecache/ 폴더로 binding (특별히 정한건 아니고, Cursor.app의 도움으로 최초 Dockerizing 구성할 때 mapping된 구조임)
|   ├── .local/share/ov/data/               # 프로젝트 폴더의 .runtime-data/data/local-ov/ binding (특별히 정한건 아니고, Cursor.app의 도움으로 최초 Dockerizing 구성할 때 mapping된 구조임)
|   ├── .nvidia-omniverse/logs/             # 프로젝트 폴더의 .runtime-data/logs/nvidia-omniverse/ binding (특별히 정한건 아니고, Cursor.app의 도움으로 최초 Dockerizing 구성할 때 mapping된 구조임)
|   └── Documents/                          
|       ├── Kit/shared/exts
|       |   └── aisl.omnigraph.extension    # COPY from resources/extensions/omni.metacom.pickandplace.extension/  배포 환경에서는 COPY, 개발 환경에서는 binding
|       └── metasejong/                     
환경에서는 binding
├── isaac-sim/                              
|   └── kit/cache                           # 프로젝트 폴더의 .runtime-data/cache/isaac-sim-kit/ 폴더로 binding (특별히 정한건 아니고, Cursor.app의 도움으로 최초 Dockerizing 구성할 때 mapping된 구조임)
└── metacom2025/
    ├── simulation_app/                         # bind to simulation_app/
    |   ├── configs
    |   └── codes
    ├── resources/
    |   ├── models/                     # COPY from resources/models/   배포 환경에서는 COPY, 개발 환경에서는 binding
    |   └── assets/                     # COPY from resources/assets/   배포 환경에서는 COPY, 개발 ```
    └── scenario-data/          
        └── answer-sheets/              # 프로젝트 폴더의 .runtime-data/scenario_data/ 폴더로 binding. 훈련 과정에서 GT를 제공해주는 결과이므로 참가자에게 guide를 줄지 여부를 판단해야 함. **Test only**

# 운용 환경(local or docker) 따른 환경변수/상수

## Docker 환경으로 동작하는지 여부 
ENV_METASEJONG_DOCKER   = NO (default) | YES # YES인 경우만 docker 환경

## 시나리오(훈련 필드 선택)
ENV_METASEJONG_SCENARIO = demo (default) | chungmu | gunja | gwanggaeto | aicenter


## 운용 환경(local, docker)에 따라 코드상에서 다르게 적용해야 하는 경로 상수
|상수 이름|local 환경|Docker 환경|
|---|---|---|
|CONST_WORKING_DIRECTORY       |{pwd}       |/metacom2025/simulation_app |
|CONST_METASEJONG_RESOURCES           |{CONST_WORKING_DIRECTORY}/resources/ | {CONST_WORKING_DIRECTORY}/resources/ |
|CONST_METASEJONG_SCENARIO_DATA       |{CONST_WORKING_DIRECTORY}/.runtime-data/scenario-data/       |/root/.metasejong/scenario-data |


# FYI: 기존에 개발하던 프로젝트 구조를 참고로 정리한것임 

## 프로젝트 구조

```
metacom2025/
├── docs/                   # documentations 
├── resources/              # USD, Assets
|   ├── assets/                 # Trash USD, texture, ...
|   ├── extensions/             # 로봇 Arm 관련 omnigraph extension
|   └── models/                 # Google drive에서 다운로드
|       ├── playground/           # playground(MetaSejong) USD
|       └── robot/                 # robot USD
├── simulation_app/
|   ├── codes/                 # Trash USD
|   └── config/             # 로봇 Arm 관련 omnigraph extension
└── workspace/              # simulation 구동하면서 생성되는 임시 파일들 
    ├── cache, log, ... /   # [Don't care] Trash USD
    └── output/             # 시나리오에서 random하게 배치되는 Trash에 대한 GT값을 파일로 생성하는 폴더
```

## Docker 환경 구조

```
/
├── root/
|   ├── .cache/                             # bind to workspace/cache/
|   |   └── {ov, pip, nvidia/GLCache}/      #   .../cache/{ov, pip,clcache}/
|   ├── .nv/ComputeCache                    # bind to workspace/cache/computecache
|   ├── .local/share/ov/data/               # bind to workspace/data/
|   ├── .nvidia-omniverse/logs/             # bind to workspace/logs/
|   ├── Documents/                          # bind to workspace/documents/          <<< 아래 extensions binding과 출돌나는거 아닌지 확인 필요 
|   |   └── Kit/shared/exts
|   |       └── aisl.omnigraph.extension    # bind to resources/extensions/omni.metacom.pickandplace.extension                
|   └── metasejong/
|       ├── models/                         # bind to resources/models/
|       ├── assets/                         # bind to resources/assets/
|       └── output/                         # bind to resources/output/ << Test only
├── isaac-sim/              # isaac-sim Dockerfile에서 지정됨
|   └── kit/cache           # bind to workspace/cache/kit
└── metacom2025/
    └── simulation_app/                         # bind to simulation_app/
        ├── configs
        └── codes
```

