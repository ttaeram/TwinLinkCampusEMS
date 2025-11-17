# 시나리오 정의 파일

## 파일 구조

### 1. 기본 구조

MetaSejong 플랫폼의 "시나리오 정의 파일"은 임무 환경을 구성하는 핵심 설정 파일입니다. 이 파일은 다음과 같은 요소들을 정의합니다:
- 임무 공간을 구성하는 USD 파일
- 초기 카메라 시점
- 임무 구역별 카메라 설치 정보
- 임무 대상 재활용 쓰레기의 유형 및 수량

경진대회 참가자는 이 파일을 수정하여 AI 애플리케이션 개발을 위한 학습 데이터를 생성할 수 있습니다.

MetaSejong 플랫폼은 환경 변수 `ENV_METASEJONG_SCENARIO`에 지정된 값을 기반으로 `{working directory}/scenario-data/{ENV_METASEJONG_SCENARIO}.yml` 파일을 읽어 임무 환경을 구성합니다.

현재 제공되는 시나리오는 다음 4가지입니다:
- demo: 데모 시나리오
- dongcheon: 동천관 시나리오
- jiphyeon: 집현관 시나리오
- gwanggaeto: 광개토관 시나리오

```
{working directory}/scenario-data/
├── demo.yml        # 데모 시나리오
├── dongcheon.yml   # 동천관 시나리오
├── jiphyeon.yml    # 집현관 시나리오
└── gwanggaeto.yml  # 광개토관 시나리오
```

### 2. YAML 파일 구조
각 시나리오 YAML 파일은 다음과 같은 구조로 구성됩니다:

```yaml
scenario:     # 시나리오에 대한 개요. '2.1 시나리오 개요'에서 설명됨
  name: Demo  
  description: Demo scenario 
  time_constraint: 15
playground:   # 시나리오가 진행되는 가상공간에 대한 정보. '2.2 가상공간 정보'에서 설명됨
  usd_file: playground/S1/SejongUniv_S1.usd
  view_point:
    camera_view_eye: [-113.03272, 64.98621, 30.59225]
    camera_view_target: [-63, 119, 1.02]
mission:    # 시나리오에서 수행할 임무에 대한 정보. '2.3 임무 정보'에서 설명됨
  {area_name}:
    camera_info:
      focal_length: 4
      horizontal_aperture: 4.8
      ground_height: 1.0
      position: [x, y, z]
      rotation: [rx, ry, rz]
    mission_objects:
      {object_name}: count
robot:    # 시나리오에서 로봇이 배치될 위치 '2.4 로봇 정보'에서 설명됨
  start_point: [x, y, z]
```

#### 2.1. 시나리오 개요

시나리오의 기본 정보를 정의합니다. 시나리오의 이름, 설명, 완료 제한 시간이 포함됩니다.

- 속성
  
  - name: 시나리오를 식별하는 고유한 이름
  - description: 시나리오에 대한 상세 설명
  - time_constraint: 시나리오 실행 제한 시간(분). 제한 시간이 경과하면 경연이 종료됩니다.

- 예시 
  ```yaml
  scenario:
    name: "Demo"
    description: "시나리오 설명 텍스트"
    time_constraint: 15
  ```

#### 2.2 가상공간 정보

시나리오에서 사용되는 3D 가상공간에 대한 정보를 정의합니다. IsaacSim USD 파일 경로와 초기 시점 정보가 포함됩니다.

- 속성

  - usd_file: 임무공간을 구성하는 IsaacSim 3D 모델 파일(USD) 경로
  - view_point: 시뮬레이션 시작 시 카메라 시점 정보
    - camera_view_eye: 카메라의 위치 좌표
    - camera_view_target: 카메라가 바라보는 목표점의 좌표

- 예시 

  ```yaml
  scenario:
    playground:
      usd_file: playground/demo_area.usd  # {python 실행 경로}/resources/models 기준 상대 경로
      view_point:
        camera_view_eye: [-113.03272, 64.98621, 30.59225]
        camera_view_target: [-63, 119, 1.02]
  ```

#### 2.3. 임무 정보

시나리오의 임무 영역별로 설치된 CCTV 정보와 해당 영역에 배치될 재활용 쓰레기의 종류 및 수량을 정의합니다.

- 속성 

  - {area_name}: 임무 영역을 식별하는 고유 이름. {scenario_id}_{sequence} 형식으로 지정

    - camera_info: 임무 영역의 CCTV 설정 정보
    
      - focal_length: 카메라 초점 거리(centimeters)
      - horizontal_aperture: 센서 가로 넓이(centimeters). 세로는 16:9 비율로 자동 계산
      - ground_height: 촬영 대상 영역의 지표면 고도(meters)
      - position: 카메라 설치 위치 좌표(meters)
      - rotation: 카메라 설치 각도(degrees)

    - mission_objects: 임무 영역에 배치될 재활용 쓰레기 정보
      - {object_type}: 쓰레기 유형별 배치할 개체 수량

- 재활용 쓰레기 유형

  | 유형 | 분류 | 이미지 |
  |------|------|------|
  |cola_can | aluminum | ![cola_can](./mission_object_images/cola_can.png "cola can") |
  |master_chef_can | aluminum| ![master_chef_can](./mission_object_images/master_chef_can.png "master chef can") |
  |juice | plastic| ![juice](./mission_object_images/juice.png "juice") |
  |cracker_box | paper| ![cracker_box](./mission_object_images/cracker_box.png "cracker box") |
  |tissue | paper| ![tissue](./mission_object_images/tissue.png "tissue") |
  |wood_block | none| ![wood_block](./mission_object_images/wood_block.png "wood block") |
  |mug | none| ![mug](./mission_object_images/mug.png "mug") |

- 예시 

  ```yaml
  scenario:
    mission:
      demo_1:
        camera_info:
          focal_length: 4
          horizontal_aperture: 4.8
          position: [-60.86416, 152.94693, 21.47511]
          rotation: [-74.492, -23.79, -168.446]
          ground_height: 16.5
        mission_objects:
          master_chef_can: 3  # 마스터 쉐프 캔 3개
          wood_block: 3       # 나무 블록 3개
          tissue: 3           # 티슈 3개
          cola_can: 3         # 콜라 캔 3개
          juice: 3            # 주스 3개
  ```

#### 2.4. 로봇 정보

로봇의 초기 배치 위치를 좌표로 지정합니다.

- 예시

  ```yaml
  scenario:
    robot:
      start_point: [x, y, z]  # 로봇 초기 위치 좌표
  ```
