# config 파일

## 파일 구조 설명

### 1. 기본 구조

{working directory}/config/config.yml 파일을 읽어서 임무 환경을 구성한다.

시나리오 정의에 대한 보조적인 설정으로 시나리오 정의에서 optional로 정의되지 않은 값에 대한 기본값 설정, 
플랫폼에서 지원하는 임무 객체(재활용 쓰레기)의 종류 목록, 임무 객체에 대한 {모델 타입, 모델 파일, scale} 정보를 정의.

```
{project-root}
  └── scenario-data/
      ├── scenario/       # 시나리오 정의 파일을 위한 sub directory (@see ./scenario_description_file.md)
      └── answer-sheets/ 
```

### 2. YAML 파일 구조
설정 YAML 파일은 다음 구조를 따릅니다:

```yaml
fixed_camera:   ##  CCTV에 대한 default 속성 정보 
  default_focal_length: 30.0
  default_horizontal_aperture: 40.0
  default_ground_height: 1.0

mission_object_types: ##  플랫폼에서 지원하는 쓰레기 유형
  - master_chef_can
  - cracker_box
  - ...

mission_object_assets:  ##  플랫폼에서 지원하는 쓰레기 모델 정보
  master_chef_can:
    type: ycb
    usd_file: 002_master_chef_can.usd   ## {CONST_METASEJONG_RESOURCES}/assets 경로로 부터의 상대경로 사용됨 
    scale: [0.4, 0.4, 0.4]
  cracker_box:
    type: ycb
    usd_file: 003_cracker_box.usd
    scale: [0.7, 0.7, 0.7]
  ... 
```
