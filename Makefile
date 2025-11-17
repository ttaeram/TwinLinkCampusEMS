# /*******************************************************************************
#  * Copyright 2025 AISL Sejong University, Korea
#  *
#  *	Licence: TBD
#  *******************************************************************************/

.PHONY: help setup run build-and-run down build-only release clean
.SILENT: help

help:
	echo "See README.md in this folder"


# playground를 포함하여 models 전체 복사하고자 하는 경우 true 
# ((주의)) Makefile.deploy 파일도 수정해줘야 함 
WITH_PG=true


# This tool now only supports compose V2, aka "docker compose" as it has replaced to old docker-compose tool.
DOCKER_COMPOSE=docker compose

DOCKER_RELEASE_REVISION=r06

ifeq ($(WITH_PG), true)
	#	이미지 명칭과 export할 tar 파일 명칭에 추가할 옵션
	DOCKER_IMAGE_OPTION=-with-playground
else
	#	이미지 명칭과 export할 tar 파일 명칭에 추가할 옵션
	DOCKER_IMAGE_OPTION=
endif

DOCKER_BASE_IMAGE=metasejong-base:ros2-isaacsim
DOCKER_RELEASE_IMAGE=metasejong:metacom-2025$(DOCKER_IMAGE_OPTION)-$(DOCKER_RELEASE_REVISION)
DOCKER_IMAGE_TAR_FILE=metasejong-metacom2025$(DOCKER_IMAGE_OPTION)-$(DOCKER_RELEASE_REVISION).tar

# Setup only needs to be executed once.
# The setup process includes logging into the NVIDIA Docker repository and configuring the X server.
setup:
	docker login nvcr.io
	xhost +local:

# Start a Docker container.
run: setup
	${DOCKER_COMPOSE} -f docker-compose.yml up

# Stop the running Docker container.
down:
	${DOCKER_COMPOSE} -f docker-compose.yml down

# Stop the running Docker container and delete all temporary volumes.
clean:
	${DOCKER_COMPOSE} -f docker-compose.yml down -v

# Build base docker image: base docker image includes IsaacSim and ROS environment
build-base-image:
	docker build -f Dockerfile.base -t $(DOCKER_BASE_IMAGE) .

# Build MetaSejong docker image: MetaSejong docker image includes MetaSejong Models and simulation app which based on base docker image
build:
	docker build -f Dockerfile.deploy -t $(DOCKER_RELEASE_IMAGE) .
	
# Create a tar file for the final submission.
release: 
	docker save $(DOCKER_RELEASE_IMAGE) -o $(DOCKER_IMAGE_TAR_FILE)
