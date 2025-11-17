# Guide for MetaSejong AI Robotics Competition 2025

# Release History 

**Note**: When a new version is released, you must download the Docker image file again.

| Version | Features |
|------|------|
|V1.0-R202504XX | **The first release** <br>- Launch IsaacSim Simulator <br>- Start designated simulation application <br>- Usefull *make commands* to develop and test simulation application|
|V1.3.0 | **First Patch After Opening** <br>- Fix bug in stage score calculation logic <br>- Optimize MetaSejong 3D model sizes  <br>- Clarify ROS2 topic naming  <br>- Update scoreboard UI|


# Developers guide

See [MetaSejong Developer's Guide](https://metasejong-competition.readthedocs.io/)

---

## Additional Setup Guides

The following items provide **official repositories/documentation links only**. Please follow the linked guides as-is for installation and configuration. This competition guide does not provide per-environment installation support.

### Mobius (oneM2M CSE)
- Official repository: **https://github.com/IoTKETI/Mobius**  
- Description: oneM2M-standard IoT server platform. Refer to the repository README/Wiki for install, operation, and API usage.

### Mobius Open Platform (Admin Console, Tools)
- Official repository: **https://github.com/IoTKETI/mobius-open-platform**  
- Description: Management tools for Mobius (status visualization, device/permission management, resource browser, dashboards). Follow the repo docs for deployment and usage.

### ROS 2 (Humble recommended)
- Official installation guide: **https://docs.ros.org/en/humble/Installation.html**  
- Description: OS-specific steps (e.g., Ubuntu 22.04 LTS) and environment setup. Competition examples target ROS 2 Humble.

### NVIDIA Omniverse Isaac Sim 4.2.0
- Official installation guide (4.2.0): **https://docs.isaacsim.omniverse.nvidia.com/4.2.0/installation/index.html**  
- Description: Workstation/Container/Cloud/PIP install paths and system requirements. Using version **4.2.0** is strongly recommended.
