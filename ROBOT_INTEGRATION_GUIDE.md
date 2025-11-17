# Isaac Sim-Robot Integration Guide

#### Replace hard-coded absolute paths
Replace any hard-coded absolute path with your own user path, e.g. `/home/(name)/Desktop/Digital-Twin-IoT-Anomaly-Detection/resources/models/playground/S2/Sensor_Value.usd`.

#### 00. Run Isaac Sim after `conda deactivate` on all terminals and activating ROS2
```
$ conda deactivate
$ source /opt/ros/humble/setup.bash
$ cd Desktop/Digital-Twin-IoT-Anomaly-Detection/
$ ./run_local_sim.sh
```

Modify settings in /opt/ros/humble/share/nav2_bringup/params/nav2_params.yaml:
```yaml
map_server:
  ros__parameters:
    use_sim_time: True
    # Overridden in launch by the "map" launch configuration or provided default value.
    # To use in yaml, remove the default "map" value in the tb3_simulation_launch.py file & provide full path to map below.
    yaml_filename: "/home/taeram/Documents/map/S1_map.yaml"
    topic_name: "map"

filter_mask_server:
  ros__parameters:
    yaml_filename: "/home/taeram/Documents/map/S1_map.yaml"
    topic_name: "/filter_mask"
    frame_id: "map"
```

#### 01. Start only the Nav2 map server node and pass the map YAML path as a parameter
Terminal 1-1
```
$ ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=/home/taeram/Documents/map/S1_map.yaml
```

#### 02. Activate the map_server lifecycle
Terminal 2
```
$ ros2 lifecycle set /map_server configure
$ ros2 lifecycle set /map_server activate
```

#### 03. Run the Nav2 AMCL node (particle-filter localization) standalone
Terminal 1-2
```
$ ros2 run nav2_amcl amcl --ros-args   -p base_frame_id:=base_link   -p odom_frame_id:=odom   -p use_map_topic:=true   -p scan_topic:=/scan
```

#### 04. Activate the AMCL lifecycle
Terminal 2
```
$ ros2 lifecycle set /amcl configure
$ ros2 lifecycle set /amcl activate
```

#### 05. Launch only the Nav2 core navigation nodes
Terminal 1-3
```
$ ros2 launch nav2_bringup navigation_launch.py map:=/home/taeram/Documents/map/S1_map.yaml
```

#### 06. Open RViz2 with the Nav2 default layout
Terminal 1-4
```
$ ros2 run rviz2 rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz
```

#### 07. Publish the robot’s initial pose once to initialize localization
Terminal 3
```
$ ros2 topic pub /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{
  header: { 
    stamp: { sec: 0, nanosec: 0 },
    frame_id: 'map' 
  },
  pose: {
    pose: {
      position: { x: 9.5588, y: 26.8858, z: 0.0 },
      orientation: { x: 0.0, y: 0.0, z: 0.7071067811865476, w: 0.7071067811865475 }
    },
    covariance: [
      0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891909122467
    ]
  }
}" --once
```

#### 08. Send a navigation goal to Nav2’s BT Navigator via action
Terminal 3
```
$ ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{
  pose: {
    header: {                     
      frame_id: 'map'
    },
    pose: {
      position: {
        x: 9.04948616027832,
        y: 46.17757034301758,
        z: 0.0
      },
      orientation: {
        x: 0.0,                     
        y: 0.0,                     
        z: 0.8530794384041365,
        w: 0.521781057314352
      }                            
    }                                             
  }
}"
```
