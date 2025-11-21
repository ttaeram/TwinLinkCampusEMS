# Twin Link CampusEMS – Project Description

## 1. Service Overview

Modern university campuses already deploy numerous CCTV cameras, some patrol robots, and various control/monitoring systems. However, day-to-day operations are still heavily based on a human security guard sitting in front of multiple monitors, manually watching the video feeds.

This project started from a very simple question:

> “What if a security guard could just speak a simple command while watching CCTV, and send a robot to the scene – and then watch the result both in the real campus and in Meta-Sejong?”

---

## 2. Things Used in This Project

### 2-1. IoT Platform

- **Mobius**: oneM2M-based IoT platform  
- **MQTT/HTTP Protocol**: messages and event transmission between entities

### 2-2. Robotics & Simulation

- **ROS2 Humble**: robot control and topic/service/action-based data handling  
- **Nav2**: path planning and autonomous robot navigation  
- **Slam Toolbox**: real-time mapping and localization  
- **Isaac Sim**: synchronization tests between simulated and real robots

### 2-3. AI & Computer Vision

- **YOLOv8**: CCTV video analysis and object detection  
- **Whisper**: speech-to-text model

### 2-4. Development Environment

- Linux **Ubuntu 22.04**  
- **macOS**  
- **GitHub**  
- **Python 3.10.12**  
- **JavaScript**  
- **HTML, CSS**

---

## 3. Mobius Resource Design

### 3-1. Design Philosophy of the Resource Tree

In this project, we used a pre-Mobius4 stable version of Mobius as the platform, but the resource tree design was strongly inspired by the architecture and philosophy of Mobius4.

Our main design principles were:

1. **Separation of model information and runtime data**  
   For each robot, CCTV camera, sensor, and Meta-Sejong object, we separated:
   - **Model-level information**: capabilities, configuration, static attributes  
   - **Runtime data**: status, events, logs, images  

   For example, a robot’s static properties (model name, sensor set, etc.) are decoupled from dynamic information (position, battery, command history).

2. **Role-based AEs & consistent CNT naming**  
   Under `/CampusEMS/...`, we defined role-based AEs such as:
   - `ControlCenter`  
   - `PatrolRobot`  
   - `CCTV`  
   - `MetaSejong`  

   Each AE contains function-based containers (CNTs) like:
   - `Command`  
   - `Status`  
   - `Image`  
   - `Log`  
   - `Event`  

   This structure helps developers and operators quickly understand the data flow just by looking at the resource browser.

3. **Prepared for future migration to Mobius4**  
   We proactively adopted the idea of separating model resources and data resources that is emphasized in Mobius4.  
   As a result, once Mobius4 is mature and ready for production in our context, we expect to reuse most of the current resource tree as-is, with minimal structural changes.

In short, while we use a stable legacy version of Mobius today, the resource tree is already designed with a **“Mobius4 mindset.”**

### 3-2. Containers

(Figures are placeholders for screenshots.)

- **Fig.01.** CampusEMS AE and its CNTs – `Screenshot 2025-11-17 at 04.20.04.png`  
- **Fig.02.** `ControlCenter` CNT – `Screenshot 2025-11-17 at 05.18.44.png`  
- **Fig.03.** `CCTV` CNT – `Screenshot 2025-11-17 at 05.19.31.png`  
- **Fig.04.** `Robots` CNT – `Screenshot 2025-11-17 at 05.19.58.png`  
- **Fig.05.** `ModelRepo` CNT – `Screenshot 2025-11-17 at 05.22.01.png`  
- **Fig.06.** `DatasetRepo` CNT – `Screenshot 2025-11-17 at 05.21.41.png`  

(Example image syntax:  
`![Fig.01 – CampusEMS AE and its CNTs](Screenshot-2025-11-17-04-20-04.png)`)

---

## 4. Meta-Sejong

### 4-1. Meta-Sejong Environment Overview

Meta-Sejong is a digital twin environment of Sejong University’s campus, built using **NVIDIA Isaac Sim**. We used this environment to simulate and validate robot navigation, mission execution, and task workflows.

Rather than creating a brand-new map, we leveraged an existing campus simulation map and adapted it to match our scenarios. We focused on **reusing and extending** the existing digital twin for CampusEMS use cases.

- Robots are configured to drive on the **road (car lanes)**, not on pedestrian sidewalks.  
- The map was slightly modified to align with the scenario requirements.

Through this setup:

- Complex situations can be demonstrated directly inside the simulator.  
- At the same time, the real robot performs the **same behavior**, so the simulation and the real world stay synchronized and operate as **one integrated system**.

### 4-2. Chungmu Hall Area

`S1_v2.png`  
**Fig.07. Meta-Sejong Chungmu Hall**

The Chungmu Hall area is modeled as a region with a park and two buildings. We use this area to demonstrate synchronized mission execution between the simulator and real robots, including:

- When the guard issues the command:  
  > “Robot 01, move to the main gate of Chungmu Hall,”  
  the robot navigates to the specified destination.  
- After completing its task, the robot automatically returns to its initial position.  
- During navigation, the robot drives safely while considering the terrain around the park and buildings.

This area is the most intuitive region to showcase how the robot’s movement and missions remain synchronized between the simulation and the real campus.

### 4-3. Yongdeok Hall Area

`Screenshot from 2025-11-17 15-25-50.png`  
**Fig.08. Meta-Sejong Yongdeok Hall**

The area in front of Yongdeok Hall is modeled as a **parking lot environment**. It reflects a cluttered, realistic scene with many parked vehicles acting as obstacles. Here we focus on obstacle avoidance synchronization between the simulator and the real robot:

- The simulated robot moves toward its goal while avoiding parked vehicles.  
- The same avoidance path (or an equivalent Nav2-generated path toward the same goal) is then sent to the real robot.  
- The real robot navigates through the actual parking lot, avoiding obstacles in a similar manner.  
- After the mission is completed, both the simulated robot and the real robot return to their initial positions.

This region is used to verify that **behavioral consistency** and **obstacle avoidance strategies** are maintained between the simulation and the real environment, even in complex, cluttered scenes.

---

## 5. Mobius-based Feature Overview

### 5-1. Campus-wide Resource Hub

`Screenshot 2025-11-17 at 06.12.52.png`  
**Fig.09. Several raw data items**

CampusEMS is represented as a single logical tree in Mobius.

The control center, robots, CCTV cameras, Meta-Sejong, and model/data repositories are all mapped to AEs/containers under the same hierarchy.

Example structure:

```text
CampusEMS/ControlCenter/...
CampusEMS/Robots/Robot01/...
CampusEMS/CCTV/Camera01/...
CampusEMS/ModelRepo/...
CampusEMS/DatasetRepo/...
```

All other features in the system are built on top of this **integrated resource hub**.

### 5-2. Voice-driven Robot Dispatch Pipeline

`Screenshot 2025-11-17 at 06.14.07.png`  
**Fig.10. Voice to Navigation**

A single spoken sentence from the manager is transformed into a robot navigation command, and every step of this pipeline is handled through Mobius resources.

#### 5-2-1. Voice collection (`VoiceRawData`)

When the manager speaks into the microphone, the encoded audio file is stored at:

```text
CampusEMS/ControlCenter/Manager/VoiceRawData
```

This container acts as the repository for **raw voice data**.

#### 5-2-2. STT conversion & command matching

- The STT model reads from `VoiceRawData` and converts the audio into text.  
- The intent parser then analyzes the text to determine:
  - which robot should move (e.g., `Robot01`, `Robot07`),  
  - where it should go (e.g., main gate of Chungmu Hall, Yongdeok Hall),  
  - what type of task it should perform (identity check, lost-and-found pickup, etc.).

#### 5-2-3. Creating robot navigation commands

`Screenshot 2025-11-17 at 14.17.50.png`  
**Fig.11. STT to navigation command**

The final structured command is written as a `cin` into the target robot’s navigation container:

```text
CampusEMS/Robots/Robot01/Control/Nav
```

The `Nav` container includes destination coordinates, options, and task metadata.

#### 5-2-4. Sub-based execution

- The robot’s navigation module subscribes to `CampusEMS/Robots/Robot01/Control/Nav`.  
- Whenever a new `cin` appears, it detects the event and immediately triggers its navigation logic.

This completes the pipeline:

> Guard’s voice → STT → intent matching → `Nav` container → robot movement

with Mobius acting as the **glue** at every stage.

### 5-3. Synchronization between Meta-Sejong and the Real World

Robot navigation is designed around Meta-Sejong as the **primary reference**.

- Meta-Sejong provides a digital twin of the campus – terrain, buildings, and key points of interest.  
- Target locations such as the main gate of regions are defined in the **coordinate system of Meta-Sejong**.  
- The real robots follow the Meta-Sejong-based coordinates encoded in the `Nav` container, so that their motion in the physical campus closely mirrors the behavior of their avatars inside Meta-Sejong.

This allows us to experiment with **Twin Link navigation**:

> “Plan and visualize a robot’s path in Meta-Sejong, then execute essentially the same behavior in the real world.”

### 5-4. Event Subscription and Automation

We leverage Mobius subscription/notification to react instantly to key events.

- **Voice input events**  
  A subscription on `CampusEMS/ControlCenter/Manager/VoiceRawData` ensures that as soon as a new audio file is created, the STT pipeline is triggered.

- **Robot navigation command events**  
  Subscriptions on `CampusEMS/Robots/.../Control/Nav` notify the corresponding robot nodes whenever a new command is written, so the navigation module starts immediately.

In other words:

- by subscribing to `ControlCenter/~/VoiceRawData` and `Robots/~Nav`,  
- we build an event-driven automation pipeline where:
  - “new voice → instant interpretation”  
  - “new command → instant robot execution”.

(Here, `~` is used as a shorthand to denote “all `VoiceRawData` under `ControlCenter`” and “all `Nav` containers under `Robots`.”)

### 5-5. Relationship and Intent toward Mobius4

`Screenshot 2025-11-17 at 06.16.09.png`  
**Fig.12. Mobius4 Utils**

Although the current system runs on a pre-Mobius4 version of Mobius, our resource structure intentionally reflects the Mobius4 concept of separating **models** and **data**.

- Each robot AE contains a `ModelDeployment` resource with `DetectionModel` describing the detection/recognition model used by that robot.  
- `CampusEMS/ControlCenter/Manager/ModelDeployment` contains `STTModel` which describes the STT configuration used by the control center.

System-wide models and datasets are organized under:

```text
CampusEMS/ModelRepo/...
├─ STTModel
└─ DetectionModel

CampusEMS/DatasetRepo/...
└─ (various datasets for training/testing)
```

This means:

- **Models** are represented via `ModelDeployment` and `ModelRepo`.  
- **Data** is stored in each AE’s containers and in `DatasetRepo`.

This design deliberately mirrors the **model/data separation** promoted in Mobius4, even though we are currently running on a previous version.

In short, while we did not deploy Mobius4 itself for this project, the presence of `ModelDeployment`, `ModelRepo`, `DatasetRepo`, and model-related resources clearly shows our intention to adopt Mobius4 concepts and to enable a smooth future migration when the time and environment are right.

---

## 6. Demo Video

**Twin Link CampusEMS**  
*(Insert demo video link or embed here.)*

---

## 7. Expectation Effectiveness

### 7-1. Enhanced Campus Safety – Human-in-the-loop with Robot Support

In the current design, robots do not make autonomous security decisions. Instead, the security manager monitors the situation and dispatches robots remotely, so humans stay in full control while robots handle on-site checks.

Since Mobius centrally manages campus events (video, commands, status data), the system can quickly identify safety-related situations and send a robot to the scene for fast verification.

### 7-2. Shorter Response Time – One Voice Command to Dispatch a Robot

Traditionally, once a guard found something suspicious on CCTV, they had to leave the control room, call other staff, or use multiple channels (phone, radio, external reporting).

Now:

- All events are delivered through Mobius’ subscription-based notifications.  
- The guard can dispatch a robot with a **single voice command** without leaving the desk.

This significantly reduces the delay between **decision and action**.

### 7-3. Reduced Workload and More Efficient Operations

Routine tasks such as patrols, lost-and-found handling, and minor situation checks can be performed by robots instead of requiring physical movement by the guard.

This allows security staff to stay at the control center and focus on truly critical incidents, improving both:

- workload balance, and  
- overall operational efficiency.

### 7-4. Unified IoT Architecture for a Smart Campus

The project establishes a campus-wide IoT architecture where:

- web services,  
- robots,  
- AI modules, and  
- the simulator (Isaac Sim)  

are all integrated around **Mobius (CampusEMS)**.

On top of this architecture, we can gradually add:

- ML-based abnormal event detection,  
- more advanced STT and natural language understanding models,  
- semi-autonomous or fully autonomous security patrol scenarios.

This creates a clear migration path from **human-driven operations today** to **AI-assisted or AI-driven campus security in the future**.

### 7-5. Sim-to-Real Integration for Faster Development and Safer Testing

Since simulated robots in Isaac Sim and real robots share the same Mobius command interface, we can:

- perform function tests,  
- verify paths, and  
- replay scenarios in simulation first,  

and then apply almost the same setup to real robots.

This approach:

- accelerates development, and  
- reduces both the **cost** and **risk** of field testing.

### 7-6. Foundation for Data-driven Decision Making and Policy Design

All events (voice commands, robot trajectories, images, incident logs, etc.) are stored in Mobius, enabling:

- statistics on high-risk areas and time periods,  
- analysis of security patterns over time,  
- optimization of patrol routes,  
- data-driven improvement of campus safety policies.  

The accumulated data can also serve as training data for future AI-based security models, supporting a natural transition from:

- **human-controlled operations** to  
- **AI-assisted, partially autonomous guarding**.
