# ROS 2 UAV Safety Reflex Architecture

A hard real-time capable ROS 2 safety architecture for autonomous flying robots using LiDAR point clouds, deterministic safety logic, Linux real-time scheduling, and shared-memory communication.

This project was developed as part of a Master's project thesis investigating the design and implementation of a safety-oriented ROS 2 architecture for autonomous flying robots.

The central idea is to separate **safety-critical reflexes** from higher-level autonomous perception and to make the safety decision dependent on both:

1. the physical stopping capability of the UAV, and
2. the validity and freshness of the sensor and communication data.

The architecture follows a fail-safe principle:

> **Missing evidence is treated as a fault. A fault causes the system to enter a braking state.**

The implementation uses ROS 2 Jazzy, Eclipse CycloneDDS, Eclipse Iceoryx shared-memory infrastructure, Linux real-time scheduling, CPU affinity, QoS liveliness monitoring, and a Livox MID-360 LiDAR.

---

# Project Overview

Autonomous aerial robots must react to obstacles within a limited physical stopping distance. A perception system can identify an obstacle correctly but still fail to provide sufficient safety if the information is delayed, stale, or lost.

This project therefore focuses on the complete safety path:

```text
LiDAR Point Cloud
       │
       ▼
closest_obstacle_node
       │
       │ closest obstacle distance
       ▼
safe_braking_node
       │
       │ required stopping distance
       ▼
Decision Logic ◄──── health_monitor_node
       │
       ├──────────────► CLEAR
       │
       ├──────────────► DANGER → BRAKE
       │
       └──────────────► FAULT  → BRAKE
```

The architecture is implemented as a modular ROS 2 node graph so that individual functions can be isolated, monitored, and assigned different execution priorities.

---

# Main Features

* ROS 2 Jazzy based safety architecture
* Livox MID-360 LiDAR point-cloud processing
* Stale LiDAR frame rejection
* Closest-obstacle distance extraction
* Predictive braking-distance calculation
* Deterministic safety decision logic
* ROS 2 DDS QoS reliability and liveliness monitoring
* Independent health-monitoring node
* Iceoryx RouDi daemon monitoring
* Linux real-time scheduling using `SCHED_FIFO` and `SCHED_RR`
* CPU affinity using `taskset`
* CycloneDDS shared-memory configuration
* Fast DDS UDP vs CycloneDDS shared-memory benchmarking
* 720p/30 FPS video transport benchmark
* YOLO-based object-detection benchmark
* Transport-latency and frame-drop measurement
* Raspberry Pi 5 target platform
* Ubuntu 24.04 / ROS 2 Jazzy environment

---

# System Architecture

The architecture consists of four primary safety nodes and the LiDAR driver.

```text
                         ┌──────────────────────────┐
                         │     Livox MID-360 LiDAR  │
                         └────────────┬─────────────┘
                                      │
                                      │ Point Cloud
                                      ▼
                         ┌──────────────────────────┐
                         │ closest_obstacle_node    │
                         │                          │
                         │ • stale-frame rejection  │
                         │ • point filtering        │
                         │ • Euclidean distance     │
                         └────────────┬─────────────┘
                                      │
                                      │ closest_distance
                                      ▼
                         ┌──────────────────────────┐
                         │ safe_braking_node        │
                         │                          │
                         │ • reaction distance      │
                         │ • braking distance       │
                         │ • safety threshold       │
                         └────────────┬─────────────┘
                                      │
                                      │ braking_distance
                                      ▼
                         ┌──────────────────────────┐
                         │       decision_node      │
                         │                          │
                         │ CLEAR / DANGER / FAULT   │
                         └────────────▲─────────────┘
                                      │
                                      │ system_health
                                      │
                         ┌────────────┴─────────────┐
                         │    health_monitor_node   │
                         │                          │
                         │ • QoS liveliness         │
                         │ • data timeout           │
                         │ • LiDAR node monitoring  │
                         │ • RouDi monitoring       │
                         └──────────────────────────┘
```

---

# Core Nodes

## `closest_obstacle_node`

The node receives the LiDAR point cloud and reduces the high-bandwidth perception stream to a single safety-relevant scalar.

Its main responsibilities are:

* receiving the LiDAR point cloud;
* rejecting point-cloud messages older than the configured freshness threshold;
* calculating Euclidean distance for valid points;
* rejecting points below the minimum valid distance;
* determining the closest valid obstacle;
* publishing the resulting distance.

The Euclidean distance is calculated as:

```text
d = √(x² + y² + z²)
```

Only the minimum valid distance is forwarded to the safety layer.

The implementation currently uses a 200 ms point-cloud freshness threshold.

---

## `safe_braking_node`

This node calculates the stopping distance used by the safety decision.

The current implementation uses a fixed test velocity and deceleration:

```text
d_stop = v · t_r + v² / (2a)
```

where:

* `v` is the current/test velocity;
* `t_r` is the assumed system reaction time;
* `a` is the available braking deceleration.

The present implementation uses fixed values for these parameters for the experimental prototype.

### Planned extension

For a flight-ready implementation, the fixed velocity should be replaced by live state-estimator data obtained from the flight controller or another validated velocity-estimation source.

Possible sources include:

* flight-controller odometry;
* visual-inertial odometry;
* LiDAR-inertial odometry;
* fused state estimation.

The braking threshold should then be updated at a rate sufficiently high that acceleration occurring between two updates cannot invalidate the previously calculated safety distance.

---

## `health_monitor_node`

The health monitor provides an independent monitoring layer.

It combines several mechanisms because a single communication mechanism cannot detect every failure mode.

### QoS liveliness

The node monitors the liveliness of critical ROS 2 publishers.

A publisher disappearing from the ROS graph can therefore be detected independently of the actual data content.

### Data freshness monitoring

Liveliness alone does not guarantee that valid sensor data is arriving.

The watchdog therefore also checks whether LiDAR-related data continues to arrive within the configured timeout.

The current implementation uses a 500 ms starvation threshold.

### Iceoryx/RouDi monitoring

The Iceoryx RouDi daemon is not monitored through ROS 2 QoS because RouDi is an external operating-system process rather than a ROS 2 publisher.

The implementation therefore performs an asynchronous `/proc` process check.

This check is deliberately asynchronous so that process inspection does not unnecessarily block the safety-monitoring timer.

---

## `decision_node`

The decision node combines:

* closest-obstacle distance;
* required braking distance;
* health-monitor status.

The resulting state is conceptually:

```text
                    ┌──────────────┐
                    │ System fault │
                    └──────┬───────┘
                           │
                           ▼
                        BRAKE
                           ▲
                           │
             ┌─────────────┴──────────────┐
             │                            │
      obstacle ≤ threshold        obstacle > threshold
             │                            │
             ▼                            ▼
           BRAKE                         CLEAR
```

The current implementation displays the result through an OpenCV-based status interface.

The ROS execution and UI processing are separated using a dedicated thread so that graphical processing is not directly coupled to ROS callback execution.

---

# Safety Philosophy

The architecture follows a fail-safe approach.

The important distinction is between:

**Evidence that the system is safe**

and

**absence of evidence that the system is unsafe.**

The architecture does not treat missing sensor or health information as permission to continue flying.

Instead:

```text
Valid sensor data
       +
Valid braking threshold
       +
Healthy communication
       +
Healthy middleware
       ↓
     CLEAR
```

If one of the required conditions becomes invalid:

```text
Missing / stale / failed information
              ↓
            FAULT
              ↓
            BRAKE
```

This approach is particularly important for safety-critical autonomous systems where communication failure can otherwise become indistinguishable from a safe operating condition.

---

# Real-Time Scheduling

The implementation uses Linux real-time scheduling mechanisms.

Critical tasks can be executed using:

```text
SCHED_FIFO
```

while less critical tasks can use:

```text
SCHED_RR
```

CPU affinity is controlled using:

```text
taskset
```

The intention is to reduce scheduling interference between critical sensor-processing tasks and computationally expensive non-critical workloads.

## Important Raspberry Pi 5 Finding

During the experiments, assigning both:

* strict CPU affinity, and
* high-priority `SCHED_FIFO`

to the critical publisher while simultaneously running other workloads caused severe system starvation on the Raspberry Pi 5.

The problem persisted with different video workloads, including:

* approximately 400 MB, 720p, 30 FPS, ten-minute H.264 video;
* a lightweight approximately 6 MB, ten-second video;
* 1080p, 60 FPS, 30-second video.

The observation indicates that the issue was not simply caused by the size or duration of the video.

The scheduling configuration was therefore changed so that:

* critical tasks retain `SCHED_FIFO`;
* non-critical computational workloads use `SCHED_RR`;
* CPU affinity remains controlled where appropriate.

This configuration provided a better compromise between real-time prioritization and overall system responsiveness.

---

# Benchmarking

A separate benchmarking package evaluates the communication path used for large image messages.

Two configurations are compared:

### Network configuration

```text
ROS 2
  │
  ▼
Fast DDS
  │
  ▼
UDP transport
  │
  ▼
YOLO subscriber
```

### Shared-memory configuration

```text
ROS 2
  │
  ▼
CycloneDDS
  │
  ▼
Iceoryx shared memory
  │
  ▼
YOLO subscriber
```

The benchmark publishes a video stream and measures:

* transport latency;
* frame drops;
* inference latency;
* communication liveliness.

The benchmark uses a queue depth of one so that old frames are discarded rather than accumulating latency.

This reflects the requirements of real-time perception systems where a recent frame is generally more valuable than an old frame.

---

# Benchmark Metrics

The YOLO subscriber records:

```text
System time
Publisher frame ID
Transport latency
Network drops
```

Transport latency is calculated from the publisher timestamp:

```text
T_transport = T_arrival − T_publication
```

Frame drops are estimated using consecutive publisher frame identifiers:

```text
N_drops = ID_i − ID_(i−1) − 1
```

Inference latency is measured independently around the YOLO inference operation.

Therefore, communication latency and AI-processing latency can be examined separately.

---

# Repository Structure

```text
.
├── README.md
├── LICENSE
├── .gitignore
├── SECURITY.md
│
├── docs/
│   ├── architecture.md
│   ├── real-time-configuration.md
│   ├── lidar-setup.md
│   └── benchmarking.md
│
├── autopilot/
│   ├── package.xml
│   ├── CMakeLists.txt
│   ├── launch/
│   │   └── autopilot.launch.py
│   └── src/
│       ├── closest_obstacle_node.cpp
│       ├── safe_braking_node.cpp
│       ├── health_monitor_node.cpp
│       └── decision_node.cpp
│
└── zerocopy_ros/
    ├── package.xml
    ├── CMakeLists.txt
    ├── launch/
    │   └── benchmark.launch.py
    └── src/
        ├── video_pub.cpp
        └── yolo_sub.py
    
```

---

# Requirements

## Hardware

The prototype was developed and tested on:

* Raspberry Pi 5
* Livox MID-360 LiDAR
* Linux-based edge-computing environment

The architecture is not fundamentally restricted to the MID-360. Another LiDAR can be used provided that an appropriate ROS 2 driver publishes a compatible point-cloud message.

---

# Software

Recommended environment:

* Ubuntu 24.04
* ROS 2 Jazzy Jalisco
* C++17
* Python 3
* OpenCV
* Eclipse CycloneDDS
* Eclipse Iceoryx
* Livox ROS Driver 2 for the MID-360
* Ultralytics YOLO for the benchmark

---

# Installation

## 1. Install Ubuntu 24.04

Install Ubuntu 24.04 on the target computer.

For Raspberry Pi 5, use the appropriate Ubuntu 24.04 Raspberry Pi image.

---

# 2. Install ROS 2 Jazzy

Follow the official ROS 2 Jazzy installation instructions:

https://docs.ros.org/en/jazzy/Installation.html

After installation:

```bash
source /opt/ros/jazzy/setup.bash
```

It is recommended to add the command to the shell configuration:

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
```

---

# 3. Install Development Dependencies

```bash
sudo apt update

sudo apt install -y \
    build-essential \
    cmake \
    git \
    python3-pip \
    python3-colcon-common-extensions \
    libopencv-dev \
    python3-opencv
```

---

# 4. CycloneDDS

The project uses Eclipse CycloneDDS as the ROS 2 middleware implementation.

The CycloneDDS source repository is available at:

https://github.com/eclipse-cyclonedds/cyclonedds

Clone the repository:

```bash
cd ~

git clone https://github.com/eclipse-cyclonedds/cyclonedds.git

cd cyclonedds
```

Build according to the official CycloneDDS instructions.

After installation, ROS 2 should provide:

```text
rmw_cyclonedds_cpp
```

The safety launch file selects CycloneDDS using:

```bash
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

---

# 5. Iceoryx

The shared-memory configuration uses Eclipse Iceoryx infrastructure and the RouDi daemon.

The important runtime component is:

```text
iox-roudi
```

RouDi manages the shared-memory resources used by Iceoryx.

The exact Iceoryx installation method should follow the version compatible with the installed CycloneDDS/Iceoryx integration.

Before running the shared-memory benchmark, verify:

```bash
which iox-roudi
```

and:

```bash
iox-roudi
```

---

# 6. Real-Time Kernel

The project uses Linux real-time scheduling features and was tested with a PREEMPT_RT-enabled environment.

On Ubuntu, Canonical provides real-time kernel support through Ubuntu Pro.

For supported Ubuntu installations, consult:

https://ubuntu.com/real-time

On Raspberry Pi, ensure that the real-time kernel variant is appropriate for the installed Ubuntu release and Pi platform.

After installing the real-time kernel, verify the scheduler configuration:

```bash
cat /proc/sys/kernel/sched_rt_runtime_us
```

A non-zero value indicates that real-time runtime throttling is enabled.

The launch file checks this value before attempting to apply the real-time execution configuration.

---

# 7. LiDAR Driver

The safety architecture is demonstrated with the Livox MID-360.

For the MID-360, use the official Livox ROS Driver 2:

https://github.com/Livox-SDK/livox_ros_driver2

The driver depends on Livox SDK2:

https://github.com/Livox-SDK/Livox-SDK2

Clone the SDK:

```bash
cd ~/ros2_ws/src

git clone https://github.com/Livox-SDK/Livox-SDK2.git
```

Build and install the SDK following the official instructions.

Then clone the ROS 2 driver:

```bash
cd ~/ros2_ws/src

git clone https://github.com/Livox-SDK/livox_ros_driver2.git
```

For ROS 2 Jazzy, follow the driver's Jazzy-specific build instructions.

The exact driver configuration depends on the LiDAR model and network configuration.

---

# Using Another LiDAR

The architecture is not intrinsically dependent on Livox.

If another LiDAR is used:

1. install the appropriate ROS 2 driver;
2. configure the driver to publish a point-cloud message;
3. adapt the subscription type if necessary;
4. remap the point-cloud topic;
5. verify timestamp quality;
6. verify QoS compatibility.

The safety architecture should not assume that every LiDAR driver has the same QoS settings, message format, timestamp semantics, or liveliness behavior.

---

# 8. Create the ROS 2 Workspace

```bash
mkdir -p ~/ros2_ws/src

cd ~/ros2_ws/src
```

Clone this repository:

```bash
git clone https://github.com/Lukog711/project_autopilot
```

Then:

```bash
cd ~/ros2_ws

source /opt/ros/jazzy/setup.bash
```

---

# 9. Build

Build the safety package:

```bash
colcon build --packages-select autopilot --symlink-install
```

For the benchmarking package:

```bash
colcon build --packages-select zerocopy_ros --symlink-install
```

Source the workspace:

```bash
source ~/ros2_ws/install/setup.bash
```

---

# 10. CycloneDDS Shared-Memory Configuration

The launch file expects the CycloneDDS configuration file:

```text
~/cyclonedds_shm.xml
```

The configuration must be created according to the installed CycloneDDS and Iceoryx versions.

Do not copy a configuration intended for a different CycloneDDS release without verifying compatibility.

The launch file sets:

```text
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

and:

```text
CYCLONEDDS_URI=file://~/cyclonedds_shm.xml
```

through the corresponding absolute path.

---

# Launching the Safety Architecture

After sourcing the workspace:

```bash
source ~/ros2_ws/install/setup.bash
```

Launch:

```bash
ros2 launch autopilot autopilot.launch.py
```

The launch file:

* configures CycloneDDS;
* starts the Iceoryx RouDi daemon;
* starts the LiDAR driver;
* starts the obstacle-processing node;
* starts the braking-distance node;
* starts the health monitor;
* starts the decision/UI node;
* applies real-time scheduling configuration when the required privileges and kernel configuration are available.

---

# Real-Time Scheduling Configuration

The launch file uses:

```text
taskset
```

for CPU affinity and:

```text
chrt
```

for real-time scheduling.

The intended configuration separates tasks according to criticality.

Critical sensor-processing work can use:

```text
SCHED_FIFO
```

while less critical computational workloads can use:

```text
SCHED_RR
```

The exact priorities and CPU assignments should be treated as platform-specific configuration rather than universal values.

---

# Important Experimental Finding

The Raspberry Pi 5 experiments demonstrated that aggressive real-time scheduling can itself become a system-level failure mechanism.

Using:

```text
SCHED_FIFO
+
strict CPU affinity
```

for a critical publisher while simultaneously executing other workloads caused system starvation/freezing.

The behavior was reproduced with multiple video workloads.

Consequently, the benchmark configuration was changed so that the critical publisher retained elevated FIFO priority while the non-critical AI workload used round-robin scheduling.

This demonstrates an important limitation of real-time systems:

> Increasing the priority of a task does not automatically increase the reliability of the complete system.

Real-time scheduling must be configured together with CPU availability, workload criticality, kernel behavior, and system-wide resource constraints.

---

# Benchmarking

The repository contains a separate benchmark for comparing communication mechanisms.

The benchmark consists of:

```text
Video Publisher
      │
      │ 720p / 30 FPS
      ▼
ROS 2 Middleware
      │
      ▼
YOLO Subscriber
```

Two configurations are tested.

## Fast DDS / UDP

```text
Fast DDS
   ↓
 UDP
   ↓
Subscriber
```

## CycloneDDS / Shared Memory

```text
CycloneDDS
    ↓
 Iceoryx
    ↓
Shared Memory
    ↓
Subscriber
```

The benchmark records transport latency and frame drops.

The YOLO processing time is also measured but should be interpreted separately from transport performance.

---

# Benchmark Video

The main benchmark used a:

* 720p H.264 video;
* 30 FPS;
* approximately 400 MB;
* ten-minute duration.

Additional videos were used to investigate the observed Raspberry Pi starvation issue.

The smaller and shorter test video did not eliminate the problem, indicating that the observed freeze was not simply caused by the duration or storage size of the benchmark video.

---

# Benchmark Output

The benchmark produces:

```text
network_metrics.csv
```

with:

```text
System_Time
Publisher_Frame_ID
Transport_Latency_ms
Network_Drops
```

The data can be analyzed using Python, MATLAB, Excel, or other statistical tools.

---

# Results

The experimental data should be interpreted as a platform-specific comparison rather than a universal statement that one DDS implementation is always faster.

The benchmark compares the tested:

```text
Fast DDS + UDP
```

configuration against:

```text
CycloneDDS + shared memory / Iceoryx
```

configuration under the same experimental workload.

The recorded latency and frame-drop measurements are used to evaluate whether shared-memory communication provides a measurable advantage for large image messages.

---

# Safety Limitations

This repository is a research prototype and should **not** be used as the sole flight-safety mechanism of an operational UAV.

In particular:

* the current braking node uses fixed velocity and deceleration parameters;
* no certified flight-control interface is implemented;
* no actuator-level emergency braking command is implemented;
* obstacle geometry is reduced to a closest-point distance;
* the system does not perform complete collision prediction;
* the braking model does not currently include live acceleration;
* no formal safety certification is provided;
* real-time behavior depends on the underlying kernel and hardware;
* ROS 2 and middleware behavior depends on configuration and implementation versions.

The architecture therefore demonstrates a safety-oriented software design rather than a certified airborne safety system.

---

# Future Development

Potential extensions include:

1. integration with flight-controller odometry;
2. live velocity and acceleration inputs;
3. adaptive braking-distance calculation;
4. directional danger zones instead of a spherical distance threshold;
5. validated emergency-braking commands;
6. hardware-in-the-loop testing;
7. actuator-response measurement;
8. formal timing analysis;
9. WCET analysis;
10. deadline monitoring;
11. fault-injection testing;
12. redundant sensing;
13. redundant safety processors;
14. deterministic memory allocation;
15. further Iceoryx zero-copy optimization;
16. comparison with other DDS implementations;
17. testing on additional embedded platforms.

---

# Thesis

This repository accompanies a Master's project thesis on:

**Design and implementation for a hard real-time capable ROS 2 based safety architecture for flying robots using LiDAR point clouds via shared memory**

The thesis investigates the interaction between:

* autonomous perception;
* physical stopping distance;
* ROS 2 communication;
* DDS QoS;
* shared-memory communication;
* Linux real-time scheduling;
* watchdog mechanisms;
* system health monitoring;
* embedded computing constraints.

---

# License

This project is released under the Apache License 2.0.

See `LICENSE` for the complete license text.

---

# Disclaimer

This project is intended for research and educational purposes.

No guarantee is provided that the software is suitable for autonomous flight, safety-critical deployment, or operation near people or property.

Any deployment on an aircraft must include an independent safety assessment and appropriate hardware-level failsafe mechanisms.
