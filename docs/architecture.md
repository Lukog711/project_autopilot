# Safety Architecture


## Design Principle


The architecture separates perception, physical safety calculation, system health monitoring, and decision making.


The purpose is to avoid coupling the safety decision directly to the high-bandwidth perception pipeline.


## Data Reduction


The LiDAR produces a large point-cloud stream.


Instead of forwarding the complete cloud through the safety decision path, the perception node reduces the cloud to:


```text
closest obstacle distance
```
This creates a low-bandwidth safety signal.

Safety Decision

The decision compares:

```text
Obstacle distance
```

against:

```text
Required stopping distance
```

The resulting state is:

```text
CLEAR
DANGER
FAULT
```

Both DANGER and FAULT result in the safety response:

```text
BRAKE
```

Health Monitoring

Health monitoring is independent from the main decision calculation.

The watchdog monitors:

ROS 2 publisher liveliness;
sensor-data freshness;
LiDAR node presence;
Iceoryx RouDi process state.
Why Multiple Monitoring Mechanisms?

ROS 2 QoS liveliness can indicate that a publisher is no longer considered alive.

However, liveliness does not prove that useful sensor data is being generated.

Therefore:

```text
QoS liveliness
+
data freshness
+
process monitoring
```

provide complementary failure-detection mechanisms.
