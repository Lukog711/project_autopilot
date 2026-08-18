# Security and Safety Notice

## Scope

This repository contains a research prototype for a ROS 2 based UAV safety architecture.

The software is not certified for safety-critical or production flight operations.

## Reporting Issues

If you identify a security vulnerability or a condition that could cause unsafe system behavior, please report it privately to the repository maintainer rather than immediately publishing exploit details.

Include:

- affected component;
- software version;
- hardware platform;
- ROS 2 distribution;
- middleware configuration;
- steps required to reproduce the problem;
- expected behavior;
- observed behavior.

## Safety-Critical Limitations

The current implementation should not be considered an independent aircraft safety system.

The following limitations are known:

- fixed braking parameters are used in the current prototype;
- no certified actuator interface is provided;
- no formal verification is included;
- timing behavior depends on the operating system and hardware;
- DDS communication behavior depends on middleware configuration;
- LiDAR driver behavior depends on the selected hardware and driver;
- the watchdog cannot guarantee detection of every possible hardware or software fault.

Users are responsible for independently validating the system before any physical deployment.

## Emergency Use

Do not rely on this software as the sole mechanism for avoiding collisions or protecting people, aircraft, or property.
