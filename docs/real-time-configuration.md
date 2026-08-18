# Linux Real-Time Configuration

## Scheduling

The prototype uses Linux real-time scheduling mechanisms.

Critical workloads can use:

```text
SCHED_FIFO
```

while less critical workloads can use:

```text
SCHED_RR
```

CPU affinity is controlled using:

```text
taskset
```

## Why FIFO Is Not Used Everywhere

A high-priority FIFO task can prevent lower-priority tasks from receiving sufficient CPU time.

During Raspberry Pi 5 testing, combining strict CPU affinity with FIFO scheduling caused severe system starvation.

This was reproduced with multiple video workloads.

The benchmark was therefore changed to use:

```text
Critical publisher → SCHED_FIFO
Non-critical AI → SCHED_RR
```

This preserves high priority for the safety-relevant workload while allowing the non-critical workload to share CPU resources.

## PREEMPT_RT

The project was tested with a Linux real-time kernel environment.

PREEMPT_RT reduces scheduling latency and improves determinism compared with a conventional general-purpose kernel.

However, PREEMPT_RT does not by itself guarantee hard real-time behavior.

Application-level timing, scheduling configuration, memory allocation, middleware behavior, interrupt handling, and hardware characteristics must also be considered.

Important

Real-time priorities and CPU affinity are platform-dependent.

The values used in the prototype should not automatically be transferred to another system without testing.
