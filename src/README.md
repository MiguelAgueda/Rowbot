# Software Overview of Rowbot


## System Requirements

| Requirement | Weight | Description                                                             |
|:-----------:|:------:|:------------------------------------------------------------------------|
| REQ-1       | 10     | Localization system accurately tracks motion.                           |
| REQ-2       | 10     | Mapping system accurately generates map of environment.                 |
| REQ-3       | 8      | Mapping system accurately detects known environments at initialization. |
| REQ-4       | 7      | System can be controlled remotely.                                      |
| REQ-5       | 4      | System saves sensor data to disk for later analysis.                    |
| REQ-6       | 4      | System saves motion data to disk for later analysis.                    |

---

## Localization

Rowbot uses an Error-State Extended Kalman Filter (ES-EKF). 
This is a variation of the Extended Kalman Filter (EKF) which estimates 
    state errors as opposed to directly estimating the state.
For an excellent overview of the EKF, see section 3.3 of 
    Probabilistic Robotics (Thrun, Burgard, Fox).

### Here is a high-level overview of the EKF implemented in [`ekfmodule`](ekfmodule.cpp).

[<img src=../assets/software/EKFProcess.jpeg height=500>]()


### The Iterative Point Cloud (ICP)

The ICP algorithm processes sequential LiDAR range scans in order to provide 
    movement estimates over time. 
As the name suggests, the ICP algorithm estimates motion by aligning point cloud data.

### Here is an overview of the ICP process implemented in [`icpmodule`](icpmodule.cpp).

<img src=../assets/software/ICPProcess.png height=400>
