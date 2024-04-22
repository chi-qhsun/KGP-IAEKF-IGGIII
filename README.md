# KGP-IAEFKF-IGGIII

## Loose Coupling of GNSS/INS Using IA-EKF, IGGIII and KGP Methods Based on KF-GINS 

## Introduction

This paper expands on prior research by integratingcutting-edge adaptive algorithms, including IAE-KF, IGGIII, and KGP. This ensures increased accuracy and resilience in loosely connected GNSS/INS systems to improve estimate procedures and handle dynamic noise and non-linearities.

We also ported existed EKF integrated GNSS/INS navigation system: [KF-GINS](https://github.com/i2Nav-WHU/KF-GINS/) from C++ to Python for further development of Machine Learning Algorithm.

**Related Reference for KF-GINS:**

- X. Niu, Q. Chen, "[Notes for GNSS/INS Integrated Navigation and Principles of IMU Navigation](http://www.i2nav.com/index/newListDetail_zw.do?newskind_id=13a8654e060c40c69e5f3d4c13069078&newsinfo_id=40f3c65b158742c099ba3b600c983aa1)", i2Nav Lab, GNSS Research Center, Wuhan University, 2022
- X. Niu, Q. Chen, "[Lecture Recordings for GNSS/INS Integrated Navigation and Principles of IMU Navigation](https://www.bilibili.com/video/BV1na411Z7rQ?spm_id_from=333.999.0.0&vd_source=a417ebe0768fc96919fe8e34c55ed591)", i2Nav Lab, GNSS Research Center, Wuhan University， 2022

## 1 Program Compilation and Execution

### 1.1 Compilation environment

KGP-IAEKF-IGGIII is compiled on Windows.

The configuration file is used as a parameter to run Main.py after successfully compiling it. To debug the program, it is also required to add the configuration file as a parameter.

This project recommends using [Anaconda](https://www.anaconda.com/) for managing Python environments and dependencies. The following steps will guide you on how to set up and activate a conda environment suitable for this project.

Open your terminal or command prompt and create a new conda environment named kgp_env with Python version 3.8 using the following command:
```shell
conda create --name kgp-IAEKF python=3.8
conda activate kgp-IAEKF
conda install -c conda-forge absl-py pyswarm scikit-learn pyyaml pyquaternion
```
absl-py: A library from Google providing application-level infrastructure and various Python utilities.
pyswarm: A particle swarm optimization (PSO) library.
scikit-learn: A powerful Python module for machine learning and data mining.
pyyaml: A library for YAML file parsing and output.
pyquaternion: A library for handling rotations in three-dimensional space using quaternions.

### 1.2 Compile under Windows

We recommend you to compile in the VSCode software.

You should first install the Python compiler and VScode software (including the necessary plug-ins, such as Python, Python Extension Pack).

After preparing your own compilation environment, you can clone the repository locally and open the KGP-IAEKF-IGGIII folder in **VSCode**:

- Set Interpreter: open the Command Palette (Ctrl+Shift+P) and type "Python: Select Interpreter", select **Python 3.8.x (kgp-IAEKF)**

After setting your configuration, open Main.py in the folder,click run:

The Application will automatically run and generate result in expexted directory. 

Default path: ./NavResult

## 2 Use KGP-IAEKF-IGGIII

### 2.1 Prerequisite Knowledge

**Frame defination:**
- IMU frame: the origin is the IMU center, the three axes point forward-right-down
- Navigation reference frame: the origin coincides with the IMU frame, the three axes point north-east-down

**Navigation state:**
- position: the geodetic coordinates of the IMU position in the Earth frame (latitude-longitude-ellipsoid height)
- velocity: the IMU velocity to the Earth projected in the navigation reference frame (north speed-east speed-down speed)
- attitude: IMU attitude to navigation frame (quaternion, direction consine matrix, or euler angles. euler angles are defined as yaw-pitch-roll, ZYX rotation order)

**IMU noise model:**
- IMU measurement noises are modeled as the Gaussian white noise
- IMU bias errors are modeled as the firt-order Gauss-Markov process
- IMU scale factor errors are modeled as the firt-order Gauss-Markov process

For more details on the algorithm, please refer to [Notes for GNSS/INS Integrated Navigation and Principles of IMU Navigation(Chinese Edition Only)](http://www.i2nav.com/index/newListDetail_zw.do?newskind_id=13a8654e060c40c69e5f3d4c13069078&newsinfo_id=40f3c65b158742c099ba3b600c983aa1).

### 2.2 Data format

**Input Data**

- The IMU text file format is defined as:

| Columns | Data description         | Units |
| ------- | ------------------------ | ----- |
| 1       | GNSS seconds of week     | $s$   |
| 2~4     | X-Y-Z axes incremental angles    | $rad$ |
| 5~7     | X-Y-Z axes incremental velocity | $m/s$ |

- The GNSS-RTK position text file format is defined as:

| Columns | Data description               | Units |
| ------- | ------------------------------ | ----- |
| 1       | GNSS seconds of  week          | $s$   |
| 2       | latitude                       | $deg$ |
| 3       | longitude                      | $deg$ |
| 4       | ellipsoid altitude             | $m$   |
| 5~7     | position STD (north-east-down) | $m$   |

**Output Data**

- The text file format of the navigation result and the ground-truth is defined as:

| Columns | Data description                | Units |
| ------- | ------------------------------- | ----- |
| 1       | GNSS week                       | -     |
| 2       | GNSS seconds of week            | $s$   |
| 3       | latitude                        | $deg$ |
| 4       | longitude                       | $deg$ |
| 5       | ellipsoid altitude              | $m$   |
| 6~8     | 3-D velocity (north-east-down)  | $m/s$ |
| 9~11    | attitude angles (roll-pitch-yaw) | $deg$ |

- The IMU error text file format(all double data) is defined as:

| Columns | Data description                        | Units   |
| ------- |-----------------------------------------| ------- |
| 1       | GNSS seconds of week                    | $s$     |
| 2~4     | X-Y-Z axes gyroscope biases             | $deg/h$ |
| 5~7     | X-Y-Z axes accelerometer biases         | $mGal$  |
| 8~10    | X-Y-Z axes gyroscope scale factors      | $ppm$   |
| 11~13   | X-Y-Z axes accelerometer scale factors  | $ppm$   |f

- The state STD text file format(all double data) is defined as:

| Columns | Data description                          | Units  |
| ------- |-------------------------------------------|--------|
| 1       | GNSS seconds of week                      | $s$    |
| 2~4     | 3-D position STD (north-east-down)        | $m$    |
| 5~7     | 3-D velocity STD (north-east-down)        | $m/s$  |
| 8~10    | 3-D attitude STD (roll-pitch-yaw)         | $deg$  |
| 11~13   | X-Y-Z axes gyroscope bias STD             | $deg/h$ |
| 14~16   | X-Y-Z axes accelerometer bias STD         | $mGal$ |
| 17~19   | X-Y-Z axes gyroscope scale factor STD     | $ppm$  |
| 20~22   | X-Y-Z axes accelerometer scale factor STD | $ppm$  |

### 2.3 Initial align

KF-GINS only supports initial alignment given all initial states currently. The initial states need to be set in the configuration file (kf-gins-real.yaml) before executing the program.

## 3 Activate KGP

Please set configuration KGP to **True**, if KGP prediction is needed.

## 4 Datasets

We use demo dataset with the configuration file, which is located in the **dataset** directory. Data directory can be set in configuration file.

## 5 Result

The Final Result is located in "NavResult" folder.

## 6 Acknowledgement

The authors would like to acknowledge the team of Prof. Xiaoji Niu of the [Integrated and Intelligent Navigation (i2Nav) group](https://github.com/i2Nav-WHU/KF-GINS/) from GNSS Research Center of Wuhan University for providing the open-source KF-GINS software that was used in the paper.
