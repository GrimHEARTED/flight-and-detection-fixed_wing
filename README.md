# 使用说明
- 环境配置：可参考[PX4-ROS2-Gazebo-YOLOv8](https://github.com/monemati/PX4-ROS2-Gazebo-YOLOv8)。
- 添加世界：将 `cadc_custom.sdf` 添加至 /PX4-Autopilot/Tools/simulation/gz/worlds，并修改对应CMake文件。总体方法见[官网教程](https://docs.px4.io/v1.15/en/sim_gazebo_gz/#adding-new-worlds-and-models)。
- 添加模型：总体方法见[官网教程](https://docs.px4.io/v1.15/en/sim_gazebo_gz/#adding-new-worlds-and-models)。
    1. 将 `rc_cessna_mono_cam` 文件夹移至/PX4-Autopilot/Tools/simulation/gz/models；
    2. 在/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes新建 `4012_gz_rc_cessna_mono_cam` ，内容为：
        ```shell
        #!/bin/sh
        #
        # @name Gazebo cessna mono cam
        #
        # @type Fixedwing
        #

        PX4_SIM_MODEL=${PX4_SIM_MODEL:=rc_cessna_mono_cam}

        . ${R}etc/init.d-posix/airframes/4003_gz_rc_cessna
        ```
    3. 修改当前文件夹下的CMake文件；
    4. 在 `4003_gz_rc_cessna` 中添加：
        ```shell
        param set-default COM_ARM_CHK_ESCS 0
        param set-default EKF2_MAG_TYPE 1
        ```
- ToDo：使用 `flight_loiter.py` 控制飞行，并保存得到的 `num_roi_` 图像作为数据，训练YOLO11n模型，将权重放在/ws/src/py_nums_coords/resource下。
-----
# 环境说明
- Ubuntu 22.04
- PX4 v1.15
- ROS2 Humble
- Gazebo Harmonic 8.10.0
- OpenCV-CUDA 4.10.0
- QGroundControl 5.0.5
-----
# 展示
[![仿真演示](https://i2.hdslb.com/bfs/archive/1c5307d020e6c1527b8303332321b219c04db52a.jpg@308w_174h)](https://www.bilibili.com/video/BV1Ascrz3ECj/)
> 该视频使用Gazebo11仿真，flight.py控制飞行，和项目提供的使用方法有所出入，仅供参考。
