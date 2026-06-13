# Fastlio2算法部署教程

## 1. Ubuntu 和 ros
    - Ubuntu 22.04
    - ros2 humble

## 2. 安装 Livox-SDK2
```
    cd ~
    git clone https://github.com/Livox-SDK/Livox-SDK2.git
    cd Livox-SDK2

    mkdir build && cd build
    cmake ..
    make -j4
    sudo make install
```

## 3. 编译 Livox Ros Driver 2
```
    mkdir -p ~/fastlio2_ws/src
    cd ~/fastlio2_ws/src

    git clone https://github.com/Livox-SDK/livox_ros_driver2.git
    cd livox_ros_driver2
    ./build.sh humble
```

## 4. 下载并编译Fast_Lio_Ros2
```
    cd ~/fastlio2_ws/src
    git clone https://github.com/Ericsii/FAST_LIO_ROS2.git --recursive

    cd ~/fastlio2_ws
    rosdep install --from-paths src --ignore-src -r -y

    rm -rf ~/fastlio2_ws/build/livox_ros_driver2/ament_cmake_python/livox_ros_driver2/livox_ros_driver2
    colcon build --symlink-install
```

## 5. 配置文件修改
    config文件夹里的yaml配置文件根据所用激光雷达型号进行选择。
    但每个yaml文件publish一类里需要加入' map_en: true '配置，否则无法得到累积点云地图。

## 6. 运行示例
```
    # 第一个终端
    cd ~/fastlio2_ws
    . install/setup.bash
    ros2 launch fast_lio mapping.launch.py config_file:=avia.yaml
```
```
    # 新开一个终端
    ros2 bag play your_bag.bag --clock
```

## 7. PCD保存
```
    # 新开一个终端
    rqt
```
    依次选择 Plugins->Services->Service Caller， 最后call /map_save服务进行保存