# VOXEL_LIO_SAM
![image](https://github.com/liangheming/VoxelMap_LIO_SAM/blob/voxel_map/imgs/sample0.jpg)
## 主要工作
1. 重构[FASTLIO2](https://github.com/hku-mars/FAST_LIO)代码，使用GN代替原有IESKF(理论上等价，但是GN更加易懂)
2. 重构[VoxelMap](https://github.com/hku-mars/VoxelMap)，并以此作为FASTLIO的地图管理模块
3. 目前暂时支持MID_360的传感器
4. 代码上LIO与ROS部分隔离，可以自行DIY至其他机器人框架
5. 优化VOXEL_MAP的内存管理，增加删除Voxel(last recent used)的功能，以便于实际部署

## 环境说明
```text
系统版本: ubuntu20.04
机器人操作系统: ros1-noetic
```

## 编译依赖
1. livox_ros_driver2
2. pcl
3. sophus
4. eigen

### 1.安装 LIVOX-SDK2
```shell
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd ./Livox-SDK2/
mkdir build
cd build
cmake .. && make -j
sudo make install
```

### 2.安装 livox_ros_driver2
```shell
mkdir -r ws_livox/src
git clone https://github.com/Livox-SDK/livox_ros_driver2.git ws_livox/src/livox_ros_driver2
cd ws_livox/src/livox_ros_driver2
./build.sh ROS1
```

### 3. 安装Sophus
```
git clone https://github.com/strasdat/Sophus.git
cd Sophus
mkdir build && cd build
cmake .. -DSOPHUS_USE_BASIC_LOGGING=ON
make
sudo make install
```
**新的Sophus依赖fmt，可以在CMakeLists.txt中添加add_compile_definitions(SOPHUS_USE_BASIC_LOGGING)去除，否则会报错**

## DEMO 数据
```text
链接: https://pan.baidu.com/s/1ZPUwWyHmvpGHuL9TiFrsmA?pwd=k9vx 提取码: k9vx 
--来自百度网盘超级会员v7的分享
```

## 启动脚本
1. 建图线程
```shell
roslaunch roslaunch voxel_lio_sam mapping_launch.launch 
rosbag play your_bag.bag
```
2. 保存地图
```
rosservice call /mapping_node/save_map "path: '/path_to_save/temp_test.pcd'
resolution: 0.1"
```
*** 切记在roslaunch的时候将config中 save_map 设置为Ture ***
## TODO
- 基于(STD)特征以及关键帧，实现回环模块以及相应的PGO；
- 添加VOXEL_MAP++的模块，实现voxel合并逻辑；

## 特别感谢
1. [FASTLIO2](https://github.com/hku-mars/FAST_LIO)
2. [VoxelMap](https://github.com/hku-mars/VoxelMap)