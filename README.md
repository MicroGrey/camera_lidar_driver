# Camera Lidar Driver

## 系统要求
- Ubuntu 22.04
- ROS 2 Humble
- Livox SDK 2.3.0+

## 依赖安装

### 1. 安装 Livox SDK
```bash
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd Livox-SDK2
mkdir build
cd build
cmake .. && make -j
sudo make install
```
### 2. 安装 Livox ROS2 Driver
```bash
mkdir -p ~/ws_livox/src
cd ~/ws_livox/src
git clone https://github.com/Livox-SDK/livox_ros2_driver.git
cd ..
colcon build
source install/setup.bash
```

## 注意事项
### 启动指令
```bash
colcon build
source install/setup.bash
ros2 launch sensor sensor.launch.py
```
### qos
在rviz里配置img的
> Depth=1  
> History Policy=Keep Last  
> Reliability Policy=Best Effort  

![example](image.png)
### 在项目根目录新建camera文件夹放你的mfs文件