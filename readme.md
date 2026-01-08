
<img width="1195" height="906" alt="image" src="https://github.com/user-attachments/assets/0d676832-a103-4fd0-a604-69368c5987fd" />

这是一个机器人控制的练习项目，仿真环境使用Mujoco，机器人头顶的长条为ToF相机  
控制方式有2种，一种使用本地的planner进行步态控制，一种使用models下的OpenVLA模型进行步态控制  
交互方式：  
    1. 通过自带的tools发生运行指令  
    2. 使用游戏手柄控制（开发中）  


# build env 
Mac M4  
其他平台暂时还没有适配，后续会考虑适配Linux平台。  

# requirements
```
brew install opencv eigen3
pip install 
```

# build
cmake --build build

# run
`
./build/MujocoDDSExample
`

open another terminal to run tool

`
./build/tools/dds_control_client basic / raw
`

# ToDO  
 - 给Spot增加IMU传感器，让其参与姿态控制
 - 结合ToF传感器，使用OpenVLA来控制机器人姿态
 - 增加多个场景（楼梯、碎石地面、障碍物）
 - 让Planner支持多种型号的机器人
 - 增加强化学习功能
 - 增加一个启动界面，用户可以选择机器人、选择场景、选择不同的控制方式