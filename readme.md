
<img width="1195" height="906" alt="image" src="https://github.com/user-attachments/assets/0d676832-a103-4fd0-a604-69368c5987fd" />

控制方式有2种，一种使用本地的planner进行步态控制，一种使用models下的OpenVLA模型进行步态控制  
交互方式：  
    1. 通过自带的tools发生运行指令  
    2. 使用游戏手柄控制（开发中）  


# build env 
Mac M4

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