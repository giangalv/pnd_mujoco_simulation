# 介绍

## PND mujoco

`pnd_mujoco` 是基于 `pnd sdk` 和 `mujoco` 开发的仿真器。用户使用 `pnd_ros2` 和 `pnd_sdk_python` 开发的控制程序可以方便地接入该仿真器，实现仿真到实物的开发流程。仓库别基于 python cyclonedds 以及 python ros2 humble实现了两个版本的仿真器

## 目录结构

- `simulate_python`: 基于 pnd_sdk_py 和 mujoco (python) 实现的仿真器
- `pnd_robots`: pnd sdk 支持的机器人 mjcf 描述文件
- `terrain_tool`: 仿真场景地形生成工具
- `example`: 例程

## 支持的 PND sdk2 消息：

**当前版本仅支持底层开发，主要用于控制器的 sim to real 验证**

- `LowCmd`: 电机控制指令
- `LowState`：电机状态

## 消息(DDS idl)类型说明

- PND Adam_u 型号的机器人使用 adam_u idl 实现底层通信

## 相关链接

- [pnd_sdk_python](https://github.com/pndbotics/pnd_sdk_python)
  <!-- - [pnd_ros2](https://github.com/pndrobotics/pnd_ros2) -->
  <!-- - [PND 文档](https://support.pnd.com/home/zh/developer) -->
- [mujoco doc](https://mujoco.readthedocs.io/en/stable/overview.html)

# 安装

## c++ 仿真器 (simulate)

### 1. 依赖

```bash
sudo apt install libyaml-cpp-dev libspdlog-dev libboost-all-dev libglfw3-dev
```

## Python 仿真器 (simulate_python)

### 1. 依赖

#### pnd_sdk_python

```bash
cd ~
sudo apt install python3-pip
git clone https://github.com/pndrobotics/pnd_sdk_python.git
cd pnd_sdk_python
pip3 install -e .
```

#### mujoco-python

```bash
pip3 install mujoco
```

#### joystick

```bash
pip3 install pygame
```

### 2. 测试

```bash
cd ./simulate_python
python3 ./pnd_mujoco.py
```

在新终端运行

```bash
python3 ./test/test_pndbotics_sdk.py
```

adam_u机器人会打开手臂然后放下。

# 使用

### python 仿真器

python 仿真器的配置文件位于 `/simulate_python/config.py` 中：

```python

ROBOT = "adam_u"

# 机器人仿真仿真场景文件
ROBOT_SCENE = "../pnd_robots/" + ROBOT + "/scene.xml" # Robot scene


# dds domain id，最好与实物(实物上默认为 0)区分开
单独打开ROS2或者DDS以及其对应的ID
# For ROS2
SDK_TYPE="ROS2" # "ROS2" or "DDS"
DOMAIN_ID = 2 # Domain id

# For DDS
SDK_TYPE="DDS" # "ROS2" or "DDS"
DOMAIN_ID = 1 # Domain id

# 网卡名称, 对于仿真建议使用本地回环 "lo"
INTERFACE = "lo" # Interface

# 是否输出机器人连杆、关节、传感器等信息，True 为输出
PRINT_SCENE_INFORMATION = True

USE_JOYSTICK = 1 # Simulate PND WirelessController using a gamepad
JOYSTICK_TYPE = "xbox" # support "xbox" and "switch" gamepad layout
JOYSTICK_DEVICE = 0 # Joystick number

# 是否使用虚拟挂带, 1 为启用
# 主要用于模拟 H1 机器人初始化挂起的过程
ENABLE_ELASTIC_BAND = False

# 仿真步长 单位(s)
# 为保证仿真的可靠性，需要大于 viewer.sync() 渲染一次所需要的时间
SIMULATE_DT = 0.003

# 可视化界面的运行步长，0.02 对应 50fps/s
VIEWER_DT = 0.02
```

### 游戏手柄

仿真器会使用 Xbox 或者 Switch 游戏来模拟机器人的无线控制器，并将手柄按键和摇杆信息发布在"rt/wireless_controller" topic。如果手上没有可以使用的游戏手柄，需要将 `config.yaml/config.py` 中的 `use_joystick/USE_JOYSTICK` 设置为 0。如果使用的手柄不属于 Xbox 和 Switch 映射，可以在源码中自行修改或添加(可以使用 `jstest` 工具查看按键和摇杆 id)：

In `simulate/src/pnd_sdk2_bridge/pnd_sdk2_bridge.cc`:

```C++
 if (js_type == "xbox")
{
    js_id_.axis["LX"] = 0; // Left stick axis x
    js_id_.axis["LY"] = 1; // Left stick axis y
    js_id_.axis["RX"] = 3; // Right stick axis x
    js_id_.axis["RY"] = 4; // Right stick axis y
    js_id_.axis["LT"] = 2; // Left trigger
    js_id_.axis["RT"] = 5; // Right trigger
    js_id_.axis["DX"] = 6; // Directional pad x
    js_id_.axis["DY"] = 7; // Directional pad y

    js_id_.button["X"] = 2;
    js_id_.button["Y"] = 3;
    js_id_.button["B"] = 1;
    js_id_.button["A"] = 0;
    js_id_.button["LB"] = 4;
    js_id_.button["RB"] = 5;
    js_id_.button["SELECT"] = 6;
    js_id_.button["START"] = 7;
}
```

In `simulate_python/pnd_sdk2_bridge.py`:

```python
if js_type == "xbox":
    self.axis_id = {
        "LX": 0,  # Left stick axis x
        "LY": 1,  # Left stick axis y
        "RX": 3,  # Right stick axis x
        "RY": 4,  # Right stick axis y
        "LT": 2,  # Left trigger
        "RT": 5,  # Right trigger
        "DX": 6,  # Directional pad x
        "DY": 7,  # Directional pad y
    }

    self.button_id = {
        "X": 2,
        "Y": 3,
        "B": 1,
        "A": 0,
        "LB": 4,
        "RB": 5,
        "SELECT": 6,
        "START": 7,
    }
```

### 人形机器人虚拟挂带

考虑到人形机器人不便于从平地上启动并进行调试，在仿真中设计了一个虚拟挂带，用于模拟人形机器人的吊起和放下。设置 `enable_elastic_band/ENABLE_ELASTIC_BAND = 1` 可以启用虚拟挂带。加载机器人后，按 `9` 启用或松开挂带，按 `7` 放下机器人，按 `8` 吊起机器人。

## 2. 地形生成工具

我们提供了一个在 mujoco 仿真器中参数化创建简单地形的工具，支持添加楼梯、杂乱地面、高程图等地形。程序位于 `terrain_tool` 文件夹中。具体的使用方法见 `terrain_tool` 文件夹下的 readme 文件。
![](./doc/terrain.png)

## 3. sim to real

`example` 文件夹下提供了使用不同接口实现 Go2 机器人站起再趴下的简单例子。这些例子简演示了如何使用 PND 提供的接口实现仿真到实物的实现。下面是每个文件夹名称的解释：

- `cpp`: 基于 `C++`, 使用 `pnd_sdk2` 接口
- `python`: 基于 `python`，使用 `pnd_sdk_python` 接口
- `ros2`: 基于`ros2`，使用 `pnd_ros2` 接口

### pnd_sdk2

1. 编译运行

```bash
cd example/cpp
mkdir build && cd build
cmake ..
make -j4
```

运行：

```bash
./stand_go2 # 控制仿真中的机器人 (需确保 Go2 仿真场景已经加载)
./stand_go2 enp3s0 # 控制机器人实物，其中 enp3s0 为机器人所连接的网卡名称
```

2. sim to real

```C++
if (argc < 2)
{
    // 如果没有输入网卡，使用仿真的 domian id 和 网卡(本地)
    ChannelFactory::Instance()->Init(1, "lo");
}
else
{
    // 否则使用指定的网卡
    ChannelFactory::Instance()->Init(0, argv[1]);
}
```

### pnd_sdk_python

1. 运行：

```bash
python3 ./stand_go2.py # 控制仿真中的机器人 (需确保 Go2 仿真场景已经加载)
python3 ./stand_go2.py enp3s0 # 控制机器人实物，其中 enp3s0 为机器人所连接的网卡名称
```

2. sim to real

```python
if len(sys.argv) <2:
    // 如果没有输入网卡，使用仿真的 domian id 和 网卡(本地)
    ChannelFactoryInitialize(1, "lo")
else:
    // 否则使用指定的网卡
    ChannelFactoryInitialize(0, sys.argv[1])
```

### pnd_ros2

1. 编译安装
   首先确保已经正确配置好 pnd_ros2 环境，见 [pnd_ros2](https://github.com/pndrobotics/pnd_ros2)。

```bash
source ~/pnd_ros2/setup.sh
cd example/ros2
colcon build
```

2. 运行仿真

```bash
source ~/pnd_ros2/setup_local.sh # 使用本地网卡
export ROS_DOMAIN_ID=1 # 修改domain id 与仿真一致
./install/stand_go2/bin/stand_go2 # 运行
```

3. 运行实物

```bash
source ~/pnd_ros2/setup.sh # 使用机器人连接的网卡
export ROS_DOMAIN_ID=0 # 使用默认的 domain id
./install/stand_go2/bin/stand_go2 # 运行
```

