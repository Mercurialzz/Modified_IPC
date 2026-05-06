# My_IPC — 多线程集成规划控制系统

基于 [HKU MARS Lab IPC](https://github.com/hku-mars/IPC) 的改进版本，在原始框架上增加了多线程异步规划、ROG-Map 建图、CIRI 安全飞行走廊及多项重规划平滑优化。

## 1 系统架构

系统以单 ROS 节点 (`ipc_node`) 运行，频率 100 Hz，包含以下核心模块：

```
激光雷达点云 → ROG-Map (3D 栅格 + ESDF)
                 ↓
          A* 路径搜索 (ESDF 代价偏置 + Floyd 平滑)
                 ↓
          CIRI 安全飞行走廊 (凸多面体约束)
                 ↓
          MPC QP 求解 (OSQP, Jerk 最优轨迹)
                 ↓
          微分平坦转换 (体轴角速率 + 推力)
                 ↓
          PX4 / SO3 仿真控制器
```

### 1.1 子系统

| 模块 | 路径 | 功能 |
|------|------|------|
| 状态机 & 主控 | `src/IPC/IPC/src/planner.cpp` | 有限状态机（等待→手动→起飞→自主飞行→降落），100 Hz 主循环 |
| 异步规划线程 | `src/IPC/IPC/src/planner.cpp` | 后台路径搜索 + 走廊生成，不阻塞控制回路 |
| A* 路径搜索 | `src/IPC/IPC/src/path_searth/rog_astar.cpp` | 基于 ROG-Map 的 A* 搜索，ESDF 距离代价偏置，Floyd 捷径简化 |
| 安全飞行走廊 | `src/IPC/IPC/src/sfc_core/` | CIRI 算法生成凸多面体 SFC 位置硬约束 |
| MPC 控制器 | `src/IPC/IPC/src/mpc_control/mpc.cpp` | 三阶积分器 + 阻力模型，OSQP 求解 Jerk 最优轨迹 |
| ROG-Map 建图 | `src/rog_map/` | 3D 概率占用栅格地图，射线投射、膨胀、ESDF |
| MARSIM 仿真器 | `src/mars_uav_sim/` | SO3 四旋翼动力学、几何控制器、激光雷达模拟 |

### 1.2 有限状态机

```
WAIT_STATUS → MANUAL_CTRL → AUTO_HOVER ⇄ AUTO_TAKEOFF
                                  ↓
                             CMD_CTRL (自主飞行)
                                  ↓
                             AUTO_LAND
```

## 2 本版本改进

- **多线程异步规划**：A* 搜索和走廊生成在独立线程运行，不阻塞控制回路，即使规划耗时较长也能保证 100 Hz 控制输出
- **ROG-Map 集成**：3D 滚动概率占用栅格地图，支持射线投射、ESDF 距离场，可滑动窗口实现大范围规划
- **Floyd 捷径安全校验**：ESDF 距离查表确保 Floyd 直线简化不会擦过障碍物边缘
- **重规划路径拼接**：新旧路径在连接点拼接并局部平滑，避免路径切换时的几何跳变
- **C2 连续性融合**：路径切换时，用当前状态 (p/v/a) 外推与新路径前几步位置加权融合，消除参考轨迹瞬态跳变
- **帧间速度低通滤波**：速度参考在帧间平滑过渡，抑制重规划速度跳变
- **偏航切线跟踪**：沿前方轨迹切线方向计算 yaw 目标，带变化率限幅
- **MPC 热启动**：复用上一周期的参考轨迹和 OSQP 原变量，加速 QP 收敛
- **多航点循环**：支持预设 4 个航点依次循环飞行
- **安全应急机制**：MPC/走廊生成失败时回退悬停，路径阻塞时切回上一次安全路径

## 3 目录结构

```
My_IPC/
├── src/
│   ├── IPC/IPC/                     # IPC 主程序
│   │   ├── src/
│   │   │   ├── main.cpp             # ROS 节点入口
│   │   │   ├── planner.cpp/h        # 主控状态机 & 重规划逻辑
│   │   │   ├── param.cpp/h          # ROS 参数加载
│   │   │   ├── path_searth/          # A* 路径搜索
│   │   │   ├── mpc_control/         # OSQP MPC 控制器
│   │   │   ├── sfc_core/            # CIRI 安全飞行走廊
│   │   │   ├── utils/               # 几何、优化工具库
│   │   │   └── trans_odom.cpp       # 坐标系转换
│   │   ├── config/                  # YAML 参数文件 + Rviz
│   │   ├── launch/                  # ROS launch 文件
│   │   ├── include/                 # 头文件
│   │   └── test/                    # 突然避障测试程序
│   ├── rog_map/                     # ROG-Map 栅格地图库
│   └── mars_uav_sim/               # MARSIM 四旋翼仿真器
├── build/                           # 编译输出 (不纳入版本控制)
├── devel/                           # ROS 环境 (不纳入版本控制)
├── .gitignore
└── README.md
```

## 4 依赖环境

### 4.1 系统

- Ubuntu 18.04 / 20.04
- ROS Noetic
- C++17 编译器
- Eigen ≥ 3.3.4
- PCL ≥ 1.6

### 4.2 OSQP & OSQP-Eigen

OSQP 版本需 **< 0.6.3**：

```bash
git clone --recursive https://github.com/osqp/osqp
cd osqp && mkdir build && cd build
cmake .. && sudo make install

git clone https://github.com/robotology/osqp-eigen.git
cd osqp-eigen && mkdir build && cd build
cmake .. && sudo make && sudo make install
```

### 4.3 其他依赖

```bash
# debug 工具 backward-cpp
sudo apt-get install libdw-dev
wget https://raw.githubusercontent.com/bombela/backward-cpp/master/backward.hpp
sudo mv backward.hpp /usr/include

# ROS 格式化输出
sudo apt-get install ros-noetic-rosfmt
```

## 5 编译

```bash
mkdir -p My_IPC_ws/src
cd My_IPC_ws/src
git clone git@github.com:Mercurialzz/My_IPC.git
cd ..
catkin_make -j$(nproc)
```

## 6 运行

### 6.1 仿真环境

```bash
source devel/setup.bash
roslaunch ipc ipc_sim.launch
```

启动后自动加载 MARSIM 仿真器（森林地图、SO3 四旋翼动力学、虚拟激光雷达）和 IPC 节点。在 Rviz 中使用 `3D Goal` 工具点击设置目标点。

### 6.2 真机飞行

共需 6 个终端，按顺序启动：

```bash
# 终端1: 启动 Livox MID360 激光雷达驱动
./start_mid360.sh

# 终端2: 启动 MAVROS (PX4 飞控通信)
./start_mavros.sh

# 终端3: 启动 Point-LIO (激光惯性里程计)
./start_pointlio.sh

# 终端4: 启动坐标转换节点 (trans_odom_node)
./start_trans.sh

# 终端5: 启动 IPC 主程序
source devel/setup.bash
roslaunch ipc ipc.launch

# 终端6: 发送起飞指令
./start_takeoff.sh
```

> **注意**：定位链路为 MID360 → Point-LIO → trans_odom → IPC，必须按顺序依次启动。起飞前确保定位已收敛，MAVROS 与飞控通信正常。

### 6.3 突然避障测试

```bash
source devel/setup.bash
roslaunch ipc ipc_avoid.launch
roslaunch ipc fast_avoid.launch
```

## 7 关键参数

### 7.1 规划参数

| 参数 | 真机 | 仿真 | 说明 |
|------|------|------|------|
| `planning_horizon` | 15.0 | 15.0 | 规划视野范围 (m) |
| `path_dis` | 0.02 | 0.02 | 路径插值间距 (m) |
| `ref_dis` | 15 | 15 | MPC 参考点间距 (索引步数) |
| `robot_r` | 0.3 | 0.3 | 碰撞半径 (m) |
| `iris_iter_num` | 2 | 2 | 走廊迭代次数 |

### 7.2 A* & Floyd

| 参数 | 真机 | 仿真 | 说明 |
|------|------|------|------|
| `esdf_weight` | 3.0 | 3.0 | ESDF 代价权重 |
| `safe_distance` | 1.0 | 1.0 | ESDF 安全阈值 (m) |
| `floyd_safe_distance` | 0.8 | 0.8 | Floyd 直连安全距离 (m) |
| `heu_type` | 2 | 2 | 启发式：0=对角 1=曼哈顿 2=欧式 |

### 7.3 MPC

| 参数 | 真机 | 仿真 | 说明 |
|------|------|------|------|
| `mpc/horizon` | 15 | 15 | 预测步数（总时间 = horizon × step） |
| `mpc/step` | 0.1 | 0.1 | 时间步长 (s) |
| `mpc/R_p` | 2000 | 2000 | 位置跟踪权重 |
| `mpc/R_u_con` | 1.0 | 1.0 | 控制平滑度权重 |
| `mpc/D_z` | 0.05 | 0.05 | Z 轴阻力系数 |

### 7.4 起飞降落

| 参数 | 真机 | 仿真 | 说明 |
|------|------|------|------|
| `takeoff_height` | 1.5 | 1.0 | 起飞高度 (m) |
| `takeoff_land_speed` | 0.3 | 0.3 | 起飞/降落速度 (m/s) |
| `enable_auto_arm` | true | — | 自动解锁（真机需关闭 QGC 走自动流程） |

## 8 论文引用

```
@article{liu2023integrated,
  title={Integrated Planning and Control for Quadrotor Navigation in Presence
         of Suddenly Appearing Objects and Disturbances},
  author={Liu, Wenyi and Ren, Yunfan and Zhang, Fu},
  journal={IEEE Robotics and Automation Letters},
  year={2023},
  publisher={IEEE}
}
```

视频：[YouTube](https://www.youtube.com/watch?v=EZFxTkqqat4) · [Bilibili](https://www.bilibili.com/video/BV1NM4y117TH)

代码：[hku-mars/IPC](https://github.com/hku-mars/IPC)
