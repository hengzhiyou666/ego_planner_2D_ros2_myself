# dog3 安全部署与验收

`dog_ego_planner` 只发布几何路径，不发布速度指令。即便如此，在机器狗已有控制系统运行时，部署验收也必须与生产 ROS 图隔离。

## 部署目录

目标目录固定为：

```text
/userdata/6_egoplanner
```

只同步源码、配置、测试、文档与许可证；不复制开发机的 `build/`、`install/`、`log/`、`.git/` 或 IDE 缓存。构建产物必须在 dog3 上原生生成，以匹配 aarch64 和机器狗自带 ROS 2 Humble。

## dog3 构建环境

dog3 的 ROS 2 安装位于 `/app/opt/ros/humble`，构建工具和 Eigen 由 `/userdata/2_slam/.deps` 提供。不要只加载厂家 ROS 的 `setup.bash` 后直接编译；那会选中 Fast DDS，并在部分同款板端触发非标准 OpenSSL 路径问题。统一使用 SLAM 工作区提供的 dog3 环境脚本：

```bash
cd /userdata/6_egoplanner
./build_on_dog.sh
```

该脚本会加载 `/userdata/2_slam/setup_dog3_env.sh`，使用 dog3 实际部署的
`rmw_zenoh_cpp`、板端 colcon/Eigen 依赖，清理旧路径构建缓存并在当前目录原生编译。

板端没有 lint 可执行程序，因此板端验收可设置 `DOG_EGO_PLANNER_ENABLE_LINT=OFF`；完整 lint 仍由开发机和 CI 执行。

该设备的 ROS 安装由厂商 sysroot 生成，部分导入目标仍引用不存在的
`/sysroot/usr/lib/aarch64-linux-gnu/libpython3.10.so`。在 dog3 原生构建时传入：

```bash
-DDOG_EGO_PLANNER_VENDOR_SYSROOT_PREFIX=/sysroot
```

此选项只重定位依赖目标中遗留的 `/sysroot/usr/...` 使用要求，不创建或修改系统级
`/sysroot`，也不会影响默认的开发机和 CI 构建。

## 不移动机器狗的测试约束

板端测试固定使用：

```bash
export ROS_DOMAIN_ID=232
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS2CLI_NO_DAEMON=1
```

`scripts/test_isolated.sh` 会在加载工作区后强制写入这些值，不继承调用者可能存在的生产
`ROS_DOMAIN_ID` 或 RMW 设置。CTest 中的 ROS 测试也配置了相同环境，且 launch test 使用专用
namespace。测试期间：

- 不启动 `robot_launch.py` 或生产 launch。
- 不启动、停止或重启 locomotion、base action、导航等系统服务。
- 不向生产 domain 发布路径、速度、模式切换或动作指令。
- 只执行编译、GTest 和本包自带的隔离 launch/integration tests。
- 空点云和时间不同步点云必须触发停车，不得作为“无障碍”恢复条件。

构建完成后执行 `scripts/test_isolated.sh`。只有隔离测试通过后，才允许由现场人员在物理急停可用的条件下另行安排仿真、架空低速和封闭场地测试；该阶段不属于自动部署步骤。

构建完成后可使用以下脚本启动规划节点，无需每次手动加载 ROS 和工作空间环境：

```bash
cd /userdata/6_egoplanner
./1_start_planner.sh
```

该脚本只启动局部规划器，不启动速度控制程序。
