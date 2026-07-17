# dog3 路径跟踪控制器部署说明

`dog_move_forward.py` 是与规划器解耦的 ROS 2 Python 速度控制节点，源码位于本仓库，
也随 `dog_ego_planner` 包安装。dog3 部署时仍把它单独放在6号目录，便于独立上锁、
启停和审计。它订阅里程计和局部路径，并向自研 `/vel_cmd_copy` 发布
`geometry_msgs/msg/Twist`。

## 安全边界

- 默认 `enabled=false`、`dry_run=true`，误启动时不发布速度。
- 空路径、输入超时、时间戳过期、坐标系不一致、NaN、目标位于后方、转向需求超限时停车。
- 检测到其他 `/vel_cmd_copy` 发布者时拒绝争抢自研命令输出。
- `/vel_cmd_copy`不会直接驱动厂家底盘；接入`/vel_cmd`必须经过单独审核的仲裁/桥接层。
- 进程退出时会尽力发送多次零速，但 `Twist` 没有有效期；底盘必须另有命令超时和物理急停。
- 自动部署和软件验收不得解除双重上锁，不得连接生产速度话题进行运动测试。

## dog3 环境

部署目录：

```text
/userdata/6_moveforward
```

默认接口为：

```text
odom_topic=/lidar_location_now
path_topic=/dog_output_local_path_copy
expected_frame=local_map_lidar_init_xyz
velocity_topic=/vel_cmd_copy
```

`/lidar_location_now` 与 `/dog_output_local_path_copy` 必须使用同一个
`local_map_lidar_init_xyz` 坐标系；任一输入超时、frame 不一致或消息时间戳异常都会触发停车。

运行 Python ROS 2 程序前必须加载：

```bash
source /userdata/1_slam/setup_dog3_env.sh
```

只做安全启动检查时保持默认参数：

```bash
python3 /userdata/6_moveforward/dog_move_forward.py
```

此时节点不会发布非零速度。真正解除运动保护必须由现场人员在物理急停可用、机器狗架空、
确认没有其他速度发布者且完成独立测试后进行。不要把解除保护的命令写入自启动服务。

## 隔离测试

以下测试只能使用不连接生产 ROS 图的 domain，并使用专用测试速度话题：

```bash
export ROS_DOMAIN_ID=231
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS2CLI_NO_DAEMON=1
export MOVEFORWARD_RUN_ROS_TEST=1
python3 /userdata/6_moveforward/test_dog_move_forward.py -v
```

测试中的非零 `Twist` 只会发布到随机生成的 `/moveforward_test_*/velocity`，不会发布到
生产 `/vel_cmd`；所有测试仅使用随机隔离话题。
