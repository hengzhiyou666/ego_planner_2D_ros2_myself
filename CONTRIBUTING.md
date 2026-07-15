# 参与贡献

感谢参与 `dog_ego_planner` 的开发。本工程用于机器狗局部路径规划，修改错误可能影响真机安全，因此应优先保证输入校验、碰撞检查和失败停车行为。

## 开发环境

- Ubuntu 22.04
- ROS 2 Humble
- C++17

构建与测试：

```bash
source /opt/ros/humble/setup.bash
./scripts/build.sh --debug
./scripts/test_isolated.sh
```

## 修改要求

- 不提交 `build/`、`install/`、`log/`、rosbag、IDE 配置或本机绝对路径。
- 保持包名 `dog_ego_planner` 和可执行程序 `dog_planner_node` 的兼容性。
- 新增话题和参数应可配置，默认使用相对话题名称。
- 对消息时间戳、坐标系、参数、数组尺寸和优化结果进行校验。
- 遇到数据过期、TF 失败、路径碰撞或规划失败时，必须保持安全停车输出。
- 修改算法时补充单元测试；修改 ROS 接口或 launch 时补充集成测试。
- 公共接口或安全行为发生变化时，同步更新 README 和 CHANGELOG。

## 真机安全

自动化测试通过不代表可以直接让机器狗自由运行。建议依次完成：

1. 单元测试和集成测试。
2. 仿真或 rosbag 回放。
3. 机器狗架空后的低速测试。
4. 带物理急停的封闭场地测试。

在 dog3 上测试时，必须遵守 [部署说明](docs/DEPLOYMENT_DOG3.md)，使用独立 ROS domain，不得发布速度指令，也不得启停运动控制服务。

## 许可证

本工程采用 `GPL-3.0-only`。请保留 SPDX 标识和上游来源说明；新增第三方代码必须提供明确来源及兼容许可证。
