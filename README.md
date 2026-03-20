# ego_planner_2D_ros2_fordog
这是一个 ROS2 本地重规划节点，主程序为 `dog_planner_node`（包名 `dog_ego_planner`）。  
先在工作空间编译并加载环境：`colcon build --packages-select dog_ego_planner && source install/setup.bash`。  
启动方式：`ros2 launch dog_ego_planner robot_launch.py`（仅启动规划节点，不启动 RViz）。  
本机调试可视化可手动：`rviz2 -d $(ros2 pkg prefix dog_ego_planner)/share/dog_ego_planner/rviz/dog_debug.rviz`。  
真机/板端请保持 `use_sim_time=false`（见 `planner_params.yaml`）；误开且无 `/clock` 会导致 TF 查询卡死。  
参数文件：`config/planner_params.yaml`（可通过 `params_file:=<your.yaml>` 覆盖）。  
节点订阅 `/odometry`（当前位置/速度）、`/pct_path`（全局参考路径）、`/lidar_points`（障碍点云）。  
收到点云后先做高度滤波，生成 2D 障碍并更新栅格地图。  
再从当前位姿沿全局路径截取局部规划段（默认约 7m 视野）并采样控制点。  
规划器使用 B-spline 进行轨迹优化，按定时器周期触发重规划。  
节点发布 `/dog_output_local_path`（局部轨迹）、`/dog_output_global_path_unfinished`（未完成全局段）、`/lidar_points_filtered`（滤波点云）。  
目录规范：主工程代码在 `application/`，第三方库在 `lib/lib_*`。  