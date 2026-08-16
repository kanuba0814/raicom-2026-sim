# raicom_round_table_navigation

2026 RAICOM 仿真的圆台/圆柱检测与**SIM-only**圆心对准、正向靠近包。不得连接或控制物理机器人。

## 数据与分层

`round_table_detector_node.py` 订阅 `/fuse_scan`，发布：

- `/round_table/circle`：`geometry_msgs/Vector3Stamped`，`x/y` 为 scan frame 圆心，`z` 为拟合半径；
- `/round_table/valid`：稳定有效标志；
- `/round_table/diagnostic`：点数、残差、半径和稳定帧数。

定位/SLAM 底图始终是 `/map`。队伍禁区只通过独立的 `/raicom/team_boundary_mask` 提供；它是完整 footprint 平移/旋转停车 sweep 的禁区 guard，不参与圆几何推断。

## 纯 Python 核心

- `circle_detector.py`：聚类、圆拟合、歧义拒绝和稳定帧；
- `docking_geometry.py`：`ALIGN_CENTER → APPROACH_CENTER → COMPLETE/STOP`；
- `boundary_guard.py`：完整矩形 footprint occupancy、平移停车和旋转 sweep；
- `docking_adapter.py`：admission、canonical grid hash、dropout gate 和 phase command guard。

```bash
PYTHONPATH=src python3 -m pytest -q test
```

所有尺寸、速度、净距和 1 cm 不确定度均为 `SIM_WORKING_ASSUMPTION`，不是赛规或实体机器人事实。

## 圆心对准→正向靠近合同

circle 在 `base_footprint` 下为 `(cx, cy, r)`，其中 `+x` 向前。状态机计算
`face_error = atan2(cy, cx)`：

1. `ALIGN_CENTER` 只发布 `wz`，最大 `0.08 rad/s`，按偏角减速并要求连续 3 帧对准；
2. `APPROACH_CENTER` 仅在偏角和小侧偏满足阈值时发布 `vx >= 0`，`wz=0`、`vy=0`，最大 `0.03 m/s`，按 gap 误差减速；偏角重新超阈值时先零输出并回到 `ALIGN_CENTER`，绝不边转边进；
3. 前端 gap 是 `cx - r - front_extent`，目标测量值为 `0.03 m`。其明确报告的 SIM 测量不确定度为 `0.01 m`，不得宣称精确 3 cm；仅 `0.02..0.04 m` 连续 3 帧才 `COMPLETE`；
4. `cx <= 0`、侧偏失效、无效观测、gap 小于 `0.01 m`、或越过完成区间均锁存 `STOP`，必须 disabled-only ack 后重新资格确认。

## ROS adapter

`round_table_docking_adapter_node.py` 是唯一运动适配层：

- 只发布 `/raicom/cmd_vel/relative`，绝不直接发布 `/cmd_vel`；
- `/cmd_vel` 唯一 publisher 必须是 `/raicom_cmd_vel_gate`；`move_base` 与 adapter 互斥，`/raicom/cmd_vel/nav` 必须无 publisher；
- 默认 `enabled=false`，且仅 `/raicom/backend=SIM` 可 admission；保留 map identity、publisher ownership、snapshot/freshness/dropout 与 soft-estop；
- 平移命令执行完整 team-boundary footprint translation stopping sweep，旋转命令执行完整 rotation stopping sweep；unknown、occupied、越界、身份/TF/发布者异常均 fail-closed。
- 红蓝边界不可关闭。若径向 3 cm 与完整 footprint 留在己方几何冲突，adapter 必须在最近安全姿态返回 `BOUNDARY_LIMIT`，不得跨界或冒充成功。

launch 不再要求切线资格、target side 或 boundary half-plane 参数。受管入口仍仅使用 `tools/raicomctl` 的窄命令。

## 当前验收边界

已在隔离 navigation 容器验证构建、disabled smoke、粗 staging、低速对心及多种 fail-closed；每次失败后 `/cmd_vel` 六维全零。尚未证明约 3 cm 终态，SIM 传感器噪声下的 gap 仍需校准。实体机器人未获授权。

## 中心物资岛（SIM-only，纯 Python 安全核心）

`center_island_mission.py` 新增的 `CenterIslandMission` 是**不含 ROS publisher**的状态机：
`NAVIGATE_STAGING → AR_ALIGN → BASE_HOLD → ARM_OBSERVE → SELECT_TARGET → PICK → STORE_REAR → ARM_SAFE_RETRACT → REFEREE_HOLD`，任何失败进入 `STOP`。`STOP` 的合同为六维底盘零速建议、吸盘关闭、`SAFE_RETRACT`；在关节反馈确认安全回收前不得清除 `arm_extended` 互锁，也不自动重试。放入后部格不等于机械臂已安全收回，只有独立回收反馈成功后才进入 `REFEREE_HOLD`，且绝不自行开始下一轮。

实际 ROS 适配器只能在既有 `/raicom/cmd_vel` gate 的唯一发布者、team-boundary、软/硬急停和 SIM backend admission 全部放行后消费建议；此模块不提供旁路，也不包含实机命令路径。底盘六维零/hold、完整 arm sequence preflight 与机械臂 swept-volume 留在己方边界内，都是 arm action 前置条件；`arm_extended` 锁存期间任意非 hold 立即停止。90 度观测转向仅为 `config/center_island_mission.json` 中的语义化配置，未把任何 SwiftPro joint 当作 90 度；默认 mapping 是 `UNRESOLVED`，因此默认配置会在臂动作前失败关闭。只有写入明确 pose mapping 且带 `SIM_TESTED` 或 `FIELD_VALIDATED` provenance 后才允许进入观测动作。

### 输入合同（没有假 YOLO）

没有模型推理实现。视觉/TF adapter 必须要么交付 `TargetCandidate`，要么明确报告 `UNAVAILABLE`；后者（及空候选）会 fail-closed 到 `TARGET_UNAVAILABLE_OR_UNSAFE`。每个候选必须使用观测时间戳完成 TF 转换，输出到现有 SwiftPro 合同的 `Base` frame `(x,y,z)` 米坐标，并携带 age、连续稳定帧、三维速度和 IK/admission 结果。选择按可抓的三维距离、再按 identifier 稳定排序；bbox 面积、像素距离和未转换 camera frame 均不能用于抓取。冻结目标后还会检查三维位置漂移，超限立即取消、不追抓。

AR 对齐仅接受 ID 4。视觉 adapter 必须在观测时间戳把 marker 转换到 `base_footprint`，再用配置的机器人前/后/侧完整平面包络计算 marker 点到包络的最短净距；**相机到 marker 的欧氏距离不能代替该净距**。净距必须新鲜且连续稳定，绝不允许低于 `hard_min=0.20m`。默认目标为 `0.23m`，先只旋转消除 bearing，再只正向微进；任一 AR stale/lost/不稳定或净距低于 hard minimum 都立即输出零速并进入 STOP。转盘时序为 `UNRESOLVED`（nominal `30s/90deg` 仅 working assumption）：相位未知时只允许同时满足连续帧数、最短稳定时长及三维速度上限的候选；若调用方知道下次旋转窗口，剩余时间必须大于 `guard + 完整抓取执行预算`，否则禁止开始或继续抓取；移动、漂移或丢失时取消且不追抓。

### 赛前 10 分钟 SIM 标定清单

1. 确认 backend 报告 `SIM`，且现有 base publisher ownership、team boundary 与 e-stop gate 均由外层检查；不要连接实体机器人。
2. 在 `center_island_mission.json` 复核 staging scenario、ID 4、机器人完整包络尺寸及 marker-to-envelope 净距定义；实测后只提高 `ar_hard_min_distance_m`（不得低于 0.20），并确保 target 的整个容差区间都不低于它。
3. 以静止 marker 校准 `ar_max_age_s`、连续稳定帧、`0.23m` 目标及容差；刻意断流一次，确认立刻 STOP/零输出。
4. 用静态已知点验证 camera/TF adapter 输出的是 `Base` 三维米坐标，调 target freshness、稳定帧、速度、漂移和 IK admission；`UNAVAILABLE` 必须失败关闭。不要临场把 YOLO 框中心直接当三维抓取点。
5. 在 arm 控制合同中映射 `camera_observation_pose` 的 90 度旋转与 pick/rear/safe pose，完成整段 IK/碰撞/己方边界 swept-volume preflight，并记录 mapping 的 `SIM_TESTED` 或 `FIELD_VALIDATED` provenance；否则核心会以 `ARM_OBSERVATION_MAPPING_UNRESOLVED` 停止。禁止把配置中的 90 度直接映射为未知 joint。
6. 将转盘相位保持 `UNRESOLVED`，除非赛方/实测证据支持 `WORKING_ASSUMPTION`；确认旋转 guard 期间不启动抓取。
7. 仅在仿真中验证失败路径的底盘六维零、吸盘关闭、安全回位和成功后的 `REFEREE_HOLD`。本包不声称实物验证。
