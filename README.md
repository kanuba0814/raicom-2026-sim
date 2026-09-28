# raicom-2026-sim · 睿抗机器人开发者大赛 2026 工业物料搬运

2026 睿抗机器人开发者大赛（RAICOM）工业组的 ROS Noetic / Gazebo 11 工作区，队伍获全国三等奖。机器人需要在对抗场地内识别工位、搬运物料并自主回充。本仓库只包含我们自己写的 15 个包。

## 自研内容

| 包 | 作用 |
|---|---|
| `raicom_workbench_vision` | 手眼相机识别工位：Hough 圆检测，再用色环剔除误检；结合 ALVAR 标记平面求解 4 个工位的空间位置。输入过期、歪斜或数量不符时输出无效，避免误抓 |
| `raicom_round_table_navigation` | 激光雷达检测圆台（聚类 + 圆拟合 + 多帧稳定），驱动“对准 → 靠近”的停靠状态机；用整车轮廓扫掠检查禁区 |
| `raicom_safety` | `cmd_vel` 速度闸门、定位 / SLAM 健康看门狗与急停 |
| `raicom_state_server`、`raicom_hmi` | 只读的机器人状态服务和操作员界面：没有任何控制机器人的发布者，缺失的数据源保持 `MISSING` / `UNKNOWN` |
| `raicom_core`、`raicom_platform`、`raicom_bringup` | 任务核心、平台抽象、仿真与实机启动 |
| `raicom_qr_task_sim`、`raicom_swiftpro_task_sim`、`raicom_charging_sim` | 二维码、机械臂和回充任务的仿真 |
| `raicom_commissioning` | 实机调试就绪度的只读证据评估 |
| `raicom_msgs`、`raicom_perception`、`raicom_2026_sim` | 消息定义、感知公共部分、仿真场景配置 |

仓库里有 22 个测试文件，不需要 ROS 环境即可运行（2026-09-28 实测 230 个用例全部通过）：

```bash
PP=$(ls -d $PWD/src/*/src | tr '\n' ':')
for t in src/*/test/test_*.py; do pkg=$(echo $t | cut -d/ -f2); (cd src/$pkg && PYTHONPATH=$PP:scripts:. python3 -m pytest -q ${t#src/$pkg/}); done
```

## 设计原则

- **不确定的规则不写死。** 赛前还没确认的数值集中放在 `src/raicom_2026_sim/config/provisional_competition.yaml`，空值是有意留的，防止把相互矛盾的规则伪装成确定事实；术语边界见 [`CONTEXT.md`](CONTEXT.md)。
- **只读优先。** 状态服务和 HMI 只观察、不控制；运动必须经过速度闸门和健康看门狗。
- **性能问题先测量。** 默认把两个 Gazebo 相机降到 640×480 @ 15 Hz，消除旧环境中两路 720×640 @ 30 Hz GPU 回读造成的周期性卡顿；图形验证会拒绝 `llvmpipe` / `swrast` 软件渲染回退。

开发以 Claude Code / Codex 为主力工具完成。

## 公开版说明

官方附件（Oryx 机器人描述、比赛场地模型与贴图）、第三方 `ar_track_alvar`，以及上级仓库中的 `.devcontainer/` 容器脚本和 `docs/` 设计基线都不包含在本仓库中，因此下文提到的 `start-gui.sh` / `verify-gui.sh` 无法直接运行，仅作为原工作流的记录保留。

---

## 原工作区说明：2026 魔力元宝工业组 GUI 仿真

这是与 legacy `rei-sim`、旧 Oryx 任务和当前新分支占位子模块隔离的 ROS Noetic / Gazebo 11 / RViz 工作区。

## 直接启动

在 Fedora 本地图形会话中执行：

```bash
bash .devcontainer/raicom-2026-sim/start-gui.sh
```

该命令会：

1. 校验并首次从本机官方附件 ZIP 提取 2026 Oryx 描述和 `rei_2026raicom` models；
2. 构建独立 GUI 容器；
3. 使用仅本次运行有效的 Xauthority cookie、仅 Arc 的 DRM 节点和 Mesa 25.1.7 `iris`；
4. 启动 **非 headless** 的 Gazebo `gzserver + gzclient`，以及 RViz；
5. 保持前台运行。关闭窗口或在终端按 `Ctrl-C` 会停止这个新容器并清理 cookie。

在另一个终端执行严格验证：

```bash
bash .devcontainer/raicom-2026-sim/verify-gui.sh
```

验证会拒绝缺失 `gzclient`、RViz、映射的 Xwayland 窗口、`/opt` Mesa / libdrm、Arc render 节点、Intel ARL renderer、Mesa 25.1.7、Iris PCI 绑定，或任何 `llvmpipe`/`swrast` 软件回退。

## 性能配置

默认 `balanced` 配置仅对本地兼容层的两个 Gazebo 摄像头使用 `640×480@15Hz`，避免旧环境的两个 `720×640@30Hz` GPU readback 周期性卡顿；官方附件原始文件不被修改。

如需按附件原始相机参数运行：

```bash
RAICOM_2026_CAMERA_PROFILE=official \
    bash .devcontainer/raicom-2026-sim/start-gui.sh
```

这不是最终比赛相机标定或任务行为的声明。

## 规则边界

- 详细的规则来源、确认项、禁止硬编码项和当前设计边界见 [`docs/2026工业组GUI仿真设计基线.md`](../docs/2026工业组GUI仿真设计基线.md)。
- 运行时使用的未确认值集中在 [`src/raicom_2026_sim/config/provisional_competition.yaml`](src/raicom_2026_sim/config/provisional_competition.yaml)。空值不是遗漏，而是防止把矛盾规则伪装成确定事实。
- 当前 GUI 仿真只验收官方候选 world、Oryx 模型、Gazebo/RViz GUI 和硬件 OpenGL。物块、物资岛、动态障碍、裁判门控、计分、导航、抓取、装配及回充尚不是已验证功能。

## Read-only Robot Context

`raicom_state_server` is launched by the actual GUI baseline, `raicom_2026_sim/gui.launch` (and remains available from `raicom_bringup/sim.launch` for composed bringup). It observes configured SIM topics only and has no publishers that command the robot. Its profile is `src/raicom_state_server/config/robot_profile.yaml`; unavailable sources stay `MISSING`/`UNKNOWN` and hardware subsystems remain `HW_UNVERIFIED`.

The state model deliberately separates:

- `/odom` as local odometry;
- latest `map -> base_link` TF plus AMCL covariance as map localization;
- `/gazebo/model_states` as SIM-only ground truth.

Use `tools/raicomctl context --full`, `tools/raicomctl tf map base_link`, and `tools/raicomctl trace --seconds 10` for bounded diagnostics. No raw high-rate stream is appended to Agent context.
