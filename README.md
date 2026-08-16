# 2026 魔力元宝工业组 GUI 仿真工作区

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
