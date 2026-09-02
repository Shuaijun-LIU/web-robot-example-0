# Franka Demo1 与 Assembly1 两项修复

日期：2026-09-02

## 目标

1. 让 GitHub Pages 上的“保存当前姿态”真正下载 JSON，而不是调用只存在于 Vite 开发服务器的写文件接口。
2. 去掉第一步结束后 Arm 1 启动第二步时的跨 IK 分支大回转。
3. 新增英文演示入口 `Franka Demo1`，只保留 Page、Play、Reset；Play 自动串行执行现有四步。

## 根因与实现

- Pages 是纯静态站点。旧代码对 `/__manual-pose-capture` 发起 POST，Pages 将未知路径回退为 HTML，随后 `response.json()` 在 `<html>` 处报错。现在生产构建直接创建并下载格式化 JSON；仅 `import.meta.env.DEV` 下继续额外 POST 到本地 Vite 中间件，方便服务器直接读取。
- Step 1 的 Arm 1 已使用平行夹爪等价的 `-90°` 分支，而 Step 2 仍使用旧 `+90°` 分支。现在 Step 2 Arm 1 也使用 `-90°`，并从 Step 1 最终关节角重新求解 approach/contact，不改 TCP 世界坐标和夹爪闭合轴。
- `Franka Demo1` 与 `Franka Assembly1` 共用相同场景、四个动作控制器和物理判定，不复制状态机。精简页隐藏 Leva、IK、拖拽、调试、性能、键盘和 GitHub 控件，只显示英文 Page、Play、Reset 与英文运行状态；默认入口暂时改为该页。

## 验证证据

- 静态 Pages 构建浏览器下载成功：`franka-assembly1-handover-pose-01.json`，schema 为 `franka-assembly1-manual-pose-v1`，包含四个 TCP；网络记录中没有 `/__manual-pose-capture` 请求。
- 自动播放真实完成 Step 1、完整通过 Step 2 的物理接触门并自动进入 Step 3 `planning`；页面可见控件只有 Play、Reset 和一个 Page 选择器，页面文本无中文。Step 2 终态 `failure=null`，四个抓取判定均为 `ok=true`。
- Arm 1 从 Step 1 结束到 Step 2 `slow-descent` 的实测最大单关节变化为 `0.1093 rad`；新 Step 2 IK 的 approach/contact 位置误差约 `0.182/0.128 mm`，方向误差约 `0.055/0.045°`。
- 截图：`artifacts/screenshots/franka-demo1-initial.png`、`artifacts/screenshots/franka-demo1-autoplay-step2.png`。

## 边界

- 本轮没有改动 Step 3/4 的运动、接触阈值或物体位姿。
- 自动播放遇到任一步物理失败时停止并提示 Reset，不跳过现有验证门。
