# Nova2 固定高度视觉抓取 skill

VLM / YOLO-OBB 顶面画框 → 平面标定中心 XY + 首点固定 Z → 目标上方 → 两指对齐短轴
→ 垂直下落 → TG9801 夹持反馈 → 原地抬升并保持物体。
`pick_and_place` 继续把物体送到 `destination` 指定的地点，并在该地点的释放高度松爪。

接口、参数与完成语义见 [CAPABILITY.md](CAPABILITY.md)，本体操作步骤与 chat 示例见
[整机部署说明](../../README.md)。本包的 `rbnx-build/venv` 独立安装依赖，
不向系统 Python、标定服务环境或旧灵巧手环境安装软件。

```bash
rbnx build -p skills/nova2_grasp
PYTHONPATH=skills/nova2_grasp skills/nova2_grasp/rbnx-build/venv/bin/python \
  -m unittest discover -s skills/nova2_grasp/tests -v
```

已加入本体 `robonix_manifest.yaml` 的 `skill:`，正常使用直接在本体目录 `rbnx boot`。
skill 启动为 INACTIVE，首次调用由 Executor 激活，初始化不运动、不使能、不操作夹爪。

相机由本体内 `primitives/orbbec_camera` 通过 SDK/USB 采集，本技能通过 Atlas/MCP 请求
原分辨率去畸变快照，核对标定指纹；不再请求 8765，也不读取标定服务口令。
VLM 使用现有 `.env` 的 `VLM_BASE_URL/VLM_API_KEY/VLM_MODEL`，
发送当前目标图像到配置的模型端点；不使用系统默认模型或隐式切换服务。

YOLO 后端已实现，但未附带训练权重；需要对应积木类别的 **YOLO-OBB** 模型：

```bash
uv pip install --python skills/nova2_grasp/rbnx-build/venv/bin/python \
  -r skills/nova2_grasp/requirements-yolo.txt
```

将 `yolo_model_path` 设为绝对路径；按需设置 `yolo_class_aliases_json`，例如
`'{"绿色积木":"green_cube"}'`。然后重启本体，调用时给 `backend="yolo"`。
普通 YOLO 水平框无法可靠求短轴，本包拒绝把它当成有向框。

参考了 [qmzs_grasp](https://github.com/syswonder/skill-qmzs-grasp-rbnx) 和
[vertical_grasp_object](https://github.com/syswonder/skill-vertical-grasp-object-rbnx) 的能力划分与
感知/动作分离方式；本包几何、检测适配、序列和原语客户端为本部署实现，不复制其机器人 SDK。
