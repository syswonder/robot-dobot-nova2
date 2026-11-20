# Orbbec SDK/USB 相机原语

包名 `robonix.primitive.orbbec.usb.camera`，本体 Provider ID `orbbec_camera`。
使用官方 `pyorbbecsdk2` 直接读取 USB；无需 ROS 2、8765 HTTP 接口或标定程序。
参考社区 [Orbbec 相机包](https://packages.robonix.ai/packages/robonix.primitive.orbbec.camera/)
的生命周期、首帧检查和快照设计，采用独立命名空间与 typed JSON 接口，未复制 ROS 驱动。

本机原生 SDK 在私有 Windows Python 中运行，WSL 原语通过子进程管道管理它。
Linux 原生环境直接在包自己的 Python 环境运行同一 SDK worker。
Windows 运行环境为 `%LOCALAPPDATA%/Robonix-Nova2-Orbbec`，与标定环境完全独立。
首次 build 安装环境；后续不需要从标定目录读取 SDK 或脚本，不改系统 Python/PATH/USB 驱动。

原语启动后持续采集，激活须等待第一张有效图像；失活/退出关闭 stdin，通知 worker
停止 SDK pipeline 并释放 USB，超时才终止进程。它不连接机械臂、不发送夹爪指令。
未发现设备、设备被占用、标定指纹不一致或过期画面时明确报错，不替换其他相机或投影。

| 工具 | 参数 | 结果 |
|---|---|---|
| `get_state` | 无 | connected/error、帧号、图像年龄、相机标识和 metadata |
| `camera_info` | 无 | 实际 K/D、序列号、分辨率、格式和像素坐标约定 |
| `snapshot` | 无 | 原分辨率去畸变 JPEG，result_json.jpeg_base64；同时返回帧号、标识与 metadata |

每个响应为 `ok/message/result_json`；先检查 ok。完整契约前缀
`robonix/primitive/orbbec_camera/`，3 个工具均为 MCP；框架自动提供 gRPC 生命周期。
这些契约不声明 ROS RGB/depth topics，也不假称返回深度。

当前真实模式为 **Gemini 336L，CPC8763000EG，1280×720、MJPG、10 fps**，
处理最多 8 帧/秒，JPEG quality=88。彩色用实际 K/D 去畸变，保持同一 K 和原分辨率，
处理标识 `undistorted_color_same_K_v1`，与现有标定完全一致。
camera_key 包含设备、内参、分辨率、格式与处理约定；本体固定为原标定指纹。
更换相机/内参/像素处理后须重新标定，并同时更新标定文件及 expected_camera_key。

配置在本体 `robonix_manifest.yaml → primitive → orbbec_camera → config`。
全部字段见 [config.spec](config.spec)。启动与标定配置见[整机部署说明](../../README.md)。
