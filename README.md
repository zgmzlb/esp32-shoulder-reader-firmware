# ESP32-S3 绘本伴读开发板固件

本目录包含 ESP32-S3 N16R8 开发板的生产固件、翻页确认逻辑、自定义板卡定义及 PlatformIO 构建配置。

## 配置与编译

安装 PlatformIO 后，将 src/device_config.local.example.h 复制为 src/device_config.local.h，填写本机 Wi-Fi 名称、密码和设备 ID。私有配置文件已加入 Git 忽略规则，不会被提交。

在本目录执行：

    python -m platformio run
    python -m platformio run --target upload --upload-port COM7
    python -m platformio device monitor --port COM7 --baud 115200

如果开发板不是 COM7，请将命令中的端口替换为 Windows 设备管理器中显示的串口。

## 固件功能

- 连续采集摄像头画面，并通过连续稳定帧确认翻页后再上传。
- 通过 BLE 广播设备身份，提供 Wi-Fi 配置、换书控制和状态通知特征。BLE 服务 UUID：12345678-1234-5678-1234-56789abcdef0。
- 每 4 秒向服务端发送一次心跳。
- 收到换书命令后拍摄封面，并调用 POST /api/v1/devices/{device_id}/book-swap。服务端返回 HTTP 200 时，固件读取 data.bookId 并作为当前书籍 ID 保存。
- 上传翻页事件时携带当前 bookId 和 clientEventId；同一事件重试时复用同一个事件 ID。

当前固件尚未实现书页透视或曲面展平，因此请求中的 flattened 为 false。
