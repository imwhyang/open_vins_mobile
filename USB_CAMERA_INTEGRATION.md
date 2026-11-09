# OpenVINS v1.0.29：外接摄像头接入

正式引用沿用原 JitPack 坐标：

```groovy
implementation('com.github.imwhyang:open_vins_mobile:v1.0.29') {
    exclude group: 'open_vins_android.thirdparty', module: 'opencv'
}
```

## 本次调整

- `Camera2ResView.setExternalInputEnabled(true)`：在启动前选定外部输入，阻止同时打开手机 Camera2。
- `submitExternalNv21Frame(...)`：同步处理原始 NV21；使用与手机 IMU 同时基的开机纳秒时间戳。
- 同源原始帧继续用于 VIO、JPEG 和视频；外部画面由宿主已有 USB TextureView 显示。
- 外部模式隐藏 OpenVINS SurfaceView，跳过重复的显示转换，避免黑色 Surface 遮挡 USB 预览。
- `notifyExternalCameraDisconnected()`：取消等待中的截图，录像仍由完成任务流程统一收尾。
- 原生 I420 转换按连续的 Y、U、V 平面写入，修复 V 平面矩形越界引起的首帧异常。
- 默认仍使用手机相机；未调用外部输入 API 的旧接入方式保持原有流程。

## 宿主接入要点

1. 进入页面时检测实际 USB 摄像头，未接入时使用手机相机，一次任务保持同一输入源。
2. CAMERA 权限取得后，复用已有 USB 监测和授权流程。
3. 使用可见的 TextureView 和 USB 库普通预览模式，打开成功后转发 NV21。
4. 转发回调返回后可复用缓冲区；不得把同一缓冲区交给无界异步队列。
5. 关闭时先停止转发，再解绑帧回调、取消待截图并按任务生命周期关闭录像及定位。

外接镜头需与手机固定安装，并使用对应的相机与手机 IMU 标定配置。

## 已验证范围

- OPPO PEQM00 实机的 5 项 JNI 回归：方形、非方形、带行填充、640×480 和既有 NV21 输入。
- 宿主 USB 画面修复已由用户实机确认通过。
- 偶发拔出 USB 的原始闪退未作为本次已解决项宣称，后续如再出现需结合崩溃堆栈定位。

## 发布

使用 JDK 17 构建并通过既有 JitPack 配置发布。`openvins-lib` 的 Maven publication 使用
`com.github.imwhyang:open_vins_mobile:v1.0.29`，不再使用临时 `com.innovation.local` 坐标。
只提交该模块的发布配置、Java/Kotlin 与原生修复、测试及本说明，不包含 `app/src/main/cpp/` 中无关的本地文件。
