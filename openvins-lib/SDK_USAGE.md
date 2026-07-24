# OpenVINS Android SDK 接入文档

`openvins-lib` 封装了相机、IMU、OpenVINS 定位、轨迹绘制、猪圈重复拍摄判断，以及视频与 Pose 文件同步记录。

宿主项目不需要直接调用 `VioEngine`，也不需要自行注册传感器或处理相机帧。

## 1. 引入依赖

项目发布到 JitPack 后，在宿主项目中添加 JitPack 仓库和实际发布坐标：

```groovy
repositories {
    maven { url "https://jitpack.io" }
}

dependencies {
    implementation "JITPACK_GROUP:openvins-lib:VERSION"
}
```

`JITPACK_GROUP` 和 `VERSION` 请替换为仓库实际发布信息。

SDK 当前只打包 `arm64-v8a` 原生库，接入项目需要允许该 ABI。

## 2. 权限

`AndroidManifest.xml`：

```xml
<uses-permission android:name="android.permission.CAMERA" />
```

推荐使用 `getExternalFilesDir()` 作为存储根目录，不需要申请公共存储权限。

## 3. 页面布局

相机视图是必需的。页面不需要展示轨迹时，不要添加 `Trajectory3DView`：

```xml
<FrameLayout
    android:layout_width="match_parent"
    android:layout_height="match_parent">

    <com.openvins.android.Camera2ResView
        android:id="@+id/openvins_camera"
        android:layout_width="match_parent"
        android:layout_height="match_parent" />

</FrameLayout>
```

## 4. 创建和初始化 SDK

```kotlin
class BarnCaptureActivity : AppCompatActivity(), OpenVinsSdkListener {

    private lateinit var sdk: OpenVinsSdk

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_barn_capture)

        sdk = OpenVinsSdk(this, this).apply {
            bindCameraView(findViewById(R.id.openvins_camera))
            initialize(
                OpenVinsStorageConfig(
                    rootDirectory = File(
                        getExternalFilesDir(Environment.DIRECTORY_DOCUMENTS),
                        "openvins"
                    ),
                    debugLoggingEnabled = false
                )
            )

            configureBarnRevisit(
                BarnRevisitConfig(
                    pointDistanceMeters = 0.5,
                    sameSideMaxAngleDegrees = 90.0
                )
            )

            // 正式项目可关闭画面中的 init/zvupt，改用状态回调展示中文交互。
            setStatusOverlayEnabled(false)
        }

        sdk.beginInsuranceSession(
            policyId = "POLICY-001",
            barnId = "BARN-01"
        )
    }
}
```

初始化顺序建议固定为：

1. 创建 `OpenVinsSdk`。
2. 绑定 `Camera2ResView`。
3. 需要实时显示轨迹时才绑定 `Trajectory3DView`。
4. 调用 `initialize()` 配置存储。
5. 调用 `configureBarnRevisit()` 配置查重。
6. 调用 `beginInsuranceSession()` 开始业务会话。

## 5. 相机权限与生命周期

取得相机权限后通知 SDK：

```kotlin
sdk.notifyCameraPermissionGranted()
```

进入采集页面后启动定位，离开页面时停止：

```kotlin
override fun onResume() {
    super.onResume()
    sdk.start()
}

override fun onPause() {
    sdk.stop()
    super.onPause()
}
```

重复调用 `start()` 或 `stop()` 不会重复启停。

## 6. 定位状态

```kotlin
override fun onTrackingStateChanged(status: OpenVinsStatus) {
    statusText.text = when (status.state) {
        OpenVinsTrackingState.STOPPED -> "定位未开始"
        OpenVinsTrackingState.INITIALIZING -> "正在初始化，请保持画面清晰并缓慢移动手机"
        OpenVinsTrackingState.TRACKING -> "定位正常"
        OpenVinsTrackingState.STATIONARY -> "定位正常，设备静止"
        OpenVinsTrackingState.CAMERA_UNAVAILABLE -> "画面不可用，当前轨迹数据已过滤"
        OpenVinsTrackingState.ALIGNING -> "正在接续上次轨迹"
        OpenVinsTrackingState.PAUSED -> "检测到轨迹异常，已暂停记录"
    }
}
```

只有 `TRACKING` 和 `STATIONARY` 状态允许业务拍摄。`capturePen()` 会自动校验，不可靠时返回 `null` 并触发 `onError()`。

当状态为 `PAUSED` 时，可以提示用户确认：

```kotlin
sdk.resumeTrajectory()
```

系统会从最后可靠轨迹点进行恢复和坐标对齐。

## 7. 同步记录视频和 Pose

开始：

```kotlin
val recording = sdk.startEvidenceRecording(
    recordingId = "POLICY-001-BARN-01"
)
```

停止：

```kotlin
val result = sdk.stopEvidenceRecording()
val videoFile = result?.videoFile
val poseFile = result?.poseFile
```

请将 `videoFile` 和 `poseFile` 作为同一份采集证据上传。

默认目录结构：

```text
openvins/
├── photos/
├── videos/
└── poses/
    └── yyyy-MM-dd_HH-mm-sspose0.csv
```

正式环境建议设置 `debugLoggingEnabled = false`，只生成后台绘制轨迹需要的 `pose0.csv`。

## 8. 拍照、记录 Pose 和查重

正式业务使用 `capturePen()`，不要把拍照和 Pose 查询拆成两次调用：

```kotlin
val request = PenCheckRequest(
    penId = "PEN-012",
    expectedPigCount = 36,
    operatorRemark = "过道左侧"
)

sdk.capturePen(request) { result ->
    if (result == null) {
        return@capturePen
    }

    val photo = result.photoFile
    val record = result.record

    if (record.revisitResult.isRevisit) {
        showDuplicateDialog()
    } else {
        upload(photo, record)
    }
}
```

一次成功调用会返回：

- 当前原始相机画面。
- 拍摄时冻结的 Pose。
- 拍摄时的完整轨迹快照。
- 与历史拍摄记录的查重结果。
- 业务传入的 `policyId`、`barnId` 和 `penId`。

照片保存失败时不会生成业务记录，也不会污染查重候选。

## 9. 猪圈查重规则

当前规则针对“中间过道、两侧猪圈”场景：

1. 将相机拍摄方向投影到水平面。
2. 两次镜头夹角小于 `sameSideMaxAngleDegrees` 时认为拍摄同侧。
3. 只有同侧记录才比较水平位置。
4. 同侧且距离小于 `pointDistanceMeters` 时提示重复。
5. 异侧即使拍摄人员站在同一位置，也不提示重复。
6. 已提示重复的 Pose 会保留在业务记录中，但不会成为新的查重候选。

推荐初始配置：

```kotlin
BarnRevisitConfig(
    pointDistanceMeters = 0.5,
    sameSideMaxAngleDegrees = 90.0,
    revisitMileageMeters = 1.5,
    routeMaxDistanceMeters = 0.5,
    routeImpactThreshold = 0.35,
    shiftingMileageMeters = 1.0
)
```

其中：

- `pointDistanceMeters`：同侧拍摄点判重距离。
- `sameSideMaxAngleDegrees`：仍视为同侧的最大镜头夹角。
- `revisitMileageMeters`：局部路径查重使用的最近里程。
- `routeMaxDistanceMeters`：路径允许偏离历史路径的距离。
- `routeImpactThreshold`：路径重合度阈值。
- `shiftingMileageMeters`：短时间移动过快的检测阈值。

可以使用 `previewBarnRevisit()` 调试当前结果，它不会写入候选；`checkBarnRevisit()` 会提交非重复候选，正式业务优先使用 `capturePen()`。

## 10. 会话结果

### 不展示控件，导出最终轨迹图

页面不需要添加或绑定 `Trajectory3DView`。结束采集前后直接调用：

```kotlin
val output = File(storageRoot, "trajectory/final.jpg")
sdk.exportTrajectoryImage(
    file = output,
    config = TrajectoryImageConfig(
        width = 1200,
        height = 1200,
        gridSpacingMeters = 1.0f
    )
) { imageFile ->
    if (imageFile != null) {
        uploadTrajectoryImage(imageFile)
    }
}
```

该方法根据 SDK 内保存的最终轨迹生成俯视图，包含米制网格、轨迹线和最终方向，不依赖 View 的尺寸与可见状态。

旧的 `captureTrajectoryImage()` 截取的是 `Trajectory3DView` 的 OpenGL 画面，仅适用于页面已经展示并绑定轨迹控件的情况。

获取当前记录：

```kotlin
val records: List<PenCaptureRecord> = sdk.getPenCaptureRecords()
```

导出 JSON：

```kotlin
val json: String = sdk.exportSessionJson()
```

结束业务会话：

```kotlin
val finalRecords = sdk.finishInsuranceSession()
```

`pose0.csv` 是设备连续运动轨迹；`PenCaptureRecord` 是拍照时刻的业务点位记录，两者用途不同。

## 11. 标定与设备差异

SDK 首次初始化会将以下配置复制到应用私有目录：

- `estimator_config.yaml`
- `kalibr_imu_chain.yaml`
- `kalibr_imucam_chain.yaml`

相机内参、相机与 IMU 外参、时间偏移会直接影响轨迹和查重效果。不同手机型号不能只根据 IMU 采样率判断效果是否一致，正式发布前应验证目标设备的相机设置和标定参数。

完整示例见：

```text
app/src/main/java/com/openvins/android/MainActivity.kt
```
