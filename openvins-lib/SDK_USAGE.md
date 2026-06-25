# OpenVINS Android SDK Usage

`openvins-lib` exposes a high-level SDK wrapper for apps that need phone-side
visual-inertial positioning, trajectory rendering, and barn revisit detection.

## Main Classes

- `OpenVinsSdk`: lifecycle controller for camera frames, IMU, OpenVINS, trajectory updates, and revisit checks.
- `Camera2ResView`: camera preview and frame provider.
- `Trajectory3DView`: optional OpenGL trajectory renderer.
- `OpenVinsSdkListener`: callbacks for pose, trajectory, revisit result, and errors.
- `BarnRevisitResult`: result model for barn/pen duplicate route or point checks.

## Minimal Layout

```xml
<FrameLayout
    android:layout_width="match_parent"
    android:layout_height="match_parent">

    <com.openvins.android.Camera2ResView
        android:id="@+id/openvins_camera"
        android:layout_width="match_parent"
        android:layout_height="match_parent" />

    <com.openvins.android.Trajectory3DView
        android:id="@+id/openvins_trajectory"
        android:layout_width="match_parent"
        android:layout_height="match_parent" />
</FrameLayout>
```

## Activity Integration

```kotlin
class BarnActivity : AppCompatActivity(), OpenVinsSdkListener {
    private lateinit var sdk: OpenVinsSdk

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_barn)

        sdk = OpenVinsSdk(this, this)
        sdk.initialize(
            recordFolder = File(
                Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOCUMENTS),
                "openvins/pose"
            )
        )
        sdk.configureBarnRevisit(
            mapOf(
                "revisitMileage" to 1.5,
                "maxDistance" to 0.5,
                "impactThreshold" to 0.35,
                "absPoseErrRotFro" to 0.2,
                "pointDistance" to 0.5,
                "shiftingMileage" to 1.0
            )
        )
        sdk.bindCameraView(findViewById(R.id.openvins_camera))
        sdk.bindTrajectoryView(findViewById(R.id.openvins_trajectory))
    }

    override fun onResume() {
        super.onResume()
        sdk.notifyCameraPermissionGranted()
        sdk.start()
    }

    override fun onPause() {
        sdk.stop()
        super.onPause()
    }

    fun onCheckBarnClicked() {
        val result = sdk.checkBarnRevisit()
        // result?.isRevisit == true means the current pose/path matches previous candidates.
    }

    override fun onBarnRevisitChecked(result: BarnRevisitResult) {
        // Use result.isRetrieve for point-level revisit.
        // Use result.isDuplicated for trajectory-level duplicate route.
        // Use result.isMovingFast to flag unstable movement.
    }
}
```

## Calibration Files

The SDK copies these files from `assets/config` into the app private config
folder on first initialization:

- `estimator_config.yaml`
- `kalibr_imu_chain.yaml`
- `kalibr_imucam_chain.yaml`

For a real barn deployment, replace these files with calibration results for
the target Android device. Poor camera/IMU calibration will directly reduce
revisit detection reliability.

## Barn Revisit Flow

1. Call `sdk.start()` when entering the scanning page.
2. Let OpenVINS initialize while the user moves the phone.
3. At each pen/barn checkpoint, call `sdk.checkBarnRevisit()`.
4. Store or upload `OpenVinsTrajectory` from `onTrajectoryChanged`.
5. Call `sdk.captureTrajectoryImage(...)` when a trajectory map image is needed.

## Revisit Thresholds

- `pointDistance`: point-level revisit distance threshold, in meters.
- `absPoseErrRotFro`: point-level orientation threshold.
- `maxDistance`: route-level distance neighborhood, in meters.
- `revisitMileage`: recent route length used as the query segment, in meters.
- `impactThreshold`: route duplicate confidence threshold.
- `shiftingMileage`: movement distance threshold in a short window, used to flag unstable scanning.
