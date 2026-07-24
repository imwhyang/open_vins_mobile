# Barn Insurance Integration

This project should be consumed as a trajectory and duplicate-check SDK inside
the main insurance app shooting page.

## Keep

- `openvins-lib`: Android library, camera preview, IMU collection, OpenVINS JNI,
  trajectory rendering, pen duplicate/revisit checks.
- `openvins-lib/src/main/assets/config`: device calibration and estimator
  configuration. Replace these three YAML files with calibration files for the
  real shooting device.
- `thirdparty/opencv`, `thirdparty/eigen3`, `thirdparty/boost`: native build
  dependencies required by OpenVINS.
- `app_device/config`: development copy of calibration files.

## Optional For Development Only

- `app`: demo application. The production insurance app should depend on
  `openvins-lib` directly instead of embedding this demo UI.
- `app_kalibr`: calibration helper scripts and sample calibration outputs.
- `docs`: screenshots and videos for the original demo.
- `openvins-lib/src/main/cpp/open_vins/docs`, `ov_data`, `ov_eval`: upstream
  documentation, datasets, and evaluation tools. They are useful for source
  reference, but they are not part of the production Android runtime path.

## Shooting Page Flow

1. Request camera permission in the main app.
2. Add `Camera2ResView` to the shooting page. Add `Trajectory3DView` only when
   an on-screen route preview is needed.
3. Create `OpenVinsSdk`, bind the views, call `initialize(OpenVinsStorageConfig)`, then call
   `beginInsuranceSession(policyId, barnId)`.
4. Call `start()` when the user enters the shooting page.
5. Call `startEvidenceRecording()` to record MP4 and `pose0.csv` together.
6. At each pen checkpoint, call `capturePen(PenCheckRequest(...))`. The callback
   returns the photo, current Pose and duplicate-check result as one record.
7. Use `PenCaptureRecord.revisitResult`:
   - `isRetrieve`: current checkpoint is close to a previously captured pen.
   - `isDuplicated`: recent route segment overlaps previous route.
   - `isMovingFast`: operator movement is too unstable for reliable capture.
8. Call `stopEvidenceRecording()` and upload its video/Pose file pair.
9. Upload `exportSessionJson()` with the app's photo/video evidence.
10. Call `stop()` when leaving the page and `finishInsuranceSession()` when the
   barn capture is complete.

## Minimal Kotlin Example

```kotlin
val sdk = OpenVinsSdk(context, listener)
sdk.initialize(recordFolder = File(context.filesDir, "openvins-records"))
sdk.bindCameraView(cameraView)
sdk.bindTrajectoryView(trajectoryView)
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

sdk.beginInsuranceSession(policyId = "P20260630001", barnId = "BARN-A")
sdk.notifyCameraPermissionGranted()
sdk.start()

sdk.capturePen(
    PenCheckRequest(
        penId = "PEN-12",
        expectedPigCount = 36,
        operatorRemark = "left aisle"
    )
) { result ->
    if (result?.record?.revisitResult?.isRevisit == true) {
        // Block or warn before allowing duplicate underwriting evidence.
    }
}
```

## Practical Notes

- OpenVINS gives relative visual-inertial trajectory, not absolute GPS position.
  For barns and pens, compare within the same capture session.
- Calibration quality is critical. Use the target phone model and camera setting
  used by the production shooting page.
- Keep the phone motion slow and continuous. Sudden turns, dark pens, repeated
  textures, and blocked camera frames will reduce reliability.
- GPL-3.0 applies to OpenVINS and this mobile code. Confirm license obligations
  before shipping inside a closed-source commercial app.
