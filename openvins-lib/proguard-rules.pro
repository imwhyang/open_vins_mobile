# OpenVINS Library ProGuard Rules

# Keep JNI native method names
-keepclasseswithmembernames class * {
    native <methods>;
}

# Keep OpenVINSManager public API
-keep class com.openvins.android.OpenVINSManager {
    public *;
}

# Keep Camera2ResView and its callback interface
-keep class com.openvins.android.Camera2ResView {
    *;
}
-keep class com.openvins.android.CameraFrameListener {
    *;
}

# Keep Trajectory3DView
-keep class com.openvins.android.Trajectory3DView {
    *;
}

# Keep OpenCV classes used by native code
-keep class org.opencv.** { *; }
