# openvins-lib proguard rules
# 保留 JNI 相关类和原生方法
-keepclasseswithmembernames class * {
    native <methods>;
}
-keep class com.openvins.android.VioEngine { *; }
-keep class com.openvins.android.Camera2ResView { *; }
-keep class com.openvins.android.Trajectory3DView { *; }
-keep class com.openvins.android.CameraFrameListener { *; }
