package com.openvins.android;

import androidx.test.ext.junit.runners.AndroidJUnit4;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import static org.junit.Assert.*;

/** 使用实际 JNI 与 OpenCV 标准 NV21 转换对照，覆盖 USB 色度平面布局及非方形尺寸。 */
@RunWith(AndroidJUnit4.class)
public class ExternalYuvConversionTest {
    static { System.loadLibrary("opencv_java4"); }

    @Test public void squareI420MatchesNv21() { compare(4, 4, 0, 0, 0, false); }
    @Test public void oddChromaRowsMatchNv21() { compare(8, 6, 0, 0, 0, false); }
    @Test public void paddedPlanesMatchNv21() { compare(8, 6, 3, 2, 4, false); }
    @Test public void usbPreviewSizeMatchesNv21() { compare(640, 480, 0, 0, 0, false); }
    @Test public void existingInterleavedInputStillWorks() { compare(8, 6, 0, 0, 0, true); }

    private void compare(int width, int height, int yPadding, int uPadding, int vPadding, boolean interleaved) {
        int yStride = width + yPadding, uStride = width / 2 + uPadding, vStride = width / 2 + vPadding;
        byte[] y = new byte[yStride * height];
        byte[] u = new byte[uStride * height / 2], v = new byte[vStride * height / 2];
        byte[] nv21 = new byte[width * height * 3 / 2];
        for (int row = 0; row < height; row++) {
            for (int col = 0; col < width; col++) {
                byte luma = (byte) (16 + (row * 17 + col * 9) % 220);
                y[row * yStride + col] = luma;
                nv21[row * width + col] = luma;
            }
        }
        for (int row = 0; row < height / 2; row++) {
            for (int col = 0; col < width / 2; col++) {
                byte cb = (byte) (80 + (row * 13 + col * 3) % 90);
                byte cr = (byte) (90 + (row * 7 + col * 11) % 100);
                u[row * uStride + col] = cb;
                v[row * vStride + col] = cr;
                int offset = width * height + row * width + col * 2;
                nv21[offset] = cr;
                nv21[offset + 1] = cb;
            }
        }
        VioEngine engine = new VioEngine();
        byte[] chroma = java.util.Arrays.copyOfRange(nv21, width * height, nv21.length);
        long address = interleaved
                ? engine.processYUVToRGBA(y, chroma, null, width, height, yStride, width, 0, 2)
                : engine.processYUVToRGBA(y, u, v, width, height, yStride, uStride, vStride, 1);
        assertNotEquals(0L, address);
        // Java Mat 接管 JNI 返回的 header；release 释放像素，header 由其 finalizer 释放一次。
        Mat actual = new Mat(address);
        Mat input = new Mat(height * 3 / 2, width, CvType.CV_8UC1);
        Mat expected = new Mat();
        try {
            input.put(0, 0, nv21);
            Imgproc.cvtColor(input, expected, Imgproc.COLOR_YUV2RGBA_NV21);
            assertEquals(height, actual.rows());
            assertEquals(width, actual.cols());
            assertEquals(4, actual.channels());
            assertTrue("JNI conversion differs from NV21 reference", Core.norm(actual, expected, Core.NORM_INF) <= 1.0);
        } finally {
            actual.release();
            input.release();
            expected.release();
        }
    }
}
