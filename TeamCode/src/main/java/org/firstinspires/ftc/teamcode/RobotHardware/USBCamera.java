package org.firstinspires.ftc.teamcode.RobotHardware;

import android.content.Context;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvCamera.AsyncCameraOpenListener;
import org.opencv.core.Mat;

/**
 * Lightweight USB webcam helper for FTC (EasyOpenCV).
 * Usage:
 *   USBCamera cam = new USBCamera(hardwareMap, "Webcam 1", 640, 480);
 *   cam.init();
 *   cam.setPipeline(new USBCamera.SimpleFrameGrabber());
 *   cam.startStreaming(OpenCvCameraRotation.SIDEWAYS_LEFT);
 *   Mat frame = cam.getLatestFrame(); // may be null
 *   cam.stopStreaming();
 */
public class USBCamera {
    private final HardwareMap hw;
    private final String deviceName;
    private final int width;
    private final int height;

    private OpenCvCamera camera;
    private volatile boolean streaming = false;
    private SimpleFrameGrabber frameGrabber = null;

    public USBCamera(HardwareMap hardwareMap, String cameraDeviceName, int width, int height) {
        this.hw = hardwareMap;
        this.deviceName = cameraDeviceName;
        this.width = width;
        this.height = height;
    }

    /**
     * Initializes the camera object. Must be called before startStreaming().
     */
    public void init() {
        try {
            WebcamName webcamName = hw.get(WebcamName.class, deviceName);
            Context ctx = hw.appContext;
            int monitorViewId = ctx.getResources().getIdentifier("cameraMonitorViewId", "id", ctx.getPackageName());
            camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, monitorViewId);
        } catch (Exception e) {
            RobotLog.e("USBCamera.init: failed to create webcam: " + e.toString());
            camera = null;
        }
    }

    /**
     * Start streaming asynchronously. Provide desired rotation (e.g. OpenCvCameraRotation.UPRIGHT).
     */
    public void startStreaming(final OpenCvCameraRotation rotation) {
        if (camera == null) {
            RobotLog.w("USBCamera.startStreaming: camera not initialized");
            return;
        }

        camera.openCameraDeviceAsync(new AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                try {
                    camera.startStreaming(width, height, rotation);
                    streaming = true;
                } catch (Exception e) {
                    RobotLog.e("USBCamera.startStreaming: " + e.toString());
                }
            }

            @Override
            public void onError(int error) {
                RobotLog.e("USBCamera.openCameraDeviceAsync onError: " + error);
            }
        });
    }

    /**
     * Stops streaming and closes the camera device.
     */
    public void stopStreaming() {
        if (camera != null) {
            try {
                camera.stopStreaming();
                camera.closeCameraDevice();
            } catch (Exception e) {
                RobotLog.w("USBCamera.stopStreaming: " + e.toString());
            }
        }
        streaming = false;
        if (frameGrabber != null) {
            frameGrabber.release();
        }
    }

    public boolean isStreaming() {
        return streaming;
    }

    /**
     * Set an OpenCvPipeline. If you pass an instance of SimpleFrameGrabber it will be used to
     * retrieve the latest frame via getLatestFrame().
     */
    public void setPipeline(OpenCvPipeline pipeline) {
        if (camera == null) {
            RobotLog.w("USBCamera.setPipeline: camera not initialized");
            return;
        }
        camera.setPipeline(pipeline);
        if (pipeline instanceof SimpleFrameGrabber) {
            frameGrabber = (SimpleFrameGrabber) pipeline;
        } else {
            frameGrabber = null;
        }
    }

    /**
     * Returns a copy of the latest frame captured by SimpleFrameGrabber pipeline, or null if none.
     * Caller is responsible for releasing the Mat when finished.
     */
    public Mat getLatestFrame() {
        if (frameGrabber == null) return null;
        return frameGrabber.getLatestMat();
    }

    /**
     * Simple frame-grabbing pipeline that stores the last frame safely.
     * Use as: setPipeline(new USBCamera.SimpleFrameGrabber());
     */
    public static class SimpleFrameGrabber extends OpenCvPipeline {
        private Mat last = null;

        @Override
        public Mat processFrame(Mat input) {
            synchronized (this) {
                if (last != null) {
                    last.release();
                    last = null;
                }
                if (input != null) {
                    last = input.clone();
                }
            }
            // Return the original frame for display downstream.
            return input;
        }

        /**
         * Returns a cloned Mat copy of the latest frame, or null if none.
         * Caller should call release() on the returned Mat when done.
         */
        public Mat getLatestMat() {
            synchronized (this) {
                return (last == null) ? null : last.clone();
            }
        }

        /**
         * Release internal storage.
         */
        public void release() {
            synchronized (this) {
                if (last != null) {
                    last.release();
                    last = null;
                }
            }
        }
    }
}
