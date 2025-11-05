// java
package org.firstinspires.ftc.teamcode.RobotHardware;

import android.util.Log;

import com.qualcomm.robotcore.util.RobotLog;

import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * AprilTag pipeline that attempts to use apriltageasy / OpenFTC apriltag via reflection.
 * Construct with tag size (meters) and camera intrinsics (fx, fy, cx, cy).
 * Add to your USBCamera via setPipeline(new AprilTagEasyPipeline(...))
 * Call getLatestDetections() to get a snapshot list of detections.
 */
public class AprilTagEasyPipeline {
    private static final String TAG = "AprilTagEasyPipeline";

    private final double tagSize;
    private final double fx, fy, cx, cy;

    private volatile List<Detection> latest = Collections.emptyList();

    // reflection handles
    private boolean detectorAvailable = false;
    private Object detectorInstance = null;
    private Method detectMethod = null;
    private Method setIntrinsicsMethod = null;

    public AprilTagEasyPipeline(double tagSizeMeters, double fx, double fy, double cx, double cy) {
        this.tagSize = tagSizeMeters;
        this.fx = fx;
        this.fy = fy;
        this.cx = cx;
        this.cy = cy;

        // Try apriltageasy first, then OpenFTC apriltag as fallback.
        String[] candidates = new String[] {
                "com.github.klabanov.apriltageasy.AprilTagDetector", // apriltageasy
                "org.openftc.apriltag.AprilTagDetector"               // OpenFTC
        };

        for (String clsName : candidates) {
            try {
                Class<?> detClass = Class.forName(clsName);

                // try no-arg constructor first
                Object det = null;
                try {
                    det = detClass.getDeclaredConstructor().newInstance();
                } catch (NoSuchMethodException ignored) {
                    // try constructor with TagFamily enum if present
                    for (Class<?> inner : detClass.getDeclaredClasses()) {
                        if (inner.getSimpleName().toLowerCase().contains("tagfamily")) {
                            Object[] enums = inner.getEnumConstants();
                            if (enums != null && enums.length > 0) {
                                det = detClass.getDeclaredConstructor(inner).newInstance(enums[0]);
                                break;
                            }
                        }
                    }
                }

                if (det == null) continue;

                // find detect(Mat) method
                Method maybeDetect = null;
                for (Method m : detClass.getMethods()) {
                    if (m.getName().toLowerCase().contains("detect") && m.getParameterTypes().length == 1) {
                        // accept methods taking Mat or similar
                        maybeDetect = m;
                        break;
                    }
                }

                // find a method to set intrinsics if present
                Method maybeSetIntr = null;
                for (Method m : detClass.getMethods()) {
                    String n = m.getName().toLowerCase();
                    if ((n.contains("intrin") || n.contains("lens") || n.contains("setcamera")) &&
                            m.getParameterTypes().length >= 3) {
                        maybeSetIntr = m;
                        break;
                    }
                }

                if (maybeDetect != null) {
                    detectorAvailable = true;
                    detectorInstance = det;
                    detectMethod = maybeDetect;
                    setIntrinsicsMethod = maybeSetIntr;

                    // try to set intrinsics (if detector exposes such method)
                    if (setIntrinsicsMethod != null) {
                        try {
                            Class<?>[] ptypes = setIntrinsicsMethod.getParameterTypes();
                            if (ptypes.length == 4) {
                                setIntrinsicsMethod.invoke(detectorInstance, fx, fy, cx, cy);
                            } else if (ptypes.length == 1 && ptypes[0].isArray()) {
                                double[] arr = new double[] {fx, fy, cx, cy};
                                setIntrinsicsMethod.invoke(detectorInstance, (Object) arr);
                            }
                        } catch (Exception ignored) { }
                    }

                    break;
                }
            } catch (Exception e) {
                // try next candidate
            }
        }

        if (!detectorAvailable) {
            RobotLog.w(TAG + ": apriltag detector (apriltageasy/OpenFTC) not found on classpath.");
        }
    }

    public Mat processFrame(Mat input) {
        if (!detectorAvailable) {
            Imgproc.putText(input, "AprilTag lib missing", new Point(10, 30),
                    Imgproc.FONT_HERSHEY_SIMPLEX, 0.8, new Scalar(255, 0, 0), 2);
            latest = Collections.emptyList();
            return input;
        }

        Mat gray = new Mat();
        Imgproc.cvtColor(input, gray, Imgproc.COLOR_RGBA2GRAY);

        try {
            Object raw = detectMethod.invoke(detectorInstance, gray);

            List<Detection> parsed = new ArrayList<>();
            if (raw instanceof List) {
                List<?> list = (List<?>) raw;
                for (Object detObj : list) {
                    try {
                        Class<?> rc = detObj.getClass();

                        int id = -1;
                        try {
                            id = rc.getField("id").getInt(detObj);
                        } catch (Exception e) {
                            try { id = (int) rc.getMethod("getId").invoke(detObj); } catch (Exception ignored) {}
                        }

                        double[] t = null;
                        double[] r = null;
                        try {
                            java.lang.reflect.Field tf = rc.getField("poseTranslation");
                            t = (double[]) tf.get(detObj);
                        } catch (Exception ignored) {}
                        try {
                            java.lang.reflect.Field rf = rc.getField("poseRotation");
                            r = (double[]) rf.get(detObj);
                        } catch (Exception ignored) {}

                        // corners: often stored as double[][] or List<double[]>
                        Point[] corners = new Point[4];
                        try {
                            java.lang.reflect.Field cf = rc.getField("corners");
                            Object cornersObj = cf.get(detObj);
                            if (cornersObj instanceof double[][]) {
                                double[][] c = (double[][]) cornersObj;
                                for (int i = 0; i < Math.min(4, c.length); i++) {
                                    corners[i] = new Point(c[i][0], c[i][1]);
                                }
                            } else if (cornersObj instanceof List) {
                                List<?> cl = (List<?>) cornersObj;
                                for (int i = 0; i < Math.min(4, cl.size()); i++) {
                                    Object el = cl.get(i);
                                    if (el instanceof double[]) {
                                        double[] dd = (double[]) el;
                                        corners[i] = new Point(dd[0], dd[1]);
                                    }
                                }
                            }
                        } catch (Exception ignored) {}

                        if (t == null) t = new double[]{0,0,0};
                        if (r == null) r = new double[]{0,0,0};

                        Detection d = new Detection(id, t[0], t[1], t[2], r[0], r[1], r[2], corners);
                        parsed.add(d);

                        // draw detection
                        for (int i = 0; i < 4; i++) {
                            Point a = corners[i];
                            Point b = corners[(i+1)%4];
                            if (a != null && b != null) {
                                Imgproc.line(input, a, b, new Scalar(0,255,0), 2);
                            }
                        }
                        if (corners[0] != null) {
                            Imgproc.putText(input, "id:" + id, new Point(corners[0].x, corners[0].y - 10),
                                    Imgproc.FONT_HERSHEY_SIMPLEX, 0.6, new Scalar(0,255,0), 2);
                        }

                    } catch (Exception e) {
                        Log.w(TAG, "parse detection failed: " + e.toString());
                    }
                }
            }

            latest = Collections.unmodifiableList(parsed);
        } catch (Exception e) {
            RobotLog.w(TAG + ": detection error: " + e.toString());
            latest = Collections.emptyList();
        } finally {
            gray.release();
        }

        return input;
    }

    /** Returns a snapshot list of detections. */
    public List<Detection> getLatestDetections() {
        return new ArrayList<>(latest);
    }

    /** Simple detection holder. */
    public static class Detection {
        public final int id;
        public final double tx, ty, tz;
        public final double rx, ry, rz;
        public final Point[] corners;

        public Detection(int id, double tx, double ty, double tz,
                         double rx, double ry, double rz, Point[] corners) {
            this.id = id;
            this.tx = tx; this.ty = ty; this.tz = tz;
            this.rx = rx; this.ry = ry; this.rz = rz;
            this.corners = corners;
        }
    }
}
