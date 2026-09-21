package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.NetworkTableEntry;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.VideoSink;
import edu.wpi.first.cameraserver.CameraServer;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.json.JSONArray;
import org.json.JSONObject;

import java.nio.charset.StandardCharsets;
import java.nio.file.Path;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.*;
import java.util.function.BooleanSupplier;

public class VisionSubsystem extends SubsystemBase {

    private static final boolean USE_JSON_VISION = true;
    private static final String JSON_PATH = "/home/lvuser/vision_data.json";
    private static final String MARKER_SCRIPT_NAME = "vision_aruco.py";
    private static final String MARKER_SCRIPT_ENV = "VISION_ARUCO_SCRIPT";

    public static final String MODE_MARKER = "MARKER";
    public static final String MODE_COLOR  = "COLOR";

    // Preserve the project's existing marker-first behavior.  Pressing the
    // physical Start button switches to COLOR mode.
    private String visionMode = MODE_MARKER;

    // Camera objects are created only in COLOR mode.  In MARKER mode the
    // Python process owns /dev/video0 and writes the marker result to JSON.
    private UsbCamera camera;
    private CvSink cvSink;
    private CvSource outputStream;
    private Process markerProcess;
    private long nextColorCameraAttemptMs = 0;
    private int colorCameraAttempts = 0;
    private long nextMarkerProcessAttemptMs = 0;
    private int colorCameraGeneration = 0;
    private String activeCameraName;
    private String activeOutputName;

    private boolean processed = false;

    private final BooleanSupplier modeButtonSupplier;
    private boolean previousModeButton = false;

    private String currentDetection = "None";
    private String savedVisionResult = "None";

    private double visionOffsetX = 0;
    private double visionObjectWidth = 0;

    private boolean visionFound = false;

    // =========================================================
    // ROI
    // =========================================================

    private double roiXPercent = 0.0;
    private double roiYPercent = 0.2;
    private double roiWidthPercent = 1.0;
    private double roiHeightPercent = 0.6;

    // =========================================================
    // MARKER
    // =========================================================

    private final MarkerTemplate markerTemplate =
            new MarkerTemplate();

    public double getVisionObjectWidth() {
        return visionObjectWidth;
    }

    // =========================================================
    // SHUFFLEBOARD
    // =========================================================

    private final ShuffleboardTab tab =
            Shuffleboard.getTab("Vision");

    private final NetworkTableEntry sbOffsetX =
            tab.add("Offset X", 0).getEntry();

    private final NetworkTableEntry sbFound =
            tab.add("Found", false).getEntry();

    private final NetworkTableEntry sbDetected =
            tab.add("Detected", "None").getEntry();

    private final NetworkTableEntry sbMode =
            tab.add("Vision Mode", MODE_MARKER).getEntry();

    private final NetworkTableEntry sbProcessed =
            tab.add("Processed Enabled", false).getEntry();

    private final NetworkTableEntry sbTemplates =
            tab.add("Marker Templates", 0).getEntry();

    private final NetworkTableEntry sbRoiY =
            tab.add("ROI Y%", 0.2).getEntry();

    private final NetworkTableEntry sbRoiH =
            tab.add("ROI H%", 0.6).getEntry();

    private final Map<String, NetworkTableEntry> savedVars =
            new HashMap<>();

    private final Map<String, NetworkTableEntry> customDoubleEntries =
            new HashMap<>();

    private final Map<String, NetworkTableEntry> customStringEntries =
            new HashMap<>();

    // =========================================================
    // CONSTRUCTOR
    // =========================================================

    public VisionSubsystem(BooleanSupplier modeButtonSupplier) {

        this.modeButtonSupplier = modeButtonSupplier;

        // JSON vision is processed by Python on the VMX camera host. Loading
        // template images would call OpenCV imgcodecs and crash on this target.
        if (!USE_JSON_VISION) {
            markerTemplate.loadTemplates();
            sbTemplates.setDouble(markerTemplate.getTemplateCount());
        }

        // Start in the existing marker mode. If the script is not present,
        // the robot still starts and reports the problem in Shuffleboard.
        startMarkerProcess();
    }

    // =========================================================
    // MODE
    // =========================================================

    public void setMode(String mode) {
        if (!MODE_MARKER.equals(mode) && !MODE_COLOR.equals(mode)) {
            return;
        }

        if (MODE_COLOR.equals(mode) && !MODE_COLOR.equals(visionMode)) {
            switchToColorMode();
        } else if (MODE_MARKER.equals(mode) && !MODE_MARKER.equals(visionMode)) {
            switchToMarkerMode();
        }
    }

    public void setProcessed(boolean enable) {
        processed = enable;
    }

    public void enableScanning(boolean enable) {
        processed = enable;
    }

    public void reloadMarkerTemplates() {

        if (USE_JSON_VISION) {
            return;
        }

        markerTemplate.loadTemplates();

        sbTemplates.setDouble(
                markerTemplate.getTemplateCount()
        );
    }

    // =========================================================
    // ROI
    // =========================================================

    public void setROI(
            double xPct,
            double yPct,
            double widthPct,
            double heightPct) {

        roiXPercent =
                Math.max(0, Math.min(1, xPct));

        roiYPercent =
                Math.max(0, Math.min(1, yPct));

        roiWidthPercent =
                Math.max(0, Math.min(1, widthPct));

        roiHeightPercent =
                Math.max(0, Math.min(1, heightPct));
    }

    public void resetROI() {

        roiXPercent = 0.0;
        roiYPercent = 0.0;

        roiWidthPercent = 1.0;
        roiHeightPercent = 1.0;
    }

    // =========================================================
    // GETTERS
    // =========================================================

    public String getCurrentDetection() {
        return currentDetection;
    }

    public String getSavedVisionResult() {
        return savedVisionResult;
    }

    public double getVisionOffsetX() {
        return visionOffsetX;
    }

    public boolean isVisionFound() {
        return visionFound;
    }

    // =========================================================
    // SHUFFLEBOARD HELPERS
    // =========================================================

    public void setText(
            String name,
            String value) {

        if (!customStringEntries.containsKey(name)) {

            customStringEntries.put(
                    name,
                    tab.add(
                            name,
                            value != null
                                    ? value
                                    : "None"
                    ).getEntry()
            );
        }

        customStringEntries
                .get(name)
                .setString(
                        value != null
                                ? value
                                : "None"
                );
    }

    public void setValue(
            String name,
            double value) {

        if (!customDoubleEntries.containsKey(name)) {

            customDoubleEntries.put(
                    name,
                    tab.add(name, value)
                            .getEntry()
            );
        }

        customDoubleEntries
                .get(name)
                .setDouble(value);
    }

    public void saveVariableOnce(
            String name,
            String value) {

        if (!savedVars.containsKey(name)) {

            NetworkTableEntry entry =
                    tab.add(
                            name,
                            value != null
                                    ? value
                                    : "None"
                    ).getEntry();

            entry.setString(
                    value != null
                            ? value
                            : "None"
            );

            savedVars.put(
                    name,
                    entry
            );
        }
    }

    // =========================================================
    // RESET
    // =========================================================

    public void resetCameraScan() {

        savedVisionResult = "None";
        currentDetection = "None";

        processed = false;

        visionFound = false;

        visionOffsetX = 0;
        visionObjectWidth = 0;
    }

    // =========================================================
    // PERIODIC
    // =========================================================

    @Override
    public void periodic() {

        updateModeFromButton();

        sbProcessed.setBoolean(processed);
        sbMode.setString(visionMode);

        sbRoiY.setDouble(roiYPercent);
        sbRoiH.setDouble(roiHeightPercent);

        if (USE_JSON_VISION && MODE_MARKER.equals(visionMode)) {
            ensureMarkerProcessRunning();
            readJsonVision();
            return;
        }

        if (MODE_COLOR.equals(visionMode) && cvSink == null) {
            long now = System.currentTimeMillis();
            if (now >= nextColorCameraAttemptMs) {
                startColorCamera();
                nextColorCameraAttemptMs = now + 250;
            }
        }

        if (!processed) {
            return;
        }

        String result = detectObject();

        currentDetection = result;

        sbDetected.setString(result);
        sbFound.setBoolean(visionFound);
        sbOffsetX.setDouble(visionOffsetX);

        if (visionFound) {
            savedVisionResult = result;
        }
    }

    private void updateModeFromButton() {
        boolean pressed = modeButtonSupplier != null
                && modeButtonSupplier.getAsBoolean();

        if (pressed && !previousModeButton) {
            if (MODE_MARKER.equals(visionMode)) {
                switchToColorMode();
            } else {
                switchToMarkerMode();
            }

            resetDetectionState();
            processed = true;
        }

        previousModeButton = pressed;
    }

    /** Switch to Java/OpenCV color detection after stopping Python. */
    private void switchToColorMode() {
        stopMarkerProcess();
        visionMode = MODE_COLOR;
        colorCameraAttempts = 0;
        nextColorCameraAttemptMs = 0;
        startColorCamera();
    }

    /** Switch to Python/JSON marker detection after releasing Java's camera. */
    private void switchToMarkerMode() {
        stopColorCamera();
        visionMode = MODE_MARKER;
        nextMarkerProcessAttemptMs = 0;
        startMarkerProcess();
    }

    private void startColorCamera() {
        if (cvSink != null) {
            return;
        }

        try {
            colorCameraAttempts++;
            colorCameraGeneration++;
            activeCameraName = "Color Camera " + colorCameraGeneration;
            activeOutputName = "Vision " + colorCameraGeneration;

            // Always reopen the physical camera at index 0.  The no-argument
            // WPILib 2020 overload increments the device index on every call.
            camera = CameraServer.getInstance()
                    .startAutomaticCapture(activeCameraName, 0);
            if (camera == null || !camera.isValid()) {
                throw new IllegalStateException("USB camera is not valid");
            }
            camera.setResolution(320, 240);
            camera.setFPS(20);
            cvSink = CameraServer.getInstance().getVideo(camera);
            outputStream = CameraServer.getInstance()
                    .putVideo(activeOutputName, 320, 240);
            if (cvSink == null || outputStream == null
                    || !cvSink.isValid() || !outputStream.isValid()) {
                throw new IllegalStateException("Camera stream is not valid");
            }
            setText("Camera Owner", "Java COLOR");
            setText("Camera Attempts", Integer.toString(colorCameraAttempts));
        } catch (Exception e) {
            closeColorCameraResources();
            setText("Camera Error", e.getClass().getSimpleName());
        }
    }

    private void stopColorCamera() {
        // Sinks/sources must be closed before the camera so /dev/video0 is
        // released for the Python process.
        closeColorCameraResources();
        setText("Camera Owner", "Python MARKER");
    }

    private void closeColorCameraResources() {
        CameraServer cameraServer = CameraServer.getInstance();

        if (cvSink != null) {
            cvSink.close();
            cvSink = null;
        }

        if (activeCameraName != null) {
            closeAndRemoveServer(cameraServer, "opencv_" + activeCameraName);
            closeAndRemoveServer(cameraServer, "serve_" + activeCameraName);
        }

        if (outputStream != null) {
            outputStream.close();
            outputStream = null;
        }

        if (activeOutputName != null) {
            closeAndRemoveServer(cameraServer, "serve_" + activeOutputName);
            cameraServer.removeCamera(activeOutputName);
        }

        if (camera != null) {
            camera.close();
            camera = null;
        }

        if (activeCameraName != null) {
            cameraServer.removeCamera(activeCameraName);
        }

        activeCameraName = null;
        activeOutputName = null;
    }

    private void closeAndRemoveServer(CameraServer cameraServer, String name) {
        VideoSink server = cameraServer.getServer(name);
        if (server != null) {
            server.close();
        }
        cameraServer.removeServer(name);
    }

    private Path findMarkerScript() {
        String configured = System.getenv(MARKER_SCRIPT_ENV);
        if (configured != null && !configured.trim().isEmpty()) {
            Path path = Paths.get(configured.trim());
            return Files.isRegularFile(path) ? path : null;
        }

        Path[] candidates = {
                Paths.get("/home/lvuser/deploy/vision_aruco.py"),
                Paths.get("/home/lvuser/vision_aruco.py"),
                Paths.get("/home/pi/vision_aruco.py")
        };

        for (Path candidate : candidates) {
            if (Files.isRegularFile(candidate)) {
                return candidate;
            }
        }
        return null;
    }

    private void startMarkerProcess() {
        if (markerProcess != null && markerProcess.isAlive()) {
            return;
        }

        markerProcess = null;
        nextMarkerProcessAttemptMs = System.currentTimeMillis() + 1000;

        Path script = findMarkerScript();
        if (script == null) {
            setText("Marker Process", "Script not found");
            return;
        }

        try {
            ProcessBuilder builder = new ProcessBuilder("python3", script.toString());
            builder.environment().put("PYTHONUNBUFFERED", "1");
            builder.redirectErrorStream(true);
            builder.redirectOutput(ProcessBuilder.Redirect.appendTo(
                    Paths.get("/home/lvuser/vision_aruco.log").toFile()));
            markerProcess = builder.start();
            setText("Marker Process", "Running");
            setText("Marker Script", script.toString());
            setText("Camera Owner", "Python MARKER");
        } catch (Exception e) {
            markerProcess = null;
            setText("Marker Process", "Start failed: " + e.getClass().getSimpleName());
        }
    }

    private void ensureMarkerProcessRunning() {
        if (markerProcess != null && markerProcess.isAlive()) {
            return;
        }

        if (markerProcess != null) {
            try {
                setText("Marker Process", "Exited: " + markerProcess.exitValue());
            } catch (IllegalThreadStateException ignored) {
                return;
            }
            markerProcess = null;
        }

        long now = System.currentTimeMillis();
        if (now >= nextMarkerProcessAttemptMs) {
            startMarkerProcess();
        }
    }

    private void stopMarkerProcess() {
        if (markerProcess != null && markerProcess.isAlive()) {
            markerProcess.destroy();
            try {
                if (!markerProcess.waitFor(1, java.util.concurrent.TimeUnit.SECONDS)) {
                    markerProcess.destroyForcibly();
                }
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                markerProcess.destroyForcibly();
            }
        }
        markerProcess = null;

        // Also stop a script that was started manually from the VMX-pi shell.
        // The pattern is restricted to this exact script name.
        try {
            Process killer = new ProcessBuilder("pkill", "-f", MARKER_SCRIPT_NAME)
                    .start();
            killer.waitFor(1, java.util.concurrent.TimeUnit.SECONDS);
        } catch (Exception ignored) {
            // No matching process is a normal case.
        }
        setText("Marker Process", "Stopped");
    }

    private void resetDetectionState() {
        currentDetection = "None";
        visionFound = false;
        visionOffsetX = 0;
        visionObjectWidth = 0;
        sbDetected.setString(currentDetection);
        sbFound.setBoolean(false);
        sbOffsetX.setDouble(0);
    }

    private void readJsonVision() {
        visionFound = false;
        visionOffsetX = 0;
        visionObjectWidth = 0;

        try {
            String text = new String(
                    Files.readAllBytes(Paths.get(JSON_PATH)),
                    StandardCharsets.UTF_8);
            JSONObject root = new JSONObject(text);
            JSONArray targets = root.optJSONArray("targets");
            int count = targets == null ? 0 : targets.length();

            if (count > 0) {
                JSONObject target = targets.optJSONObject(0);
                if (target != null) {
                    int id = target.optInt("id", -1);
                    double x = target.optDouble("x", 160.0);
                    visionFound = root.optBoolean("found", true);
                    visionOffsetX = x - 160.0;
                    currentDetection = id >= 0 ? "Marker_" + id : "Marker";
                    savedVisionResult = currentDetection;
                    sbDetected.setString(currentDetection);
                    sbFound.setBoolean(visionFound);
                    sbOffsetX.setDouble(visionOffsetX);
                    setValue("Target Y", target.optDouble("y", 120.0));
                    setValue("Target ID", id);
                    setValue("Target Count", count);
                    return;
                }
            }

            currentDetection = "None";
            sbDetected.setString(currentDetection);
            sbFound.setBoolean(false);
            sbOffsetX.setDouble(0);
            setValue("Target Count", 0);
        } catch (Exception e) {
            currentDetection = "Vision JSON unavailable";
            sbDetected.setString(currentDetection);
            sbFound.setBoolean(false);
            sbOffsetX.setDouble(0);
        }
    }

    // =========================================================
    // CAMERA + ROI + DETECTION
    // =========================================================

    private String detectObject() {

        Mat work = new Mat();

        // Сбрасываем данные предыдущего кадра
        visionFound = false;
        visionOffsetX = 0;
        visionObjectWidth = 0;

        if (cvSink == null || outputStream == null) {
            work.release();
            return "Color camera unavailable";
        }

        if (cvSink.grabFrame(work) == 0) {

            work.release();

            return "None";
        }

        int frameW = work.cols();
        int frameH = work.rows();

        // =====================================================
        // ROI PIXELS
        // =====================================================

        int rx =
                (int) (frameW * roiXPercent);

        int ry =
                (int) (frameH * roiYPercent);

        int rw =
                (int) (frameW * roiWidthPercent);

        int rh =
                (int) (frameH * roiHeightPercent);

        // Защита
        rx =
                Math.max(
                        0,
                        Math.min(
                                rx,
                                frameW - 1
                        )
                );

        ry =
                Math.max(
                        0,
                        Math.min(
                                ry,
                                frameH - 1
                        )
                );

        rw =
                Math.max(
                        1,
                        Math.min(
                                rw,
                                frameW - rx
                        )
                );

        rh =
                Math.max(
                        1,
                        Math.min(
                                rh,
                                frameH - ry
                        )
                );

        Mat roi =
                work.submat(
                        new Rect(
                                rx,
                                ry,
                                rw,
                                rh
                        )
                );

        // =====================================================
        // ROI FRAME
        // =====================================================

        Imgproc.rectangle(
                work,
                new Point(rx, ry),
                new Point(
                        rx + rw,
                        ry + rh
                ),
                new Scalar(255, 255, 0),
                1
        );

        String detected;

        // =====================================================
        // MARKER MODE
        // =====================================================

        if (MODE_MARKER.equals(visionMode)) {

            double[] offsetOut = {0};

            detected =
                    markerTemplate.detect(
                            roi,
                            offsetOut
                    );

            if (!"Marker_None".equals(detected)) {

                visionFound = true;

                /*
                 * offsetOut считается
                 * относительно центра ROI.
                 */

                double roiCenterX =
                        rx + rw / 2.0;

                double frameCenterX =
                        frameW / 2.0;

                visionOffsetX =
                        offsetOut[0]
                                + roiCenterX
                                - frameCenterX;
            }
        }

        // =====================================================
        // COLOR MODE
        // =====================================================

        else {

            detected =
                    detectColor(roi);

            if (!"Color_None".equals(detected)) {

                visionFound = true;

                double roiCenterX =
                        rx + rw / 2.0;

                double frameCenterX =
                        frameW / 2.0;

                visionOffsetX +=
                        roiCenterX
                                - frameCenterX;
            }
        }

        roi.release();

        outputStream.putFrame(work);

        work.release();

        return detected;
    }

    // =========================================================
    // COLOR DETECTION
    // =========================================================

    private String detectColor(Mat work) {

        Mat hsv = new Mat();

        Imgproc.cvtColor(
                work,
                hsv,
                Imgproc.COLOR_BGR2HSV
        );

        Imgproc.GaussianBlur(
                hsv,
                hsv,
                new Size(5, 5),
                0
        );

        String result = "Color_None";

        for (ColorObject obj :
                ColorObject.OBJECTS) {

            if (detectColorObject(
                    hsv,
                    obj,
                    work)) {

                result = obj.name;

                break;
            }
        }

        hsv.release();

        return result;
    }

    // =========================================================
    // COLOR OBJECT
    // =========================================================

    private boolean detectColorObject(
            Mat hsv,
            ColorObject obj,
            Mat output) {

        Mat mask = new Mat();

        // =====================================================
        // PRIMARY RANGE
        // =====================================================

        if (obj.hasSecondRange) {

            Mat m1 = new Mat();
            Mat m2 = new Mat();

            Core.inRange(
                    hsv,
                    new Scalar(
                            obj.hMin,
                            obj.sMin,
                            obj.vMin
                    ),
                    new Scalar(
                            obj.hMax,
                            obj.sMax,
                            obj.vMax
                    ),
                    m1
            );

            Core.inRange(
                    hsv,
                    new Scalar(
                            obj.h2Min,
                            obj.sMin,
                            obj.vMin
                    ),
                    new Scalar(
                            obj.h2Max,
                            obj.sMax,
                            obj.vMax
                    ),
                    m2
            );

            Core.addWeighted(
                    m1,
                    1.0,
                    m2,
                    1.0,
                    0.0,
                    mask
            );

            m1.release();
            m2.release();

        } else {

            Core.inRange(
                    hsv,
                    new Scalar(
                            obj.hMin,
                            obj.sMin,
                            obj.vMin
                    ),
                    new Scalar(
                            obj.hMax,
                            obj.sMax,
                            obj.vMax
                    ),
                    mask
            );
        }

        // =====================================================
        // EXTRA HSV RANGES
        // =====================================================

        for (ColorObject.HsvRange r :
                obj.extraRanges) {

            Mat extra = new Mat();

            Core.inRange(
                    hsv,
                    new Scalar(
                            r.hMin,
                            r.sMin,
                            r.vMin
                    ),
                    new Scalar(
                            r.hMax,
                            r.sMax,
                            r.vMax
                    ),
                    extra
            );

            Core.addWeighted(
                    mask,
                    1.0,
                    extra,
                    1.0,
                    0.0,
                    mask
            );

            extra.release();
        }

        // =====================================================
        // MORPHOLOGY
        // =====================================================

        Mat kernel =
                Imgproc.getStructuringElement(
                        Imgproc.MORPH_ELLIPSE,
                        new Size(5, 5)
                );

        Imgproc.erode(
                mask,
                mask,
                kernel
        );

        Imgproc.dilate(
                mask,
                mask,
                kernel
        );

        kernel.release();

        // =====================================================
        // CONTOURS
        // =====================================================

        List<MatOfPoint> contours =
                new ArrayList<>();

        Imgproc.findContours(
                mask,
                contours,
                new Mat(),
                Imgproc.RETR_EXTERNAL,
                Imgproc.CHAIN_APPROX_SIMPLE
        );

        mask.release();

        if (contours.isEmpty()) {
            return false;
        }

        int cx =
                hsv.width() / 2;

        Rect best = null;

        double minDist =
                Double.MAX_VALUE;

        // =====================================================
        // SEARCH BEST OBJECT
        // =====================================================

        for (MatOfPoint c : contours) {

            double area =
                    Imgproc.contourArea(c);

            if (area < obj.minArea) {
                c.release();
                continue;
            }

            Rect rect =
                    Imgproc.boundingRect(c);

            double centerX =
                    rect.x
                            + rect.width / 2.0;

            double dist =
                    Math.abs(
                            centerX - cx
                    );

            if (dist < minDist) {

                minDist = dist;
                best = rect;
            }

            c.release();
        }

        if (best == null) {
            return false;
        }

        // =====================================================
        // RESULT
        // =====================================================

        visionObjectWidth =
                best.width;

        visionOffsetX =
                best.x
                        + best.width / 2.0
                        - cx;

        Scalar drawColor =
                nameToColor(obj.name);

        Imgproc.rectangle(
                output,
                best.tl(),
                best.br(),
                drawColor,
                3
        );

        Imgproc.putText(
                output,
                obj.name,
                new Point(
                        best.x,
                        Math.max(
                                best.y - 8,
                                16
                        )
                ),
                0,
                0.6,
                drawColor,
                2
        );

        return true;
    }

    // =========================================================
    // COLOR NAME
    // =========================================================

    private Scalar nameToColor(
            String name) {

        String n =
                name.toLowerCase();

        if (n.contains("red")) {
            return new Scalar(
                    0,
                    0,
                    255
            );
        }

        if (n.contains("green")) {
            return new Scalar(
                    0,
                    255,
                    0
            );
        }

        if (n.contains("blue")) {
            return new Scalar(
                    255,
                    0,
                    0
            );
        }

        if (n.contains("yellow")) {
            return new Scalar(
                    0,
                    255,
                    255
            );
        }

        if (n.contains("orange")) {
            return new Scalar(
                    0,
                    165,
                    255
            );
        }

        if (n.contains("purple")) {
            return new Scalar(
                    255,
                    0,
                    255
            );
        }

        return new Scalar(
                255,
                255,
                255
        );
    }
}
