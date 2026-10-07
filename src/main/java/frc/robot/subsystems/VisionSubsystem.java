package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.nio.charset.StandardCharsets;
import java.nio.file.AtomicMoveNotSupportedException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.StandardCopyOption;
import java.nio.file.StandardOpenOption;
import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;

import org.json.JSONArray;
import org.json.JSONObject;

/**
 * Vision interface for VMX-pi.
 *
 * One persistent Python process is the only owner of /dev/video0. Java never
 * opens or closes the USB camera. Mode, ROI and processing settings are sent
 * through a control JSON file; detections return through vision_data.json.
 */
public class VisionSubsystem extends SubsystemBase {

    public static final String MODE_MARKER = "MARKER";
    public static final String MODE_COLOR = "COLOR";

    private static final String JSON_PATH = "/home/lvuser/vision_data.json";
    private static final String CONTROL_PATH = "/home/lvuser/vision_control.json";
    private static final String VISION_SCRIPT_NAME = "vision_aruco.py";
    private static final String VISION_SCRIPT_ENV = "VISION_ARUCO_SCRIPT";
    private static final String STREAM_HOST_ENV = "VISION_STREAM_HOST";
    private static final String STREAM_PORT_ENV = "VISION_STREAM_PORT";
    private static final double JSON_FRESHNESS_SEC = 2.0;
    private static final long PROCESS_RETRY_MS = 1500;
    private static final long STARTUP_CAMERA_RELEASE_MS = 750;

    private final BooleanSupplier modeButtonSupplier;
    private boolean previousModeButton;
    private boolean modeButtonEnabled = true;

    private String visionMode = MODE_MARKER;
    private boolean processed;

    private String currentDetection = "None";
    private String savedVisionResult = "None";
    private String visionColorIdentity = "None";
    private double visionOffsetX;
    private double visionObjectWidth;
    private double visionObjectHeight;
    private double visionTargetY = 120.0;
    private double visionTargetBottomY = 120.0;
    private double visionFrameTimestamp;
    private boolean visionFound;
    private boolean colorCameraReady;
    private boolean markerCameraReady;

    private double roiXPercent = 0.0;
    private double roiYPercent = 0.2;
    private double roiWidthPercent = 1.0;
    private double roiHeightPercent = 0.6;

    private Process visionProcess;
    private long nextProcessAttemptMs;

    private final ShuffleboardTab tab = Shuffleboard.getTab("Vision");
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
            tab.add("ROI Y%", roiYPercent).getEntry();
    private final NetworkTableEntry sbRoiH =
            tab.add("ROI H%", roiHeightPercent).getEntry();

    private final Map<String, NetworkTableEntry> savedVars = new HashMap<>();
    private final Map<String, NetworkTableEntry> customDoubleEntries =
            new HashMap<>();
    private final Map<String, NetworkTableEntry> customStringEntries =
            new HashMap<>();

    public VisionSubsystem(BooleanSupplier modeButtonSupplier) {
        this.modeButtonSupplier = modeButtonSupplier;
        publishCameraStream();
        writeVisionControl();
        stopStaleVisionProcess();
        nextProcessAttemptMs = System.currentTimeMillis()
                + STARTUP_CAMERA_RELEASE_MS;
    }

    private void publishCameraStream() {
        String host = System.getenv(STREAM_HOST_ENV);
        if (host == null || host.trim().isEmpty()) {
            host = "raspberrypi.local";
        }
        String port = System.getenv(STREAM_PORT_ENV);
        if (port == null || port.trim().isEmpty()) {
            port = "1186";
        }

        String url = "http://" + host.trim() + ":" + port.trim()
                + "/?action=stream";
        NetworkTableInstance.getDefault()
                .getTable("CameraPublisher")
                .getSubTable("Vision Processed")
                .getEntry("streams")
                .setStringArray(new String[] {"mjpeg:" + url});
        tab.add("Camera Stream URL", url);
    }

    public double getVisionObjectWidth() {
        return visionObjectWidth;
    }

    public double getVisionObjectHeight() {
        return visionObjectHeight;
    }

    public double getVisionTargetY() {
        return visionTargetY;
    }

    public double getVisionTargetBottomY() {
        return visionTargetBottomY;
    }

    public double getVisionOffsetX() {
        return visionOffsetX;
    }

    public boolean isVisionFound() {
        return visionFound;
    }

    public double getVisionFrameTimestamp() {
        return visionFrameTimestamp;
    }

    public boolean isVisionFrameRecent(double maxAgeSeconds) {
        double age = System.currentTimeMillis() / 1000.0
                - visionFrameTimestamp;
        return visionFrameTimestamp > 0.0
                && age >= -0.1
                && age <= maxAgeSeconds;
    }

    public boolean isColorCameraReady() {
        return MODE_COLOR.equals(visionMode) && colorCameraReady;
    }

    public boolean isMarkerCameraReady() {
        return MODE_MARKER.equals(visionMode) && markerCameraReady;
    }

    public String getCurrentDetection() {
        return currentDetection;
    }

    public String getVisionColorIdentity() {
        return visionColorIdentity;
    }

    public String getSavedVisionResult() {
        return savedVisionResult;
    }

    public void setMode(String mode) {
        if (!MODE_MARKER.equals(mode) && !MODE_COLOR.equals(mode)) {
            return;
        }
        if (mode.equals(visionMode)) {
            return;
        }

        visionMode = mode;
        colorCameraReady = false;
        markerCameraReady = false;
        resetDetectionState();
        writeVisionControl();
        setText("Camera Owner", "Python " + mode);
        setText("Camera State", "Waiting for " + mode + " result");
    }

    public void setProcessed(boolean enable) {
        if (processed == enable) {
            return;
        }
        processed = enable;
        writeVisionControl();
    }

    public void enableScanning(boolean enable) {
        setProcessed(enable);
    }

    public void setModeButtonEnabled(boolean enabled) {
        modeButtonEnabled = enabled;
        previousModeButton = modeButtonSupplier != null
                && modeButtonSupplier.getAsBoolean();
    }

    public void reloadMarkerTemplates() {
        sbTemplates.setDouble(0);
    }

    public void setROI(
            double xPct,
            double yPct,
            double widthPct,
            double heightPct) {
        roiXPercent = clamp01(xPct);
        roiYPercent = clamp01(yPct);
        roiWidthPercent = clamp01(widthPct);
        roiHeightPercent = clamp01(heightPct);
        writeVisionControl();
    }

    public void resetROI() {
        setROI(0.0, 0.0, 1.0, 1.0);
    }

    public void resetCameraScan() {
        savedVisionResult = "None";
        processed = false;
        colorCameraReady = false;
        markerCameraReady = false;
        resetDetectionState();
        writeVisionControl();
    }

    public void setText(String name, String value) {
        String safeValue = value != null ? value : "None";
        NetworkTableEntry entry = customStringEntries.get(name);
        if (entry == null) {
            entry = tab.add(name, safeValue).getEntry();
            customStringEntries.put(name, entry);
        }
        entry.setString(safeValue);
    }

    public void setValue(String name, double value) {
        NetworkTableEntry entry = customDoubleEntries.get(name);
        if (entry == null) {
            entry = tab.add(name, value).getEntry();
            customDoubleEntries.put(name, entry);
        }
        entry.setDouble(value);
    }

    public void saveVariableOnce(String name, String value) {
        if (savedVars.containsKey(name)) {
            return;
        }
        String safeValue = value != null ? value : "None";
        NetworkTableEntry entry = tab.add(name, safeValue).getEntry();
        entry.setString(safeValue);
        savedVars.put(name, entry);
    }

    @Override
    public void periodic() {
        updateModeFromButton();
        ensureVisionProcessRunning();

        sbProcessed.setBoolean(processed);
        sbMode.setString(visionMode);
        sbRoiY.setDouble(roiYPercent);
        sbRoiH.setDouble(roiHeightPercent);
        readJsonVision();
    }

    private void updateModeFromButton() {
        boolean pressed = modeButtonSupplier != null
                && modeButtonSupplier.getAsBoolean();
        if (!modeButtonEnabled) {
            previousModeButton = pressed;
            return;
        }

        if (pressed && !previousModeButton) {
            setMode(MODE_MARKER.equals(visionMode) ? MODE_COLOR : MODE_MARKER);
            setProcessed(true);
        }
        previousModeButton = pressed;
    }

    private void writeVisionControl() {
        JSONObject control = new JSONObject();
        control.put("mode", visionMode);
        control.put("processed", processed);
        control.put("roi", new JSONArray()
                .put(roiXPercent)
                .put(roiYPercent)
                .put(roiWidthPercent)
                .put(roiHeightPercent));

        Path destination = Paths.get(CONTROL_PATH);
        Path temporary = Paths.get(CONTROL_PATH + ".tmp");
        try {
            Files.createDirectories(destination.getParent());
            Files.write(
                    temporary,
                    control.toString().getBytes(StandardCharsets.UTF_8),
                    StandardOpenOption.CREATE,
                    StandardOpenOption.TRUNCATE_EXISTING,
                    StandardOpenOption.WRITE);
            try {
                Files.move(
                        temporary,
                        destination,
                        StandardCopyOption.REPLACE_EXISTING,
                        StandardCopyOption.ATOMIC_MOVE);
            } catch (AtomicMoveNotSupportedException ignored) {
                Files.move(
                        temporary,
                        destination,
                        StandardCopyOption.REPLACE_EXISTING);
            }
        } catch (Exception e) {
            setText("Control Error", e.getClass().getSimpleName());
        }
    }

    private void readJsonVision() {
        try {
            String text = new String(
                    Files.readAllBytes(Paths.get(JSON_PATH)),
                    StandardCharsets.UTF_8);
            JSONObject root = new JSONObject(text);

            String resultMode = root.optString("mode", MODE_MARKER);
            double timestamp = root.optDouble("timestamp", 0.0);
            double age = Math.abs(System.currentTimeMillis() / 1000.0 - timestamp);
            boolean fresh = timestamp > 0 && age <= JSON_FRESHNESS_SEC
                    && root.optBoolean("cameraOk", true);

            colorCameraReady = fresh
                    && MODE_COLOR.equals(visionMode)
                    && MODE_COLOR.equals(resultMode);
            markerCameraReady = fresh
                    && MODE_MARKER.equals(visionMode)
                    && MODE_MARKER.equals(resultMode);

            if (!fresh || !visionMode.equals(resultMode)) {
                colorCameraReady = false;
                markerCameraReady = false;
                resetDetectionState();
                setText("Camera State", fresh
                        ? "Waiting for " + visionMode + " result"
                        : "Waiting for fresh Python frame");
                return;
            }

            visionFrameTimestamp = timestamp;
            setValue("Vision Processing FPS", root.optDouble("processingFps", 0));
            setValue("Vision Frame Age", age);

            if (MODE_COLOR.equals(resultMode)) {
                readColorResult(root);
                setText("Camera State", "READY");
                setText("Camera Owner", "Python COLOR");
            } else {
                readMarkerResult(root);
                setText("Camera State", "READY");
                setText("Camera Owner", "Python MARKER");
            }
            publishDetection();
        } catch (Exception e) {
            colorCameraReady = false;
            markerCameraReady = false;
            resetDetectionState();
            setText("Camera State", "Vision JSON unavailable");
            setText("Camera Error", e.getClass().getSimpleName());
        }
    }

    private void readColorResult(JSONObject root) {
        setText("Color Tracking State", root.optString("trackingState", "Unknown"));
        setValue("Color Candidate Count", root.optInt("candidateCount", 0));
        setValue("Target ID", -1); // An ArUco ID from an old mode is not a color ID.
        visionFound = root.optBoolean("found", false);
        currentDetection = visionFound
                ? root.optString("name", "Color")
                : "Color_None";
        visionColorIdentity = visionFound
                ? root.optString("colorGroup", currentDetection) : "None";
        double scaleX = 320.0 / Math.max(1.0, root.optDouble("frameWidth", 320.0));
        double scaleY = 240.0 / Math.max(1.0, root.optDouble("frameHeight", 240.0));
        double centerX = root.optDouble("x", 160.0 / scaleX) * scaleX;
        visionOffsetX = visionFound ? centerX - 160.0 : 0.0;
        visionObjectWidth = visionFound
                ? root.optDouble("width", 0.0) * scaleX
                : 0.0;
        visionObjectHeight = visionFound ? root.optDouble("height", 0.0) * scaleY : 0.0;
        visionTargetY = visionFound ? root.optDouble("y", 120.0 / scaleY) * scaleY : 120.0;
        visionTargetBottomY = visionFound
                ? root.optDouble(
                        "bottomY",
                        (visionTargetY + visionObjectHeight / 2.0) / scaleY) * scaleY
                : 120.0;
        if (visionFound && (!Double.isFinite(visionOffsetX)
                || !Double.isFinite(visionObjectWidth) || visionObjectWidth <= 0
                || visionObjectWidth > 320 || !Double.isFinite(visionObjectHeight)
                || visionObjectHeight <= 0 || visionObjectHeight > 240
                || !Double.isFinite(visionTargetY) || visionTargetY < 0 || visionTargetY >= 240
                || !Double.isFinite(visionTargetBottomY) || visionTargetBottomY >= 240
                || visionTargetBottomY < visionTargetY || Math.abs(visionOffsetX) > 160)) {
            resetDetectionState();
            setText("Camera Error", "Invalid color measurement");
        }
        setValue("Target Y", visionTargetY);
        setValue("Target Bottom Y", visionTargetBottomY);
        setValue("Target Count", root.optInt("targetCount", 0));
        JSONArray hsv = root.optJSONArray("hsv");
        setValue("Target H", visionFound && hsv != null ? hsv.optDouble(0, -1) : -1);
        setValue("Target S", visionFound && hsv != null ? hsv.optDouble(1, -1) : -1);
        setValue("Target V", visionFound && hsv != null ? hsv.optDouble(2, -1) : -1);
        setValue("Color Mask Purity", visionFound ? root.optDouble("colorPurity", 0) : 0);
        setValue("Color Profile Coverage", visionFound ? root.optDouble("profileCoverage", 0) : 0);
    }

    private void readMarkerResult(JSONObject root) {
        JSONArray targets = root.optJSONArray("targets");
        int count = targets == null ? 0 : targets.length();
        JSONObject target = count > 0 ? targets.optJSONObject(0) : null;

        visionFound = root.optBoolean("found", false) && target != null;
        visionObjectWidth = 0.0;
        visionObjectHeight = 0.0;
        if (!visionFound) {
            currentDetection = "None";
            visionOffsetX = 0.0;
            setValue("Target Count", count);
            return;
        }

        int id = target.optInt("id", -1);
        double centerX = target.optDouble("x", 160.0);
        currentDetection = id >= 0 ? "Marker_" + id : "Marker";
        visionOffsetX = centerX - 160.0;
        setValue("Target Y", target.optDouble("y", 120.0));
        setValue("Target ID", id);
        setValue("Target Count", count);
    }

    private void publishDetection() {
        sbDetected.setString(currentDetection);
        sbFound.setBoolean(visionFound);
        sbOffsetX.setDouble(visionOffsetX);
        // Live raw width for distance calibration, independent of pickup state.
        setValue("Target Width", visionObjectWidth);
        if (visionFound) {
            savedVisionResult = currentDetection;
        }
    }

    private void resetDetectionState() {
        visionColorIdentity = "None";
        visionFrameTimestamp = 0.0;
        currentDetection = "None";
        visionFound = false;
        visionOffsetX = 0.0;
        visionObjectWidth = 0.0;
        visionObjectHeight = 0.0;
        visionTargetY = 120.0;
        visionTargetBottomY = 120.0;
        sbDetected.setString(currentDetection);
        sbFound.setBoolean(false);
        sbOffsetX.setDouble(0.0);
        setValue("Target Width", 0.0);
    }

    private Path findVisionScript() {
        String configured = System.getenv(VISION_SCRIPT_ENV);
        if (configured != null && !configured.trim().isEmpty()) {
            Path path = Paths.get(configured.trim());
            return Files.isRegularFile(path) ? path : null;
        }

        Path[] candidates = {
            Paths.get("/home/lvuser/deploy/" + VISION_SCRIPT_NAME),
            Paths.get("/home/lvuser/" + VISION_SCRIPT_NAME),
            Paths.get("/home/pi/" + VISION_SCRIPT_NAME)
        };
        for (Path candidate : candidates) {
            if (Files.isRegularFile(candidate)) {
                return candidate;
            }
        }
        return null;
    }

    private void stopStaleVisionProcess() {
        try {
            new ProcessBuilder("pkill", "-9", "-f", VISION_SCRIPT_NAME)
                    .start();
            setText("Vision Process", "Cleaning stale process");
        } catch (Exception e) {
            setText("Vision Process", "Cleanup skipped: "
                    + e.getClass().getSimpleName());
        }
    }

    private void startVisionProcess() {
        if (visionProcess != null && visionProcess.isAlive()) {
            return;
        }

        visionProcess = null;
        nextProcessAttemptMs = System.currentTimeMillis() + PROCESS_RETRY_MS;
        Path script = findVisionScript();
        if (script == null) {
            setText("Vision Process", "Script not found");
            return;
        }

        try {
            ProcessBuilder builder = new ProcessBuilder("python3", script.toString());
            builder.environment().put("PYTHONUNBUFFERED", "1");
            builder.environment().put("VISION_JSON_PATH", JSON_PATH);
            builder.environment().put("VISION_CONTROL_PATH", CONTROL_PATH);
            builder.redirectErrorStream(true);
            builder.redirectOutput(ProcessBuilder.Redirect.appendTo(
                    Paths.get("/home/lvuser/vision_aruco.log").toFile()));
            visionProcess = builder.start();
            setText("Vision Process", "Running");
            setText("Vision Script", script.toString());
            setText("Camera Owner", "Python " + visionMode);
        } catch (Exception e) {
            visionProcess = null;
            setText("Vision Process", "Start failed: "
                    + e.getClass().getSimpleName());
        }
    }

    private void ensureVisionProcessRunning() {
        if (visionProcess != null && visionProcess.isAlive()) {
            return;
        }

        if (visionProcess != null) {
            try {
                setText("Vision Process", "Exited: " + visionProcess.exitValue());
            } catch (IllegalThreadStateException ignored) {
                return;
            }
            visionProcess = null;
        }

        if (System.currentTimeMillis() >= nextProcessAttemptMs) {
            startVisionProcess();
        }
    }

    private double clamp01(double value) {
        return Math.max(0.0, Math.min(1.0, value));
    }
}
