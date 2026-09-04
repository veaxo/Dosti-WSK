package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.NetworkTableEntry;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.first.cameraserver.CameraServer;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

import java.util.*;

public class VisionSubsystem extends SubsystemBase {

    public static final String MODE_MARKER = "MARKER";
    public static final String MODE_COLOR  = "COLOR";

    private String visionMode = MODE_COLOR;

    private UsbCamera camera;
    private CvSink    cvSink;
    private CvSource  outputStream;

    private boolean processed        = false;
    private String  currentDetection = "None";
    private String  savedVisionResult= "None";

    private double  visionOffsetX    = 0;
    private double  visionObjectWidth = 0;
    private boolean visionFound      = false;

    // ROI (Region of Interest) — область детекции в долях от размера кадра
    private double roiXPercent      = 0.0;
    private double roiYPercent      = 0.2;
    private double roiWidthPercent  = 1.0;
    private double roiHeightPercent = 0.6;

    // Шаблоны маркеров
    private final MarkerTemplate markerTemplate = new MarkerTemplate();
    public double getVisionObjectWidth() { return visionObjectWidth; }

    // Shuffleboard
    private final ShuffleboardTab        tab         = Shuffleboard.getTab("Vision");
    private final NetworkTableEntry      sbOffsetX   = tab.add("Offset X",          0).getEntry();
    private final NetworkTableEntry      sbFound     = tab.add("Found",          false).getEntry();
    private final NetworkTableEntry      sbDetected  = tab.add("Detected",       "None").getEntry();
    private final NetworkTableEntry      sbMode      = tab.add("Vision Mode",    "COLOR").getEntry();
    private final NetworkTableEntry      sbProcessed = tab.add("Processed Enabled", false).getEntry();
    private final NetworkTableEntry      sbTemplates = tab.add("Marker Templates",   0).getEntry();
    private final NetworkTableEntry      sbRoiY      = tab.add("ROI Y%",           0.2).getEntry();
    private final NetworkTableEntry      sbRoiH      = tab.add("ROI H%",           0.6).getEntry();

    private final Map<String, NetworkTableEntry> savedVars           = new HashMap<>();
    private final Map<String, NetworkTableEntry> customDoubleEntries = new HashMap<>();
    private final Map<String, NetworkTableEntry> customStringEntries = new HashMap<>();

    public VisionSubsystem() {
        camera = CameraServer.getInstance().startAutomaticCapture();
        camera.setResolution(320, 240);
        camera.setFPS(20);

        cvSink       = CameraServer.getInstance().getVideo();
        outputStream = CameraServer.getInstance().putVideo("Vision", 320, 240);

        markerTemplate.loadTemplates();
        sbTemplates.setDouble(markerTemplate.getTemplateCount());
    }

    // ─── Настройка режима и сканирования ────────────────────────────────────

    public void setMode(String mode)           { this.visionMode = mode; }
    public void setProcessed(boolean enable)   { this.processed  = enable; }
    public void enableScanning(boolean enable) { this.processed  = enable; }

    public void reloadMarkerTemplates() {
        markerTemplate.loadTemplates();
        sbTemplates.setDouble(markerTemplate.getTemplateCount());
    }

    // ─── Настройка ROI ───────────────────────────────────────────────────────

    /**
     * Задаёт область детекции в долях от размера кадра (0.0 – 1.0).
     *
     * @param xPct      левый край области (0.0 = левый край кадра)
     * @param yPct      верхний край области (0.0 = верх кадра)
     * @param widthPct  ширина области (1.0 = весь кадр)
     * @param heightPct высота области (1.0 = весь кадр)
     *
     * Примеры:
     *   setROI(0.0, 0.2, 1.0, 0.6)  — средняя горизонтальная полоса (20%–80%)
     *   setROI(0.0, 0.5, 1.0, 0.5)  — только нижняя половина
     *   setROI(0.5, 0.0, 0.5, 1.0)  — только правая половина
     *   setROI(0.0, 0.0, 1.0, 1.0)  — весь кадр (по умолчанию выкл. ROI)
     */
    public void setROI(double xPct, double yPct, double widthPct, double heightPct) {
        roiXPercent      = xPct;
        roiYPercent      = yPct;
        roiWidthPercent  = widthPct;
        roiHeightPercent = heightPct;
    }

    /** Сбрасывает ROI на весь кадр. */
    public void resetROI() {
        roiXPercent      = 0.0;
        roiYPercent      = 0.0;
        roiWidthPercent  = 1.0;
        roiHeightPercent = 1.0;
    }

    // ─── Геттеры ─────────────────────────────────────────────────────────────

    public String  getCurrentDetection()  { return currentDetection;  }
    public String  getSavedVisionResult() { return savedVisionResult; }
    public double  getVisionOffsetX()     { return visionOffsetX;     }
    public boolean isVisionFound()        { return visionFound;        }

    // ─── Shuffleboard хелперы ────────────────────────────────────────────────

    public void setText(String name, String value) {
        if (!customStringEntries.containsKey(name))
            customStringEntries.put(name, tab.add(name, value != null ? value : "None").getEntry());
        customStringEntries.get(name).setString(value != null ? value : "None");
    }

    public void setValue(String name, double value) {
        if (!customDoubleEntries.containsKey(name))
            customDoubleEntries.put(name, tab.add(name, value).getEntry());
        customDoubleEntries.get(name).setDouble(value);
    }

    public void saveVariableOnce(String name, String value) {
        if (!savedVars.containsKey(name)) {
            NetworkTableEntry entry = tab.add(name, value != null ? value : "None").getEntry();
            entry.setString(value != null ? value : "None");
            savedVars.put(name, entry);
        }
    }

    // ─── Сброс сканирования ──────────────────────────────────────────────────

    public void resetCameraScan() {
        savedVisionResult = "None";
        currentDetection  = "None";
        processed         = false;
        visionFound       = false;
        visionOffsetX     = 0;
    }

    // ─── Periodic ────────────────────────────────────────────────────────────

    @Override
    public void periodic() {
        sbProcessed.setBoolean(processed);
        sbMode.setString(visionMode);
        sbRoiY.setDouble(roiYPercent);
        sbRoiH.setDouble(roiHeightPercent);

        if (!processed) return;

        String result = detectObject();
        currentDetection = result;
        visionFound      = !result.contains("None");

        sbDetected.setString(result);
        sbFound.setBoolean(visionFound);
        sbOffsetX.setDouble(visionOffsetX);

        if (visionFound) {
            savedVisionResult = result;
        }
    }

    // ─── Захват кадра и детекция с ROI ───────────────────────────────────────

    private String detectObject() {
        Mat work = new Mat();
        if (cvSink.grabFrame(work) == 0) {
            work.release();
            return "None";
        }

        int frameW = work.cols();
        int frameH = work.rows();

        // Вычисляем пиксельные координаты ROI
        int rx = (int)(frameW * roiXPercent);
        int ry = (int)(frameH * roiYPercent);
        int rw = (int)(frameW * roiWidthPercent);
        int rh = (int)(frameH * roiHeightPercent);

        // Защита от выхода за пределы кадра
        rx = Math.max(0, Math.min(rx, frameW - 1));
        ry = Math.max(0, Math.min(ry, frameH - 1));
        rw = Math.max(1, Math.min(rw, frameW - rx));
        rh = Math.max(1, Math.min(rh, frameH - ry));

        // Вырезаем область интереса (без копирования данных)
        Mat roi = work.submat(new Rect(rx, ry, rw, rh));

        // Рисуем зелёную рамку ROI на итоговом кадре
        Imgproc.rectangle(work,
            new Point(rx, ry),
            new Point(rx + rw, ry + rh),
            new Scalar(0, 255, 0), 1);

        String detected;

        if (visionMode.equals(MODE_MARKER)) {
            double[] offsetOut = {0};
            detected = markerTemplate.detect(roi, offsetOut);
            // Пересчитываем смещение относительно центра полного кадра
            visionOffsetX = offsetOut[0] + (rx + rw / 2.0) - (frameW / 2.0);

        } else {
            // detectColor считает offsetX относительно центра roi,
            // после чего корректируем до центра полного кадра
            detected = detectColor(roi);
            if (visionFound) {
                double roiCenterX  = rx + rw / 2.0;
                double frameCenterX = frameW / 2.0;
                visionOffsetX += roiCenterX - frameCenterX;
            }
        }

        roi.release();
        outputStream.putFrame(work);
        work.release();
        return detected;
    }

    // ─── Цветовая детекция ───────────────────────────────────────────────────

    private String detectColor(Mat work) {
        Mat hsv = new Mat();
        Imgproc.cvtColor(work, hsv, Imgproc.COLOR_BGR2HSV);
        Imgproc.GaussianBlur(hsv, hsv, new Size(5, 5), 0);

        String result = "Color_None";

        for (ColorObject obj : ColorObject.OBJECTS) {
            if (detectColorObject(hsv, obj, work)) {
                result = obj.name;
                break;
            }
        }

        hsv.release();
        return result;
    }

    private boolean detectColorObject(Mat hsv, ColorObject obj, Mat output) {
        Mat mask = new Mat();

        if (obj.hasSecondRange) {
            Mat m1 = new Mat(), m2 = new Mat();
            Core.inRange(hsv,
                new Scalar(obj.hMin, obj.sMin, obj.vMin),
                new Scalar(obj.hMax, obj.sMax, obj.vMax), m1);
            Core.inRange(hsv,
                new Scalar(obj.h2Min, obj.sMin, obj.vMin),
                new Scalar(obj.h2Max, obj.sMax, obj.vMax), m2);
            Core.addWeighted(m1, 1.0, m2, 1.0, 0.0, mask);
            m1.release(); m2.release();
        } else {
            Core.inRange(hsv,
                new Scalar(obj.hMin, obj.sMin, obj.vMin),
                new Scalar(obj.hMax, obj.sMax, obj.vMax), mask);
        }

        // Доп. диапазоны освещения
        for (ColorObject.HsvRange r : obj.extraRanges) {
            Mat extra = new Mat();
            Core.inRange(hsv,
                new Scalar(r.hMin, r.sMin, r.vMin),
                new Scalar(r.hMax, r.sMax, r.vMax), extra);
            Core.addWeighted(mask, 1.0, extra, 1.0, 0.0, mask);
            extra.release();
        }

        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(5, 5));
        Imgproc.erode (mask, mask, kernel);
        Imgproc.dilate(mask, mask, kernel);
        kernel.release();

        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(mask, contours, new Mat(),
            Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        mask.release();

        if (contours.isEmpty()) return false;

        int cx    = hsv.width() / 2;
        Rect best = null;
        double minDist = Double.MAX_VALUE;

        for (MatOfPoint c : contours) {
            if (Imgproc.contourArea(c) < obj.minArea) continue;

            Rect rect   = Imgproc.boundingRect(c);
            double dist = Math.abs((rect.x + rect.width / 2.0) - cx);
            if (dist < minDist) { minDist = dist; best = rect; }
        }

        if (best == null) return false;

        visionObjectWidth = best.width;
        visionOffsetX     = best.x + best.width / 2.0 - cx;
        visionFound       = true;

        Scalar drawColor = nameToColor(obj.name);
        Imgproc.rectangle(output, best.tl(), best.br(), drawColor, 3);
        Imgproc.putText(output, obj.name,
            new Point(best.x, best.y - 8),
            0, 0.6, drawColor, 2);

        return true;
    }

    private Scalar nameToColor(String name) {
        String n = name.toLowerCase();
        if (n.contains("red"))    return new Scalar(0,   0,   255);
        if (n.contains("green"))  return new Scalar(0,   255, 0);
        if (n.contains("blue"))   return new Scalar(255, 0,   0);
        if (n.contains("yellow")) return new Scalar(0,   255, 255);
        if (n.contains("orange")) return new Scalar(0,   165, 255);
        if (n.contains("purple")) return new Scalar(255, 0,   255);
        return new Scalar(255, 255, 255);
    }
}