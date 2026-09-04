package frc.robot.subsystems;

import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

public class MarkerTemplate {

    private static final String   MARKERS_DIR  = "/home/lvuser/deploy/markers/";
    private static final String[] MARKER_FILES = {"Marker_1.png", "Marker_2.png", "Marker_3.png"};

    private static final int    GRID            = 10;
    private static final double PADDING         = 0.12;
    private static final double MATCH_THRESHOLD = 0.55;
    private static final double MIN_AREA        = 400;
    private static final double MIN_ASPECT      = 0.5;
    private static final double MAX_ASPECT      = 2.0;

    private static class Template {
        final String  name;
        final boolean[] fp;

        Template(String name, boolean[] fp) {
            this.name = name;
            this.fp   = fp;
        }
    }

    private final List<Template> templates = new ArrayList<>();

    public MarkerTemplate() {}

    public void loadTemplates() {
        templates.clear();

        for (String fileName : MARKER_FILES) {
            String path = MARKERS_DIR + fileName;
            if (!new File(path).exists()) continue;

            Mat gray = Imgcodecs.imread(path, Imgcodecs.IMREAD_GRAYSCALE);
            if (gray.empty()) { gray.release(); continue; }

            Mat binary = new Mat();
            Imgproc.threshold(gray, binary, 0, 255, Imgproc.THRESH_BINARY + Imgproc.THRESH_OTSU);
            gray.release();

            boolean[] fp = extractFingerprint(binary);
            binary.release();

            if (fp == null) continue;

            templates.add(new Template(fileName.replace(".png", ""), fp));
        }
    }

    public int getTemplateCount() { return templates.size(); }

    public String detect(Mat work, double[] offsetXOut) {
        offsetXOut[0] = 0;
        if (templates.isEmpty()) return "Marker_None";

        Mat gray   = new Mat();
        Mat binary = new Mat();
        Imgproc.cvtColor(work, gray, Imgproc.COLOR_BGR2GRAY);
        Imgproc.GaussianBlur(gray, gray, new Size(5, 5), 0);
        Imgproc.threshold(gray, binary, 0, 255, Imgproc.THRESH_BINARY + Imgproc.THRESH_OTSU);
        gray.release();

        Mat kernel   = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
        Mat combined = new Mat();
        binary.copyTo(combined);
        Imgproc.morphologyEx(combined, combined, Imgproc.MORPH_CLOSE, kernel);
        kernel.release();

        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(combined, contours, new Mat(),
                Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        combined.release();

        String bestName  = "Marker_None";
        double bestScore = 0;
        Rect   bestRect  = null;

        for (MatOfPoint c : contours) {
            if (Imgproc.contourArea(c) < MIN_AREA) continue;

            Rect r = Imgproc.boundingRect(c);
            double aspect = (double) r.width / r.height;
            if (aspect < MIN_ASPECT || aspect > MAX_ASPECT) continue;

            Imgproc.rectangle(work, r.tl(), r.br(), new Scalar(0, 0, 255), 1);

            Mat roi = new Mat(binary, r);
            boolean[] fp = extractFingerprint(roi);
            roi.release();

            if (fp == null) continue;

            for (Template t : templates) {
                double score = computeScore(fp, t.fp);
                if (score >= MATCH_THRESHOLD && score > bestScore) {
                    bestScore = score;
                    bestName  = t.name;
                    bestRect  = r;
                }
            }
        }

        binary.release();

        if (bestRect != null) {
            offsetXOut[0] = (bestRect.x + bestRect.width / 2.0) - (work.width() / 2.0);
            Imgproc.rectangle(work, bestRect.tl(), bestRect.br(), new Scalar(0, 255, 0), 3);
            Imgproc.putText(work,
                    bestName + " " + String.format("%.0f%%", bestScore * 100),
                    new Point(bestRect.x, Math.max(bestRect.y - 8, 16)),
                    0, 0.7, new Scalar(0, 255, 0), 2);
        }

        return bestName;
    }

    private boolean[] extractFingerprint(Mat binary) {
        if (binary.empty() || binary.rows() < 20 || binary.cols() < 20) return null;

        int h = binary.rows();
        int w = binary.cols();

        boolean[] fp  = new boolean[GRID * GRID * 2];
        int       idx = 0;

        int[][] informative = {{0, 0}, {1, 1}};

        for (int[] rc : informative) {
            int r   = rc[0], col = rc[1];
            int qy1 = r   * h / 2, qy2 = (r + 1) * h / 2;
            int qx1 = col * w / 2, qx2 = (col + 1) * w / 2;

            int ph = (int)((qy2 - qy1) * PADDING);
            int pw = (int)((qx2 - qx1) * PADDING);

            int cx1 = Math.max(0, qx1 + pw), cy1 = Math.max(0, qy1 + ph);
            int cx2 = Math.min(w, qx2 - pw), cy2 = Math.min(h, qy2 - ph);

            if (cx2 - cx1 < 4 || cy2 - cy1 < 4) return null;

            Mat inner   = new Mat(binary, new Rect(cx1, cy1, cx2 - cx1, cy2 - cy1));
            Mat resized = new Mat();
            Imgproc.resize(inner, resized, new Size(GRID, GRID), 0, 0, Imgproc.INTER_AREA);
            inner.release();

            for (int i = 0; i < GRID; i++)
                for (int j = 0; j < GRID; j++)
                    fp[idx++] = resized.get(i, j)[0] > 127;

            resized.release();
        }

        return fp;
    }

    private double computeScore(boolean[] a, boolean[] b) {
        if (a.length != b.length) return 0;
        int matches = 0;
        for (int i = 0; i < a.length; i++)
            if (a[i] == b[i]) matches++;
        return (double) matches / a.length;
    }
}