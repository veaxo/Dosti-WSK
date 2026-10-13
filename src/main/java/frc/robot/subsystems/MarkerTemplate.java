package frc.robot.subsystems;

import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

public class MarkerTemplate {

    private static final String MARKERS_DIR = "/home/lvuser/deploy/markers/";

    private static final String[] MARKER_FILES = {
            "Marker_1.png",
            "Marker_2.png",
            "Marker_3.png"
    };

    // Размер нормализованного маркера
    private static final int MARKER_SIZE = 160;

    // Минимальная площадь кандидата
    private static final double MIN_AREA = 500;

    // Максимальная площадь кандидата относительно кадра
    private static final double MAX_FRAME_AREA = 0.80;

    // Допустимое соотношение сторон
    private static final double MIN_ASPECT = 0.55;
    private static final double MAX_ASPECT = 1.80;

    // Насколько соединяем четыре чёрных квадрата маркера
    private static final int CONNECT_SIZE = 9;

    // Минимальное сходство с шаблоном
    private static final double MATCH_THRESHOLD = 0.5;

    private static class Template {

        final String name;
        final Mat image;

        Template(String name, Mat image) {
            this.name = name;
            this.image = image;
        }

        void release() {
            image.release();
        }
    }

    private final List<Template> templates = new ArrayList<>();

    public MarkerTemplate() {
    }

    /**
     * Загружает Marker_1.png, Marker_2.png, Marker_3.png
     */
    public void loadTemplates() {

        for (Template template : templates) {
            template.release();
        }

        templates.clear();

        for (String fileName : MARKER_FILES) {

            String path = MARKERS_DIR + fileName;

            if (!new File(path).exists()) {
                System.out.println(
                        "[MarkerTemplate] File not found: " + path
                );
                continue;
            }

            Mat image = Imgcodecs.imread(
                    path,
                    Imgcodecs.IMREAD_GRAYSCALE
            );

            if (image.empty()) {
                image.release();

                System.out.println(
                        "[MarkerTemplate] Cannot read: " + path
                );

                continue;
            }

            Mat normalized = normalizeTemplate(image);

            image.release();

            if (normalized.empty()) {
                normalized.release();
                continue;
            }

            String name = fileName.replace(".png", "");

            templates.add(
                    new Template(name, normalized)
            );

            System.out.println(
                    "[MarkerTemplate] Loaded: " + name
            );
        }

        System.out.println(
                "[MarkerTemplate] Templates loaded: "
                        + templates.size()
        );
    }

    public int getTemplateCount() {
        return templates.size();
    }

    /**
     * Основная функция распознавания.
     *
     * offsetXOut[0] = смещение центра маркера
     * относительно центра переданного изображения.
     */
    public String detect(Mat frame, double[] offsetXOut) {

        offsetXOut[0] = 0;

        if (frame == null || frame.empty()) {
            return "Marker_None";
        }

        if (templates.isEmpty()) {
            return "Marker_None";
        }

        Mat gray = new Mat();
        Mat blackMask = new Mat();
        Mat connected = new Mat();

        try {

            // -----------------------------------------
            // 1. Переводим изображение в grayscale
            // -----------------------------------------

            Imgproc.cvtColor(
                    frame,
                    gray,
                    Imgproc.COLOR_BGR2GRAY
            );

            Imgproc.GaussianBlur(
                    gray,
                    gray,
                    new Size(5, 5),
                    0
            );

            // -----------------------------------------
            // 2. Получаем чёрные области
            // -----------------------------------------

            Imgproc.threshold(
                    gray,
                    blackMask,
                    0,
                    255,
                    Imgproc.THRESH_BINARY_INV
                            + Imgproc.THRESH_OTSU
            );

            // -----------------------------------------
            // 3. Соединяем четыре части маркера
            // -----------------------------------------

            Mat kernel = Imgproc.getStructuringElement(
                    Imgproc.MORPH_RECT,
                    new Size(CONNECT_SIZE, CONNECT_SIZE)
            );

            Imgproc.morphologyEx(
                    blackMask,
                    connected,
                    Imgproc.MORPH_CLOSE,
                    kernel
            );

            Imgproc.dilate(
                    connected,
                    connected,
                    kernel
            );

            kernel.release();

            // -----------------------------------------
            // 4. Ищем внешние контуры
            // -----------------------------------------

            List<MatOfPoint> contours = new ArrayList<>();

            Imgproc.findContours(
                    connected,
                    contours,
                    new Mat(),
                    Imgproc.RETR_EXTERNAL,
                    Imgproc.CHAIN_APPROX_SIMPLE
            );

            double bestScore = 0;
            String bestName = "Marker_None";

            MatOfPoint2f bestCorners = null;

            // -----------------------------------------
            // 5. Проверяем каждый кандидат
            // -----------------------------------------

            for (MatOfPoint contour : contours) {

                double area = Imgproc.contourArea(contour);

                if (area < MIN_AREA) {
                    contour.release();
                    continue;
                }

                double frameArea =
                        frame.cols() * frame.rows();

                if (area > frameArea * MAX_FRAME_AREA) {
                    contour.release();
                    continue;
                }

                Rect rect =
                        Imgproc.boundingRect(contour);

                double aspect =
                        (double) rect.width / rect.height;

                if (aspect < MIN_ASPECT
                        || aspect > MAX_ASPECT) {

                    contour.release();
                    continue;
                }

                // -------------------------------------
                // 6. Пытаемся получить 4 угла
                // -------------------------------------

                MatOfPoint2f contour2f =
                        new MatOfPoint2f(contour.toArray());

                double perimeter =
                        Imgproc.arcLength(
                                contour2f,
                                true
                        );

                MatOfPoint2f approx =
                        new MatOfPoint2f();

                Imgproc.approxPolyDP(
                        contour2f,
                        approx,
                        perimeter * 0.04,
                        true
                );

                contour2f.release();

                if (approx.total() != 4) {
                    approx.release();
                    contour.release();
                    continue;
                }

                // -------------------------------------
                // 7. Упорядочиваем углы
                // -------------------------------------

                Point[] corners =
                        orderCorners(approx.toArray());

                approx.release();

                if (corners == null) {
                    contour.release();
                    continue;
                }

                // -------------------------------------
                // 8. Выпрямляем перспективу
                // -------------------------------------

                Mat normalized =
                        warpMarker(frame, corners);

                if (normalized.empty()) {
                    normalized.release();
                    contour.release();
                    continue;
                }

                // -------------------------------------
                // 9. Сравниваем с шаблонами
                // -------------------------------------

                for (Template template : templates) {

                    double score =
                            compareMarker(
                                    normalized,
                                    template.image
                            );

                    if (score > bestScore) {

                        bestScore = score;
                        bestName = template.name;

                        if (bestCorners != null) {
                            bestCorners.release();
                        }

                        bestCorners =
                                new MatOfPoint2f(corners);
                    }
                }

                normalized.release();
                contour.release();
            }

            // -----------------------------------------
            // 10. Проверяем результат
            // -----------------------------------------

            if (bestCorners == null
                    || bestScore < MATCH_THRESHOLD) {

                if (bestCorners != null) {
                    bestCorners.release();
                }

                return "Marker_None";
            }

            // -----------------------------------------
            // 11. Получаем центр маркера
            // -----------------------------------------

            Point[] p = bestCorners.toArray();

            double centerX = 0;
            double centerY = 0;

            for (Point point : p) {
                centerX += point.x;
                centerY += point.y;
            }

            centerX /= 4.0;
            centerY /= 4.0;

            offsetXOut[0] =
                    centerX - frame.cols() / 2.0;

            // -----------------------------------------
            // 12. Рисуем результат
            // -----------------------------------------

            for (int i = 0; i < 4; i++) {

                Point a = p[i];
                Point b = p[(i + 1) % 4];

                Imgproc.line(
                        frame,
                        a,
                        b,
                        new Scalar(0, 255, 0),
                        2
                );
            }

            Imgproc.circle(
                    frame,
                    new Point(centerX, centerY),
                    4,
                    new Scalar(0, 0, 255),
                    -1
            );

            Imgproc.putText(
                frame,
                bestName + " " + String.format("%.0f%%", bestScore * 100),
                new Point(
                        Math.max(0, centerX - 60),
                        Math.max(20, centerY - 20)
                ),
                0,
                0.6,
                new Scalar(0, 255, 0),
                2
            );

            bestCorners.release();

            return bestName;

        } finally {

            gray.release();
            blackMask.release();
            connected.release();
        }
    }

    /**
     * Подготавливает шаблон.
     */
    private Mat normalizeTemplate(Mat input) {

        Mat gray = new Mat();

        if (input.channels() == 1) {
            input.copyTo(gray);
        } else {
            Imgproc.cvtColor(
                    input,
                    gray,
                    Imgproc.COLOR_BGR2GRAY
            );
        }

        Imgproc.threshold(
                gray,
                gray,
                0,
                255,
                Imgproc.THRESH_BINARY
                        + Imgproc.THRESH_OTSU
        );

        Mat resized = new Mat();

        Imgproc.resize(
                gray,
                resized,
                new Size(
                        MARKER_SIZE,
                        MARKER_SIZE
                ),
                0,
                0,
                Imgproc.INTER_AREA
        );

        gray.release();

        return resized;
    }

    /**
     * Перспективное выравнивание маркера.
     */
    private Mat warpMarker(
            Mat frame,
            Point[] corners
    ) {

        MatOfPoint2f source =
                new MatOfPoint2f(corners);

        Point[] destinationPoints = {

                new Point(0, 0),

                new Point(
                        MARKER_SIZE - 1,
                        0
                ),

                new Point(
                        MARKER_SIZE - 1,
                        MARKER_SIZE - 1
                ),

                new Point(
                        0,
                        MARKER_SIZE - 1
                )
        };

        MatOfPoint2f destination =
                new MatOfPoint2f(
                        destinationPoints
                );

        Mat transform =
                Imgproc.getPerspectiveTransform(
                        source,
                        destination
                );

        Mat result = new Mat();

        Imgproc.warpPerspective(
                frame,
                result,
                transform,
                new Size(
                        MARKER_SIZE,
                        MARKER_SIZE
                )
        );

        source.release();
        destination.release();
        transform.release();

        Mat gray = new Mat();

        Imgproc.cvtColor(
                result,
                gray,
                Imgproc.COLOR_BGR2GRAY
        );

        result.release();

        Imgproc.threshold(
                gray,
                gray,
                0,
                255,
                Imgproc.THRESH_BINARY
                        + Imgproc.THRESH_OTSU
        );

        return gray;
    }

    /**
     * Сравнение двух нормализованных маркеров.
     */
    private double compareMarker(
            Mat a,
            Mat b
    ) {

        if (a.empty() || b.empty()) {
            return 0;
        }

        Mat resizedA = new Mat();
        Mat resizedB = new Mat();

        Imgproc.resize(
                a,
                resizedA,
                new Size(
                        MARKER_SIZE,
                        MARKER_SIZE
                )
        );

        Imgproc.resize(
                b,
                resizedB,
                new Size(
                        MARKER_SIZE,
                        MARKER_SIZE
                )
        );

        // Сравниваем четыре поворота.
        double best = 0;

        Mat current = resizedA;

        for (int rotation = 0;
             rotation < 4;
             rotation++) {

            double score =
                    binarySimilarity(
                            current,
                            resizedB
                    );

            best = Math.max(best, score);

            if (rotation < 3) {

                Mat rotated = new Mat();

                Core.rotate(
                        current,
                        rotated,
                        Core.ROTATE_90_CLOCKWISE
                );

                if (current != resizedA) {
                    current.release();
                }

                current = rotated;
            }
        }

        if (current != resizedA) {
            current.release();
        }

        resizedA.release();
        resizedB.release();

        return best;
    }

    /**
     * Побитовое сравнение.
     */
    private double binarySimilarity(
            Mat a,
            Mat b
    ) {

        if (a.size().equals(b.size()) == false) {
            return 0;
        }

        Mat diff = new Mat();

        Core.absdiff(
                a,
                b,
                diff
        );

        Mat threshold = new Mat();

        Imgproc.threshold(
                diff,
                threshold,
                30,
                255,
                Imgproc.THRESH_BINARY
        );

        double different =
                Core.countNonZero(threshold);

        double total =
                a.rows() * a.cols();

        diff.release();
        threshold.release();

        return 1.0 - (different / total);
    }

    /**
     * Расставляет углы:
     *
     * 0 = TL
     * 1 = TR
     * 2 = BR
     * 3 = BL
     */
    private Point[] orderCorners(
            Point[] points
    ) {

        if (points == null
                || points.length != 4) {
            return null;
        }

        Point tl = null;
        Point tr = null;
        Point br = null;
        Point bl = null;

        double minSum = Double.MAX_VALUE;
        double maxSum = -Double.MAX_VALUE;

        double minDiff = Double.MAX_VALUE;
        double maxDiff = -Double.MAX_VALUE;

        for (Point p : points) {

            double sum =
                    p.x + p.y;

            double diff =
                    p.x - p.y;

            if (sum < minSum) {
                minSum = sum;
                tl = p;
            }

            if (sum > maxSum) {
                maxSum = sum;
                br = p;
            }

            if (diff > maxDiff) {
                maxDiff = diff;
                tr = p;
            }

            if (diff < minDiff) {
                minDiff = diff;
                bl = p;
            }
        }

        if (tl == null
                || tr == null
                || br == null
                || bl == null) {
            return null;
        }

        return new Point[]{
                tl,
                tr,
                br,
                bl
        };
    }
}