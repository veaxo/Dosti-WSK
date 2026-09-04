package frc.robot.subsystems;

public class ColorObject {
    public final String name;
    public final double hMin, sMin, vMin;
    public final double hMax, sMax, vMax;
    public final double minArea;
    public final boolean hasSecondRange;
    public final double h2Min, h2Max;

    public static class HsvRange {
        public final double hMin, sMin, vMin;
        public final double hMax, sMax, vMax;
        public HsvRange(double hMin, double sMin, double vMin,
                        double hMax, double sMax, double vMax) {
            this.hMin = hMin; this.sMin = sMin; this.vMin = vMin;
            this.hMax = hMax; this.sMax = sMax; this.vMax = vMax;
        }
    }

    public final HsvRange[] extraRanges;

    public ColorObject(String name,
                       double hMin, double sMin, double vMin,
                       double hMax, double sMax, double vMax,
                       double minArea) {
        this.name  = name;
        this.hMin  = hMin; this.sMin = sMin; this.vMin = vMin;
        this.hMax  = hMax; this.sMax = sMax; this.vMax = vMax;
        this.minArea = minArea;
        this.hasSecondRange = false;
        this.h2Min = 0; this.h2Max = 0;
        this.extraRanges = new HsvRange[0];
    }

    public ColorObject(String name,
                       double hMin, double sMin, double vMin,
                       double hMax, double sMax, double vMax,
                       double minArea,
                       HsvRange... extraRanges) {
        this.name  = name;
        this.hMin  = hMin; this.sMin = sMin; this.vMin = vMin;
        this.hMax  = hMax; this.sMax = sMax; this.vMax = vMax;
        this.minArea = minArea;
        this.hasSecondRange = false;
        this.h2Min = 0; this.h2Max = 0;
        this.extraRanges = extraRanges;
    }

    public ColorObject(String name,
                       double hMin, double sMin, double vMin,
                       double hMax, double sMax, double vMax,
                       double h2Min, double h2Max,
                       double minArea) {
        this.name  = name;
        this.hMin  = hMin; this.sMin = sMin; this.vMin = vMin;
        this.hMax  = hMax; this.sMax = sMax; this.vMax = vMax;
        this.minArea = minArea;
        this.hasSecondRange = true;
        this.h2Min = h2Min; this.h2Max = h2Max;
        this.extraRanges = new HsvRange[0];
    }

    public ColorObject(String name,
                       double hMin, double sMin, double vMin,
                       double hMax, double sMax, double vMax,
                       double h2Min, double h2Max,
                       double minArea,
                       HsvRange... extraRanges) {
        this.name  = name;
        this.hMin  = hMin; this.sMin = sMin; this.vMin = vMin;
        this.hMax  = hMax; this.sMax = sMax; this.vMax = vMax;
        this.minArea = minArea;
        this.hasSecondRange = true;
        this.h2Min = h2Min; this.h2Max = h2Max;
        this.extraRanges = extraRanges;
    }

    public static final ColorObject[] OBJECTS = {

        new ColorObject("green ball",
            35,  60,  40,
            85,  255, 255,
            600,
            new HsvRange(35, 40, 25, 85, 180, 160)
        ),

        new ColorObject("Red ball",
             0,  100, 40,
            10,  255, 255,
            160, 180,
            600,
            new HsvRange(0,   70, 25,  10, 220, 160),
            new HsvRange(160, 70, 25, 180, 220, 160)
        ),

        new ColorObject("Yellow ball",
        18, 109, 143,
        30, 230, 238,
        400
        ),

        new ColorObject("Yellow ball",
        18,  80,  100,
        30,  255, 255,
        400,
        new HsvRange(18, 109, 143, 30, 230, 238),  // твой оригинал
        new HsvRange(18,  50,  60, 30, 200, 140),  // темнее
        new HsvRange(15,  60, 180, 33, 255, 255)   // светлее
),

        new ColorObject("Blue ball",
        98, 182, 80,
        112, 255, 216,
        400
        ),

    };
}