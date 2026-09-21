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
            35, 60, 40,
            85, 255, 255,
            400
        ),

        new ColorObject("Red ball",
             0, 80, 40,
            10, 255, 255,
            160, 180,
            400
        ),

        new ColorObject("Yellow ball",
            18, 80, 70,
            35, 255, 255,
            400
        )
    };
}
