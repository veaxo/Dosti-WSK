package frc.robot.util;

/** Image gates in the fixed, calibrated camera poses; all coordinates are 320x240. */
public final class PickupGeometry {
  // First phase prepares a second camera view; it does not authorize grip.
  // Preserve the user's width setting. Y depends on the scan angle, so it
  // cannot be a proximity requirement before changing to the second pose.
  public static final double FIRST_WIDTH = 110.0;
  // Actual image boundary (320x240), independent of approach thresholds.
  public static final double EDGE_BOTTOM_Y = 235.0;

  private PickupGeometry() {}

  public static boolean firstView(double x, double y, double bottom, double width) {
    // A low first-view target may be distant because of the scan angle.
    // Only sufficient horizontal width can prepare the second pose. A
    // visible cropped portion remains trackable, but its bottom is not a
    // distance measurement. Later grip gates still require an unclipped view.
    return VisionPickupGate.inZone(x, y, bottom, width, 21.0, 0.0, 0.0, FIRST_WIDTH)
        && bottom < 240.0 && y < 240.0 && width <= 320.0;
  }

  public static boolean inView(double x, double y, double bottom, double width,
      double minWidth, double minCenterY, double minBottomY) {
    return VisionPickupGate.inZone(x, y, bottom, width, 21.0,
        minCenterY, minBottomY, minWidth)
        && bottom < EDGE_BOTTOM_Y && y < EDGE_BOTTOM_Y && width <= 320.0;
  }
}
