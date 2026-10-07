package frc.robot.util;

/** Confirms a visual pickup position using distinct, consecutive camera frames. */
public final class VisionPickupGate {
  private final int minimumFrames;
  private final double holdSeconds;
  private double lastFrame;
  private double firstReadyAt = Double.NaN;
  private int readyFrames;

  public VisionPickupGate(int minimumFrames, double holdSeconds) {
    this.minimumFrames = minimumFrames;
    this.holdSeconds = holdSeconds;
  }

  /** Ignore images taken before the new camera pose was ready. */
  public void reset(double currentFrame) {
    lastFrame = currentFrame;
    clearConfirmation();
  }

  public boolean update(boolean ready, double frame, double now) {
    if (!ready || !Double.isFinite(frame) || frame <= 0 || !Double.isFinite(now)) {
      clearConfirmation();
      if (Double.isFinite(frame)) lastFrame = Math.max(lastFrame, frame);
      return false;
    }
    if (frame <= lastFrame) return false;
    lastFrame = frame;
    if (Double.isNaN(firstReadyAt)) firstReadyAt = now;
    readyFrames++;
    return readyFrames >= minimumFrames && now - firstReadyAt >= holdSeconds;
  }

  private void clearConfirmation() {
    firstReadyAt = Double.NaN;
    readyFrames = 0;
  }

  public static boolean inZone(double offset, double centerY, double bottomY,
      double width, double maximumOffset, double minimumCenterY,
      double minimumBottomY, double minimumWidth) {
    return Double.isFinite(offset) && Double.isFinite(centerY)
        && Double.isFinite(bottomY) && Double.isFinite(width)
        && Math.abs(offset) <= maximumOffset
        && centerY >= minimumCenterY && bottomY >= minimumBottomY
        && bottomY >= centerY && width >= minimumWidth;
  }
}
