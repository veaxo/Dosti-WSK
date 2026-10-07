package frc.robot.util;

/** Stops pushing when a fixed camera sees no size growth, or an approach exceeds its budget. */
public final class PickupApproachGuard {
  private double drivenSeconds;
  private double lastTime;
  private double bestWidth;
  private double progressAt;
  private double limit;
  private String failure;

  public void reset(double now, double width, double maximumDriveSeconds) {
    drivenSeconds = 0.0;
    lastTime = now;
    bestWidth = width;
    progressAt = 0.0;
    limit = maximumDriveSeconds;
    failure = null;
  }

  public boolean update(double now, double width, boolean newFrame, double forwardPower) {
    if (failure != null) return false;
    if (!Double.isFinite(now) || !Double.isFinite(width) || !Double.isFinite(forwardPower)) {
      failure = "INVALID_APPROACH_MEASUREMENT";
      return false;
    }
    double dt = Math.max(0.0, Math.min(0.10, now - lastTime));
    lastTime = now;
    if (forwardPower > 0.005) drivenSeconds += dt;
    if (newFrame && width >= bestWidth + 3.0) {
      bestWidth = width;
      progressAt = drivenSeconds;
    }
    if (drivenSeconds >= limit) failure = "APPROACH_TIME_LIMIT";
    else if (drivenSeconds - progressAt >= 2.0) failure = "NO_VISUAL_APPROACH_PROGRESS";
    return failure == null;
  }

  /** Restart size comparisons after a stopped camera move, preserving drive time. */
  public void rebaseView(double now, double width) {
    lastTime = now;
    bestWidth = width;
    progressAt = drivenSeconds;
  }

  public String getFailure() { return failure; }
}
