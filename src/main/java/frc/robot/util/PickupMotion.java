package frc.robot.util;

/** Smooth chassis commands with a quiet steering zone and braking before reversal. */
public final class PickupMotion {
  public static final double CENTER_ACCEPTANCE_PX = 18.0;
  private double forward;
  private double lateral;
  private double lastTime = Double.NaN;
  private boolean correcting;

  public void reset(double now) {
    forward = lateral = 0.0;
    lastTime = now;
    correcting = false;
  }

  /** Use the current frame when smoothing would hide a larger lateral error. */
  public static double controlOffset(double filtered, double current) {
    if (!Double.isFinite(filtered) || !Double.isFinite(current)) return Double.NaN;
    return Math.abs(current) > Math.abs(filtered) || filtered * current < 0.0
        ? current : filtered;
  }

  /** Cancel forward travel immediately when the current image is misaligned. */
  public void stopForward() { forward = 0.0; }

  public double steering(double offset, double gain, double maximum) {
    if (!Double.isFinite(offset)) return 0.0;
    if (Math.abs(offset) <= 10.0) correcting = false;
    else if (Math.abs(offset) >= CENTER_ACCEPTANCE_PX) correcting = true;
    if (!correcting) return 0.0;
    // Subtract the quiet zone so crossing its boundary never causes a jump.
    return Math.copySign(Math.min(maximum,
        Math.max(0.0, Math.abs(offset) - 10.0) * gain), offset);
  }

  public double[] update(double desiredForward, double desiredLateral, double now) {
    if (!Double.isFinite(desiredForward) || !Double.isFinite(desiredLateral)
        || !Double.isFinite(now)) {
      reset(now);
      return new double[] {0.0, 0.0};
    }
    double dt = Double.isFinite(lastTime) ? Math.max(0.0, Math.min(0.04, now - lastTime)) : 0.02;
    lastTime = now;
    forward = approach(forward, desiredForward, dt);
    lateral = approach(lateral, desiredLateral, dt);
    return new double[] {forward, lateral};
  }

  private double approach(double current, double target, double dt) {
    // Brake to zero first when the requested correction changes direction.
    if (current * target < 0.0) target = 0.0;
    double rate = Math.abs(target) < Math.abs(current) ? 0.35 : 0.18;
    double step = rate * dt;
    return current + Math.max(-step, Math.min(step, target - current));
  }

  /** Preserve the working pickup's slow drive floor until position is confirmed. */
  public static double approachSpeed(double maximum, double minimum, double width,
      double desiredWidth, double offset) {
    if (!Double.isFinite(width) || !Double.isFinite(offset)) return 0.0;
    if (Math.abs(offset) >= 28.0) return 0.0;
    // Do not throttle a distant target already accepted as centered. Slow
    // forward travel only outside that zone, reaching zero at 28 pixels.
    double alignment = Math.max(0.0, 1.0
        - Math.max(0.0, Math.abs(offset) - CENTER_ACCEPTANCE_PX)
        / (28.0 - CENTER_ACCEPTANCE_PX));
    double proximity = Math.max(0.0, Math.min(1.0, (desiredWidth - width) / 70.0));
    double slow = Math.max(0.0, Math.min(maximum, minimum));
    return (slow + (maximum - slow) * proximity) * alignment;
  }
}
