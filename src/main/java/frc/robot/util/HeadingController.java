package frc.robot.util;

import frc.robot.Constants;

/** Bounded PI control of yaw with damping from the measured gyro rate. */
public final class HeadingController {
  private double target = Double.NaN;
  private double lastTime = Double.NaN;
  private double integral;
  private double filteredRate;
  private double error;
  private double correction;
  private boolean holding;
  private boolean settling;
  private double settledSince = Double.NaN;

  public void reset() {
    target = Double.NaN;
    resetFeedback();
    holding = false;
    settling = false;
    settledSince = Double.NaN;
  }

  private void resetFeedback() {
    lastTime = Double.NaN;
    integral = 0.0;
    filteredRate = 0.0;
    error = 0.0;
    correction = 0.0;
  }

  /** Called on every frame with an intentional operator turn command. */
  public void manualTurn() {
    reset();
    settling = true;
  }

  /** Captures a new course at translation start or after a manual turn settles. */
  public double hold(double yaw, double rate, double now) {
    if (!Double.isFinite(yaw) || !Double.isFinite(rate) || !Double.isFinite(now)) {
      reset();
      return 0.0;
    }
    if (!holding) {
      if (Math.abs(rate) > Constants.HEADING_SETTLE_RATE_DEG_PER_SEC) {
        settling = true;
        settledSince = Double.NaN;
        return 0.0;
      }
      if (settling) {
        if (!Double.isFinite(settledSince) || now < settledSince) {
          settledSince = now;
        }
        if (now - settledSince < Constants.HEADING_SETTLE_TIME_SEC) {
          return 0.0;
        }
      }
      target = wrapDegrees(yaw);
      resetFeedback();
      holding = true;
      settling = false;
    }
    return calculate(target, yaw, rate, now);
  }

  /** Returns clockwise correction; reverse travel uses the same yaw sign. */
  public double calculate(double desiredYaw, double yaw, double rate, double now) {
    if (!Double.isFinite(desiredYaw) || !Double.isFinite(yaw)
        || !Double.isFinite(rate) || !Double.isFinite(now)) {
      reset();
      return 0.0;
    }
    double desired = wrapDegrees(desiredYaw);
    double elapsed = now - lastTime;
    if (!Double.isFinite(target) || Math.abs(wrapDegrees(desired - target)) > 1e-6
        || !Double.isFinite(elapsed) || elapsed <= 0.0 || elapsed > 0.10) {
      resetFeedback();
      filteredRate = rate;
      elapsed = 0.02;
    }
    target = desired;
    lastTime = now;
    double dt = Math.min(0.05, elapsed);
    double alpha = dt / (Constants.HEADING_RATE_FILTER_SEC + dt);
    filteredRate += alpha * (rate - filteredRate);
    error = wrapDegrees(target - yaw);
    double feedbackError = Math.abs(error) <= Constants.HEADING_DEADBAND_DEG
        ? 0.0 : error;
    double nextIntegral = clamp(integral + feedbackError * dt,
        -Constants.HEADING_INTEGRAL_LIMIT, Constants.HEADING_INTEGRAL_LIMIT);
    double proposed = Constants.HEADING_KP * feedbackError
        + Constants.HEADING_KI * nextIntegral - Constants.HEADING_KD * filteredRate;
    // Stop accumulating error into a saturated output, but permit unwinding.
    if (Math.abs(proposed) <= Constants.HEADING_MAX_CORRECTION
        || feedbackError * proposed < 0.0) {
      integral = nextIntegral;
    }
    correction = clamp(Constants.HEADING_KP * feedbackError
        + Constants.HEADING_KI * integral - Constants.HEADING_KD * filteredRate,
        -Constants.HEADING_MAX_CORRECTION, Constants.HEADING_MAX_CORRECTION);
    return correction;
  }

  public boolean isHolding() {
    return holding;
  }

  public double getTarget() {
    return Double.isFinite(target) ? target : 0.0;
  }

  public double getError() {
    return error;
  }

  public double getCorrection() {
    return correction;
  }

  public static double wrapDegrees(double angle) {
    double wrapped = angle % 360.0;
    if (wrapped > 180.0) wrapped -= 360.0;
    if (wrapped < -180.0) wrapped += 360.0;
    return wrapped;
  }

  private static double clamp(double value, double lower, double upper) {
    return Math.max(lower, Math.min(upper, value));
  }
}
