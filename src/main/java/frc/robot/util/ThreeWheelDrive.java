package frc.robot.util;

/** Kinematics for front-left, front-right and rear omni wheels at 120 degrees. */
public final class ThreeWheelDrive {
  public static final double FORWARD_PROJECTION = Math.sqrt(3.0) / 2.0;

  private ThreeWheelDrive() {}

  /**
   * Returns logical left, right, rear powers. Positive inputs mean movement
   * right, movement forward and clockwise rotation, respectively. All outputs
   * share one normalization factor so saturation preserves the requested path.
   */
  public static double[] calculate(double strafe, double forward, double clockwise) {
    if (!Double.isFinite(strafe) || !Double.isFinite(forward)
        || !Double.isFinite(clockwise)) {
      return new double[] {0.0, 0.0, 0.0};
    }

    double left = 0.5 * strafe + FORWARD_PROJECTION * forward + clockwise;
    double right = -0.5 * strafe + FORWARD_PROJECTION * forward - clockwise;
    double rear = strafe - clockwise;
    double scale = Math.max(1.0,
        Math.max(Math.abs(left), Math.max(Math.abs(right), Math.abs(rear))));
    return new double[] {left / scale, right / scale, rear / scale};
  }

  /** Chassis forward travel in mm from signed wheel travel in mm. */
  public static double forwardDistance(double left, double right) {
    return (left + right) / (2.0 * FORWARD_PROJECTION);
  }

  /** Chassis rightward travel in mm; rotational wheel travel cancels out. */
  public static double strafeDistance(double left, double right, double rear) {
    return (left - right + 2.0 * rear) / 3.0;
  }
}
