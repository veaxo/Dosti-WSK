package frc.robot.drive;

/** Kinematics for left, right and transverse rear omni wheels. */
public final class ThreeWheelDrive {
  private ThreeWheelDrive() {}

  /**
   * Converts robot-relative movement into logical wheel powers.
   * Positive forward drives both side wheels forward. Positive strafe moves
   * the robot right without intentionally rotating it.
   */
  public static double[] mix(double forward, double strafe, double turn) {
    if (!Double.isFinite(forward) || !Double.isFinite(strafe)
        || !Double.isFinite(turn)) {
      return new double[] {0, 0, 0};
    }

    double left = forward + 0.5 * strafe + turn;
    double right = forward - 0.5 * strafe - turn;
    double rear = strafe - turn;

    double peak = Math.max(1.0,
        Math.max(Math.abs(left), Math.max(Math.abs(right), Math.abs(rear))));
    return new double[] {left / peak, right / peak, rear / peak};
  }
}
