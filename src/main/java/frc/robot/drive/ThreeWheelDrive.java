package frc.robot.drive;

/** Kinematics for left, right and transverse rear omni wheels. */
public final class ThreeWheelDrive {
  private ThreeWheelDrive() {}

  /**
   * Pure in-place rotation: all three motors spin at the exact same speed
   * so the robot rotates on the spot without translating.
   * Positive turn rotates clockwise (right); negative turn rotates
   * counter-clockwise (left).
   */
  public static double[] rotateInPlace(double turn) {
    if (!Double.isFinite(turn)) {
      return new double[] {0, 0, 0};
    }
    double speed = Math.max(-1.0, Math.min(1.0, turn));
    return new double[] {speed, -speed, -speed};
  }

  /**
   * Converts robot-relative movement into logical wheel powers.
   * Positive forward drives both side wheels forward. Positive strafe moves
   * the robot right without intentionally rotating it. When rotating in place
   * (forward == 0 and strafe == 0), all three wheels spin at the exact same
   * speed magnitude.
   */
  public static double[] mix(double forward, double strafe, double turn) {
    if (!Double.isFinite(forward) || !Double.isFinite(strafe)
        || !Double.isFinite(turn)) {
      return new double[] {0, 0, 0};
    }

    if (Math.abs(forward) < 1e-6 && Math.abs(strafe) < 1e-6) {
      return rotateInPlace(turn);
    }

    double comp = frc.robot.Constants.STRAFE_SIDE_COMPENSATION;
    double left = forward + comp * strafe + turn;
    double right = forward - comp * strafe - turn;
    double rear = strafe - turn;

    double peak = Math.max(1.0,
        Math.max(Math.abs(left), Math.max(Math.abs(right), Math.abs(rear))));
    return new double[] {left / peak, right / peak, rear / peak};
  }
}
