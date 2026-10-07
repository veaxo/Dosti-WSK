package frc.robot.util;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;

import org.junit.Test;

public class ThreeWheelDriveTest {
  private static final double EPSILON = 1e-9;

  @Test
  public void straightTravelLeavesRearWheelStationary() {
    double projection = Math.sqrt(3.0) / 2.0;
    assertArrayEquals(new double[] {projection, projection, 0.0},
        ThreeWheelDrive.calculate(0.0, 1.0, 0.0), EPSILON);
    assertArrayEquals(new double[] {-projection, -projection, 0.0},
        ThreeWheelDrive.calculate(0.0, -1.0, 0.0), EPSILON);
  }

  @Test
  public void sidewaysTravelUsesAllThreeWheels() {
    assertArrayEquals(new double[] {0.5, -0.5, 1.0},
        ThreeWheelDrive.calculate(1.0, 0.0, 0.0), EPSILON);
    assertArrayEquals(new double[] {-0.5, 0.5, -1.0},
        ThreeWheelDrive.calculate(-1.0, 0.0, 0.0), EPSILON);
  }

  @Test
  public void rotationUsesAllThreeWheelsWithoutTranslation() {
    double[] powers = ThreeWheelDrive.calculate(0.0, 0.0, 1.0);
    assertArrayEquals(new double[] {1.0, -1.0, -1.0}, powers, EPSILON);
    assertEquals(0.0, ThreeWheelDrive.forwardDistance(powers[0], powers[1]), EPSILON);
    assertEquals(0.0,
        ThreeWheelDrive.strafeDistance(powers[0], powers[1], powers[2]), EPSILON);
  }

  @Test
  public void legacyVisionCommandsKeepTheirLeftRightPower() {
    double left = 0.25;
    double right = -0.10;
    double[] powers = ThreeWheelDrive.calculate(0.0,
        (left + right) / (2.0 * ThreeWheelDrive.FORWARD_PROJECTION),
        (left - right) / 2.0);
    assertArrayEquals(new double[] {0.25, -0.10, -0.175}, powers, EPSILON);
  }

  @Test
  public void encoderProjectionSeparatesTravelFromRotation() {
    // 1000 mm forward + 150 mm right + 200 mm rotational wheel travel.
    double left = 1000.0 * Math.sqrt(3.0) / 2.0 + 75.0 + 200.0;
    double right = 1000.0 * Math.sqrt(3.0) / 2.0 - 75.0 - 200.0;
    double rear = 150.0 - 200.0;
    assertEquals(1000.0, ThreeWheelDrive.forwardDistance(left, right), EPSILON);
    assertEquals(150.0, ThreeWheelDrive.strafeDistance(left, right, rear), EPSILON);
  }

  @Test
  public void saturationPreservesCombinedTranslationAndRotation() {
    double[] powers = ThreeWheelDrive.calculate(1.0, 1.0, 1.0);
    assertEquals(1.0, powers[0], EPSILON);
    double forward = ThreeWheelDrive.forwardDistance(powers[0], powers[1]);
    double strafe = ThreeWheelDrive.strafeDistance(powers[0], powers[1], powers[2]);
    double rotation = (powers[0] - powers[1] - powers[2]) / 3.0;
    assertEquals(forward, strafe, EPSILON);
    assertEquals(forward, rotation, EPSILON);
  }

  @Test
  public void stopAndInvalidCommandsZeroEveryWheel() {
    double[] stopped = {0.0, 0.0, 0.0};
    assertArrayEquals(stopped, ThreeWheelDrive.calculate(0.0, 0.0, 0.0), EPSILON);
    assertArrayEquals(stopped, ThreeWheelDrive.calculate(Double.NaN, 1.0, 0.0), EPSILON);
    assertArrayEquals(stopped,
        ThreeWheelDrive.calculate(0.0, 1.0, Double.POSITIVE_INFINITY), EPSILON);
  }
}
