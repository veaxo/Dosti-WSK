package frc.robot.util;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import frc.robot.Constants;
import org.junit.Test;

public class HeadingControllerTest {
  private static final double EPSILON = 1e-9;

  @Test
  public void correctsBothDirectionsOfYawDisturbance() {
    HeadingController controller = new HeadingController();
    assertTrue(controller.calculate(0.0, 5.0, 0.0, 0.0) < 0.0);
    controller.reset();
    assertTrue(controller.calculate(0.0, -5.0, 0.0, 0.0) > 0.0);
  }

  @Test
  public void usesShortPathAcrossYawWrap() {
    HeadingController controller = new HeadingController();
    assertTrue(controller.calculate(-179.0, 179.0, 0.0, 0.0) > 0.0);
    assertEquals(2.0, controller.getError(), EPSILON);
    controller.reset();
    assertTrue(controller.calculate(179.0, -179.0, 0.0, 0.0) < 0.0);
    assertEquals(-2.0, controller.getError(), EPSILON);
  }

  @Test
  public void dampsRotationBeforeTheRobotOvershoots() {
    HeadingController controller = new HeadingController();
    assertTrue(controller.calculate(0.0, 0.0, 20.0, 0.0) < 0.0);
    controller.reset();
    assertTrue(controller.calculate(0.0, 0.0, -20.0, 0.0) > 0.0);
  }

  @Test
  public void saturationDoesNotWindUpAfterLargeDisturbance() {
    HeadingController controller = new HeadingController();
    for (int frame = 0; frame < 300; frame++) {
      assertEquals(Constants.HEADING_MAX_CORRECTION,
          controller.calculate(0.0, -90.0, 0.0, frame * 0.02), EPSILON);
    }
    assertEquals(0.0, controller.calculate(0.0, 0.0, 0.0, 6.0), EPSILON);
  }

  @Test
  public void compensationAccumulatesForPersistentSmallBias() {
    HeadingController controller = new HeadingController();
    double first = controller.calculate(0.0, -1.0, 0.0, 0.0);
    double last = first;
    for (int frame = 1; frame <= 100; frame++) {
      last = controller.calculate(0.0, -1.0, 0.0, frame * 0.02);
    }
    assertTrue(last > first);
    controller.reset();
    assertEquals(0.0, controller.calculate(0.0, 0.0, 0.0, 3.0), EPSILON);
  }

  @Test
  public void manualTurnWaitsForSettlingAndCapturesNewHeading() {
    HeadingController controller = new HeadingController();
    controller.hold(10.0, 0.0, 0.0);
    assertTrue(controller.isHolding());
    controller.manualTurn();
    assertFalse(controller.isHolding());
    assertEquals(0.0, controller.hold(30.0, 20.0, 1.0), EPSILON);
    assertEquals(0.0, controller.hold(35.0, 0.0, 1.1), EPSILON);
    assertFalse(controller.isHolding());
    assertEquals(0.0, controller.hold(37.0, 0.0, 1.3), EPSILON);
    assertTrue(controller.isHolding());
    assertEquals(37.0, controller.getTarget(), EPSILON);
    assertTrue(controller.hold(39.0, 0.0, 1.32) < 0.0);
  }

  @Test
  public void sameCorrectionWorksForForwardAndReverseTravel() {
    HeadingController controller = new HeadingController();
    double correction = controller.calculate(0.0, 4.0, 0.0, 0.0);
    for (double speed : new double[] {0.3, -0.3}) {
      double[] powers = ThreeWheelDrive.calculate(
          0.0, speed / ThreeWheelDrive.FORWARD_PROJECTION, correction);
      assertEquals(speed, (powers[0] + powers[1]) / 2.0, EPSILON);
      assertTrue(powers[0] < powers[1]);
      assertTrue(powers[2] > 0.0);
      assertEquals(0.0,
          ThreeWheelDrive.strafeDistance(powers[0], powers[1], powers[2]), EPSILON);
    }
  }

  @Test
  public void rejectsInvalidMeasurementsAndResetsHold() {
    HeadingController controller = new HeadingController();
    controller.hold(10.0, 0.0, 0.0);
    assertEquals(0.0, controller.hold(Double.NaN, 0.0, 0.02), EPSILON);
    assertFalse(controller.isHolding());
    assertEquals(0.0,
        controller.calculate(0.0, 0.0, Double.POSITIVE_INFINITY, 0.04), EPSILON);
  }

  @Test
  public void recoversFromDisturbanceAndMotorBiasInSimpleYawModel() {
    HeadingController controller = new HeadingController();
    double yaw = 0.0;
    double rate = 0.0;
    for (int frame = 0; frame < 1500; frame++) {
      if (frame == 250) yaw += 8.0;
      double correction = controller.calculate(0.0, yaw, rate, frame * 0.02);
      // Synthetic motor response, not a calibration of the real chassis.
      rate = 100.0 * correction + 2.0;
      yaw += rate * 0.02;
    }
    assertTrue("Failed to recover course: " + yaw, Math.abs(yaw) < 1.0);
  }
}
