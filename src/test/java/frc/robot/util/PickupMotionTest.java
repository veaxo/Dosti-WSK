package frc.robot.util;

import static org.junit.Assert.*;
import org.junit.Test;

public class PickupMotionTest {
  @Test public void pixelNoiseDoesNotToggleSteering() {
    PickupMotion motion = new PickupMotion();
    for (double x : new double[] {12, -13, 17, -17, 11, 0}) {
      assertEquals(0, motion.steering(x, 0.004, 0.07), 1e-9);
    }
    assertTrue(motion.steering(20, 0.004, 0.07) > 0);
    assertTrue(motion.steering(15, 0.004, 0.07) > 0);
    assertEquals(0, motion.steering(9, 0.004, 0.07), 1e-9);
    assertEquals(0, motion.steering(15, 0.004, 0.07), 1e-9);
  }

  @Test public void reversalBrakesBeforeChangingDirection() {
    PickupMotion motion = new PickupMotion();
    motion.reset(0);
    double previous = 0;
    for (int i = 1; i <= 30; i++) {
      double value = motion.update(0, 0.07, i * 0.02)[1];
      assertTrue(value - previous <= 0.18 * 0.02 + 1e-9);
      previous = value;
    }
    double firstReverse = motion.update(0, -0.07, 0.62)[1];
    assertTrue(firstReverse >= 0);
    assertTrue(previous - firstReverse <= 0.35 * 0.02 + 1e-9);
    for (int i = 32; i <= 70; i++) previous = motion.update(0, -0.07, i * 0.02)[1];
    assertEquals(-0.07, previous, 1e-9);
  }

  @Test public void schedulerPauseDoesNotCauseLargePowerJump() {
    PickupMotion motion = new PickupMotion();
    motion.reset(0);
    assertTrue(motion.update(0.08, 0, 10)[0] <= 0.18 * 0.04 + 1e-9);
    motion.reset(10);
    assertEquals(0, motion.update(0.08, 0, 10)[0], 1e-9);
    assertEquals(0, motion.update(Double.NaN, 0, 11)[0], 1e-9);
  }

  @Test public void proximityAndAlignmentReduceForwardPower() {
    double far = PickupMotion.approachSpeed(0.12, 0.08, 50, 145, 0);
    double near = PickupMotion.approachSpeed(0.12, 0.08, 140, 145, 0);
    assertEquals(0.12, far, 1e-9);
    assertTrue(near > 0 && near < far);
    assertTrue(PickupMotion.approachSpeed(0.12, 0.08, 50, 145, 20) < far);
    assertEquals(0, PickupMotion.approachSpeed(0.12, 0.08, 50, 145, 28), 1e-9);
  }

  @Test public void acceptedCenterOffsetDoesNotThrottleDistantApproach() {
    for (double offset : new double[] {-18, -16, -10, 0, 10, 16, 18}) {
      assertEquals(0.12, PickupMotion.approachSpeed(0.12, 0.08, 50, 145, offset), 1e-9);
    }
    assertEquals(0.06, PickupMotion.approachSpeed(0.12, 0.08, 50, 145, 23), 1e-9);
    assertTrue(PickupMotion.approachSpeed(0.12, 0.08, 140, 145, 16) >= 0.08);
  }

  @Test public void currentFrameMisalignmentOverridesLaggingFilter() {
    double offset = PickupMotion.controlOffset(-13.470454, -52.135264);
    assertEquals(-52.135264, offset, 1e-9);
    assertEquals(0, PickupMotion.approachSpeed(0.12, 0.08, 116.352586, 145, offset), 1e-9);
    assertEquals(-40, PickupMotion.controlOffset(30, -40), 1e-9);
    assertEquals(-8, PickupMotion.controlOffset(13, -8), 1e-9);
    assertEquals(16, PickupMotion.controlOffset(16, 0), 1e-9);
  }

  @Test public void misalignmentStopsForwardWithoutResettingSmoothTurn() {
    PickupMotion motion = new PickupMotion();
    motion.reset(0);
    double[] command = null;
    for (int i = 1; i <= 30; i++) command = motion.update(0.08, 0.07, i * 0.02);
    motion.stopForward();
    double[] stopped = motion.update(0, 0.07, 0.62);
    assertEquals(0, stopped[0], 1e-9);
    assertEquals(command[1], stopped[1], 1e-9);
  }

  @Test public void approachRetainsWorkingSlowPowerUntilPositionIsConfirmed() {
    for (double width : new double[] {50, 100, 140, 145, 200}) {
      double power = PickupMotion.approachSpeed(0.12, 0.08, width, 145, 0);
      assertTrue(power >= 0.08 && power <= 0.12);
    }
    assertEquals(0.08, PickupMotion.approachSpeed(0.12, 0.08, 200, 145, 0), 1e-9);
    assertEquals(0.08, PickupMotion.approachSpeed(0.08, 0.08, 130, 120, 0), 1e-9);
  }
}
