package frc.robot.util;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.assertEquals;

import org.junit.Test;

public class VisionPickupGateTest {
  private boolean firstZone(double x, double y, double bottom, double width) {
    return PickupGeometry.firstView(x, y, bottom, width);
  }

  @Test
  public void firstPhaseCanSwitchWhileObjectIsStillFullyVisible() {
    assertFalse(firstZone(0, 120, 155, 105));
    assertTrue(firstZone(0, 160, 185, 116));
    assertTrue(firstZone(0, 170, 190, 110));
    assertTrue(firstZone(0, 180, 210, 150));
  }

  @Test
  public void smallOrMisalignedTargetCannotAuthorizeNormalTransition() {
    assertFalse(firstZone(0, -1, 155, 180));
    assertFalse(firstZone(40, 180, 220, 180));
    assertFalse(firstZone(0, 180, 220, 60));
  }

  @Test
  public void rawAndFilteredZonesMustBothMatch() {
    boolean filtered = firstZone(0, 180, 215, 155);
    boolean raw = firstZone(35, 180, 215, 155);
    assertFalse(filtered && raw);
  }

  @Test
  public void duplicateCameraFrameCannotConfirmOverTime() {
    VisionPickupGate gate = new VisionPickupGate(3, 0.30);
    gate.reset(1);
    assertFalse(gate.update(true, 2, 0));
    for (int cycle = 1; cycle < 100; cycle++) {
      assertFalse(gate.update(true, 2, cycle * 0.02));
    }
  }

  @Test
  public void requiresBothDistinctFramesAndConfirmationTime() {
    VisionPickupGate gate = new VisionPickupGate(3, 0.30);
    gate.reset(1);
    assertFalse(gate.update(true, 2, 0));
    assertFalse(gate.update(true, 3, 0.10));
    assertFalse(gate.update(true, 4, 0.20));
    assertTrue(gate.update(true, 5, 0.31));
  }

  @Test
  public void lossOrMisalignmentRestartsConfirmation() {
    VisionPickupGate gate = new VisionPickupGate(3, 0.30);
    gate.reset(1);
    assertFalse(gate.update(true, 2, 0));
    assertFalse(gate.update(true, 3, 0.15));
    assertFalse(gate.update(false, 4, 0.20));
    assertFalse(gate.update(true, 5, 0.31));
    assertFalse(gate.update(true, 6, 0.50));
    assertTrue(gate.update(true, 7, 0.65));
  }

  @Test
  public void changingLiftPoseDiscardsPreviousAuthorizationAndFrames() {
    VisionPickupGate gate = new VisionPickupGate(3, 0.30);
    gate.reset(1);
    gate.update(true, 2, 0);
    gate.update(true, 3, 0.15);
    assertTrue(gate.update(true, 4, 0.31));
    gate.reset(10);
    assertFalse(gate.update(true, 4, 2));
    assertFalse(gate.update(true, 10, 2));
    assertFalse(gate.update(true, 11, 2.1));
    assertFalse(gate.update(true, 12, 2.2));
    assertTrue(gate.update(true, 13, 2.5));
  }

  @Test
  public void finalGripRequiresObjectLowAndLargeAtPickupHeight() {
    assertFalse(VisionPickupGate.inZone(0, 140, 185, 140, 21, 175, 225, 120));
    assertFalse(VisionPickupGate.inZone(0, 185, 228, 90, 21, 175, 225, 120));
    assertTrue(VisionPickupGate.inZone(0, 185, 228, 130, 21, 175, 225, 120));
  }

  @Test
  public void invalidGeometryOrFrameCannotAuthorizeGrip() {
    assertFalse(firstZone(Double.NaN, 180, 220, 150));
    assertFalse(firstZone(0, 230, 210, 150));
    assertFalse(firstZone(0, 180, 220, Double.POSITIVE_INFINITY));
    VisionPickupGate gate = new VisionPickupGate(3, 0.30);
    assertFalse(gate.update(true, Double.NaN, 1));
    assertFalse(gate.update(true, 0, 1));
  }

  @Test
  public void secondPhaseLateralCorrectionUsesRearWheelWithoutYaw() {
    double forward = 0.08;
    double strafe = 0.04;
    double[] powers = ThreeWheelDrive.calculate(strafe,
        forward / ThreeWheelDrive.FORWARD_PROJECTION, 0);
    assertEquals(forward, (powers[0] + powers[1]) / 2, 1e-9);
    assertEquals(strafe, ThreeWheelDrive.strafeDistance(
        powers[0], powers[1], powers[2]), 1e-9);
    assertEquals(0, (powers[0] - powers[1] - powers[2]) / 3, 1e-9);
    assertTrue(powers[2] > 0);
  }
}
