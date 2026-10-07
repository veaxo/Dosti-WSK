package frc.robot.util;

import static org.junit.Assert.*;
import org.junit.Test;

public class PickupGeometryTest {
  @Test public void firstViewPreparesSecondPhaseBeforeObjectFillsImage() {
    assertTrue(PickupGeometry.firstView(0, 120, 155, 110));
    assertTrue(PickupGeometry.firstView(0, 90, 130, 110));
    assertFalse(PickupGeometry.firstView(0, 160, 185, 60));
    assertFalse(PickupGeometry.firstView(40, 160, 185, 116));
    assertTrue(PickupGeometry.firstView(0, 160, 185, 116));
    assertTrue(PickupGeometry.firstView(0, 180, 210, 150));
  }

  @Test public void finalPoseCanConfirmWithoutPushingToImageEdge() {
    assertTrue(PickupGeometry.inView(0, 185, 210, 130, 120, 175, 175));
    assertFalse(PickupGeometry.inView(0, 140, 185, 140, 120, 175, 175));
    assertFalse(PickupGeometry.inView(0, 185, 210, 90, 120, 175, 175));
  }

  @Test public void clippedOrInvalidObjectNeverAuthorizesGrip() {
    assertFalse(PickupGeometry.inView(0, 200, 235, 150, 120, 175, 175));
    assertFalse(PickupGeometry.inView(0, 200, 250, 150, 120, 175, 175));
    assertFalse(PickupGeometry.firstView(Double.NaN, 180, 210, 150));
  }

  @Test public void edgeCannotBypassFirstViewWidthThreshold() {
    assertFalse(PickupGeometry.firstView(-14.314859, 217.486861, 239, 50.059326));
    assertFalse(PickupGeometry.firstView(0, 220, 239, 60));
    assertTrue(PickupGeometry.firstView(0, 220, 239, 110));
    assertFalse(PickupGeometry.inView(0, 220, 239, 130, 120, 175, 175));
    assertFalse(PickupGeometry.firstView(40, 220, 239, 60));
    assertFalse(PickupGeometry.firstView(0, 220, 240, 60));
    assertFalse(PickupGeometry.firstView(0, 220, 239, 0));
  }
}
