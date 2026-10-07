package frc.robot.util;

import static org.junit.Assert.*;
import org.junit.Test;

public class PickupApproachGuardTest {
  @Test public void pushingObjectWithUnchangingImageStopsAndLatches() {
    PickupApproachGuard guard = new PickupApproachGuard();
    guard.reset(0, 80, 12);
    for (int i = 1; i < 100; i++) assertTrue(guard.update(i * 0.02, 80, true, 0.08));
    assertFalse(guard.update(2.02, 80, true, 0.08));
    assertEquals("NO_VISUAL_APPROACH_PROGRESS", guard.getFailure());
    assertFalse(guard.update(2.04, 160, true, 0));
  }

  @Test public void realSizeGrowthAllowsApproachUntilTotalBudget() {
    PickupApproachGuard guard = new PickupApproachGuard();
    guard.reset(0, 70, 3);
    for (int i = 1; i < 150; i++) assertTrue(guard.update(i * 0.02, 70 + i, true, 0.04));
    assertFalse(guard.update(3.02, 230, true, 0.04));
    assertEquals("APPROACH_TIME_LIMIT", guard.getFailure());
  }

  @Test public void waitingAndCenteringDoNotConsumeForwardBudget() {
    PickupApproachGuard guard = new PickupApproachGuard();
    guard.reset(0, 70, 3);
    for (int i = 1; i <= 500; i++) assertTrue(guard.update(i * 0.02, 70, true, 0));
    assertTrue(guard.update(10.02, 70, true, 0.04));
  }

  @Test public void duplicateFramesCannotInventProgress() {
    PickupApproachGuard guard = new PickupApproachGuard();
    guard.reset(0, 70, 3);
    for (int i = 1; i < 100; i++) assertTrue(guard.update(i * 0.02, 160, false, 0.04));
    assertFalse(guard.update(2.02, 160, false, 0.04));
  }

  @Test public void cameraAdjustmentPreservesTotalDrivingBudget() {
    PickupApproachGuard guard = new PickupApproachGuard();
    guard.reset(0, 80, 3);
    for (int i = 1; i <= 80; i++) {
      assertTrue(guard.update(i * 0.02, 80 + i, true, 0.04));
    }
    guard.rebaseView(5, 100);
    for (int i = 1; i < 70; i++) {
      assertTrue(guard.update(5 + i * 0.02, 100 + i, true, 0.04));
    }
    assertFalse(guard.update(6.42, 175, true, 0.04));
    assertEquals("APPROACH_TIME_LIMIT", guard.getFailure());
  }

  @Test public void changedPerspectiveRestartsSizeComparisonsWithoutUnlatchingFailure() {
    PickupApproachGuard guard = new PickupApproachGuard();
    guard.reset(0, 200, 12);
    for (int i = 1; i <= 80; i++) assertTrue(guard.update(i * 0.02, 200, true, 0.04));
    guard.rebaseView(5, 100);
    for (int i = 1; i < 100; i++) assertTrue(guard.update(5 + i * 0.02, 100, true, 0.04));
    assertFalse(guard.update(7.02, 100, true, 0.04));
    guard.rebaseView(8, 150);
    assertFalse(guard.update(8.02, 160, true, 0.04));
    assertEquals("NO_VISUAL_APPROACH_PROGRESS", guard.getFailure());
  }
}
