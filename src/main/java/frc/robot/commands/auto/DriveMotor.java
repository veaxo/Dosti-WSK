package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import java.util.HashMap;
import java.util.Map;

public class DriveMotor extends CommandBase
{
  private final ExampleSubsystem o_subsystem;
  private final VisionSubsystem o_vision;
  
  public final double maxSpeed =                     0.5;
  public final double minSpeed =                     0.2;

  public int stateAutomatic =                         0;
  public double angleRobot =                          0;

  public double numberBlackBand =                     0;

  boolean flzekSharp = true;
  boolean flzekSonic = false;
  boolean flzekSonicRight = false;
  boolean flzekSonicLeft = false;

  double targetAngle = 0;  
  boolean angleLocked = false;

  boolean start = false;
  boolean previousRobotStart = false;
  boolean found = false;
  String basket_1 = null;
  String basket_2 = null;
  String seve_scan;

  private double sortTimer = 0;
  private int    sortState = 0;
  private int timedActionCase = -1;
  private double timedActionStart = 0;

  private static final int SERVO_CENTER = 170;
  private static final int SERVO_LEFT   = 107;
  private static final int SERVO_RIGHT  = 185;

  private enum VisionPickupState {
    CALIBRATING_SERVOS,
    IDLE,
    RAISING_FOR_SEARCH,
    STARTING_CAMERA,
    CENTERING,
    APPROACHING,
    RAISING_FOR_FINAL_VIEW,
    LOWERING_ARM,
    REACQUIRING_OBJECT,
    LOWERING_LIFT,
    GRABBING,
    RAISING_LIFT,
    RAISING_ARM,
    COMPLETE
  }

  private static final double VISION_CENTER_TOLERANCE_PX = 14.0;
  private static final double PICKUP_FRAME_MAX_AGE_SEC = 0.35;
  private static final double PICKUP_FILTER_ALPHA = 0.25;
  private static final double PICKUP_DRIVE_ACCEL_PER_SEC = 0.50;
  private static final double PICKUP_DRIVE_DECEL_PER_SEC = 1.50;
  private static final double VISION_TARGET_LOST_TIMEOUT_SEC = 0.75;
  private static final double VISION_CENTER_HOLD_SEC = 0.25;
  private static final double VISION_TARGET_CONFIRM_SEC = 0.15;
  private static final double VISION_CAMERA_CLOSE_CONFIRM_SEC = 0.30;
  private static final double VISION_CAMERA_CLOSE_WIDTH_PX = 170.0;
  private static final double PICKUP_ZONE_MIN_CENTER_Y_PX = 155.0;
  private static final double PICKUP_ZONE_MIN_BOTTOM_Y_PX = 140.0;
  // First view only decides when to switch to the closer second view. At the
  // old 180 px width the object could reach the camera edge first and stall.
  private static final double PICKUP_CAMERA_GRAB_WIDTH_PX = 105.0;
  private static final double PICKUP_CAMERA_GRAB_BOTTOM_Y_PX = 140.0;
  private static final double PICKUP_CAMERA_EDGE_MIN_WIDTH_PX = 90.0;
  private static final double PICKUP_CAMERA_EDGE_STOP_BOTTOM_Y_PX = 300.0;
  private static final int PICKUP_CAMERA_GRAB_MIN_FRAMES = 3;
  private static final double VISION_GRIP_DELAY_SEC = 1.50;
  private static final double SERVO_CALIBRATION_SETTLE_SEC = 1.50;
  private static final double LIFT_LOWER_SETTLE_SEC = 0.25;
  private static final double ARM_LOWER_SETTLE_SEC = 0.50;
  private static final double FINAL_VIEW_CONFIRM_SEC = 0.30;
  private static final int FINAL_VIEW_CONFIRM_FRAMES = 3;
  private static final double FINAL_APPROACH_SPEED = 0.16;
  // Second view: approach until the object is lower and larger in the frame.
  // Keep this bottom threshold below the camera-edge safety stop (235 px).
  private static final double FINAL_VIEW_GRAB_CENTER_Y_PX = 170.0;
  private static final double FINAL_VIEW_GRAB_BOTTOM_Y_PX = 225.0;
  private static final double FINAL_VIEW_MIN_WIDTH_PX = 110.0;
  private static final double LIFT_RAISE_SETTLE_SEC = 0.35;
  private static final double SERVO_LIFT_STEP = 0.8;
  private static final double PICKUP_LIFT_TRACK_STEP = 0.4;
  private static final double SERVO_ARM_STEP = 0.8;
  private static final double SERVO_CLAW_STEP = 2.0;
  private static final double VISION_TURN_KP = 0.004;
  private static final double VISION_MAX_TURN = 0.10;
  private static final double VISION_APPROACH_SPEED = 0.12;
  private static final double VISION_APPROACH_SLOW_SPEED = 0.08;
  private static final double PICKUP_SCAN_STEP_DEG = 5.0;
  private static final double PICKUP_SCAN_DWELL_SEC = 0.45;
  private static final double PICKUP_ARM_UPDATE_SEC = 0.10;
  // During approach keep the target in the lower half of the 240 px frame.
  private static final double PICKUP_IMAGE_CENTER_Y_PX = 170.0;
  private static final double PICKUP_IMAGE_Y_DEADBAND_PX = 18.0;
  private static final double PICKUP_ARM_KP = 0.015;
  private static final double PICKUP_ARM_MAX_CORRECTION_DEG = 0.8;
  // User-calibrated physical positions. Do not infer direction from the
  // numeric angle or swap these values automatically.
  private static final int CLAW_OPEN_ANGLE = 18;
  private static final int CLAW_CLOSED_ANGLE = 170;
  // 80 degrees looks forward; increasing the angle aims down at the floor.
  // The arm servo's 15-degree safety margin clamps its 180-degree endpoint
  // to 165 degrees in ExampleSubsystem.
  private static final double ARM_SEARCH_ANGLE = 80.0;
  private static final double ARM_SCAN_DOWN_ANGLE = 165.0;
  private static final int LIFT_PICKUP_ANGLE = 110;
  private static final int LIFT_SEARCH_ANGLE = 50;
  // Lift position during the second camera-guided approach; tune separately.
  private static final int LIFT_FINAL_VIEW_ANGLE = 15;

  private VisionPickupState visionPickupState = VisionPickupState.IDLE;
  private double visionPickupStateStart = 0;
  private double visionTargetLastSeen = 0;
  private double visionTargetSeenSince = 0;
  private double visionCenteredSince = 0;
  private double visionCameraCloseSince = 0;
  private int pickupCameraCloseFrames = 0;
  private double pickupScanArmTarget = ARM_SEARCH_ANGLE;
  private int pickupScanDirection = 1;
  private double pickupScanNextStepAt = 0;
  private double pickupTrackedArmTarget = ARM_SEARCH_ANGLE;
  private double pickupTrackedLiftTarget = LIFT_SEARCH_ANGLE;
  private double pickupLastArmUpdateAt = 0;
  private double pickupLastFrameTimestamp = 0;
  private boolean pickupFrameUpdated = false;
  private boolean pickupFilterReady = false;
  private double pickupFilteredX = 0;
  private double pickupFilteredY = PICKUP_IMAGE_CENTER_Y_PX;
  private double pickupFilteredWidth = 0;
  private double pickupFilteredBottom = PICKUP_IMAGE_CENTER_Y_PX;
  private double pickupDriveLeft = 0;
  private double pickupDriveRight = 0;
  private double pickupDriveLastAt = 0;
  private String pickupCandidateColor = null;
  private String pickupLockedColor = null;
  private int pickupServoStage = 0;
  private double pickupServoSettledSince = 0;
  private boolean visionPickupComplete = false;
  private int visionPickupCase = -1;

  // This state belongs only to visionColor()/visionAruco(). Keeping it
  // separate prevents standalone recognition cases from resetting pickup.
  private static final double VISION_RECOGNITION_CONFIRM_SEC = 0.15;
  private static final double VISION_RECOGNITION_DURATION_SEC = 3.0;
  private static final double VISION_SCAN_LIFT_ANGLE = SERVO_CENTER;
  private static final double VISION_SCAN_ARM_FORWARD_ANGLE = ARM_SEARCH_ANGLE;
  private int visionRecognitionCase = -1;
  private String visionRecognitionMode = "";
  private String visionRecognitionCandidate = "None";
  private double visionRecognitionSeenSince = 0;
  private double visionRecognitionStartedAt = -1;
  private String lastDetectedColor = "None";
  private String lastDetectedAruco = "None";

  private final Map<String, Integer> ballCount = new HashMap<>();

  public DriveMotor(ExampleSubsystem subsystem, VisionSubsystem vision) {
    o_subsystem = subsystem;
    o_vision = vision;
    addRequirements(o_subsystem);
    ballCount.put("Red ball",    0);
    ballCount.put("Yellow ball", 0);
    ballCount.put("green ball",  0);
  }

  @Override
  public void initialize() {
    // During autonomous the physical Start button starts this command. Do not
    // let the same press also toggle camera ownership between Python and Java.
    o_vision.setModeButtonEnabled(false);
    o_vision.resetCameraScan();
    // o_subsystem.servo_Hook(300);
    o_subsystem.resetYaw();
    o_subsystem.resetEncoder();
    o_subsystem.setButtonLed("Running", false);
    o_subsystem.setButtonLed("Stopped", true);
    o_subsystem.setButtonLed("Start",   true);
    o_subsystem.setButtonLed("Stop",    true);
    angleRobot = normalizeYaw(o_subsystem.getYaw()); 
    previousRobotStart = false;
    resetVisionPickup();
    resetVisionRecognition();
    resetTimedActions();
  }

  @Override
  public void execute() {
    boolean robotStartPressed = o_subsystem.getButtonState("Start");

    if (robotStartPressed && !previousRobotStart) {
      start = true;
      stateAutomatic = 0;
      resetVisionPickup();
      resetVisionRecognition();
      resetTimedActions();
      flzekSharp = true;         
      flzekSonic = false;
      flzekSonicLeft = true;      
      flzekSonicRight = true;
      o_subsystem.setButtonLed("Running", true);
      o_subsystem.setButtonLed("Stopped", false); 
    }

    previousRobotStart = robotStartPressed;

    if (o_subsystem.getButtonState("Stop")) {
      start = false;
      stateAutomatic = -1;
      o_subsystem.setButtonLed("Running", false);
      o_subsystem.setButtonLed("Stopped", true);
      o_subsystem.resetEncoder();
      o_subsystem.resetYaw();
      o_subsystem.stopAllMotors();
    }

    if (start) {
      if (o_subsystem.getButtonState("Stop")) {
        start = false;
        stateAutomatic = -1;
        o_subsystem.setButtonLed("Running", false);
        o_subsystem.setButtonLed("Stopped", true);
        o_subsystem.resetEncoder();
        o_subsystem.resetYaw();
        o_subsystem.stopAllMotors();
      }

      o_vision.setValue("Auto Case", stateAutomatic);
      switch (stateAutomatic) {
        case 0:
          rotateTheRobot(90);
          break;
        case 1:
          goBackSonic(6);
          break;
        case 2:
          rotateTheRobot(0);
          break;
        case 3:
          goForwardDistance(60);
          break;
        case 4:
          visionPickup();
          break;
        case 5:
          rotateTheRobot(0);
          break;
        case 6:
          goBackDistance(100);
          break;
        case 7:
        goBackSonic(7);
          break;
        case 8:
        rotateTheRobot(90);
          break;
        case 9:
          o_subsystem.servo_Hook(30);
          stateAutomatic++;
          break;
        case 10:
          rotateTheRobot(0);
          break;
        case 11:
          goForwardDistance(55);
          break;
        case 12:
          rotateTheRobot(90);
          break;
        case 13:
          goForwardDistance(100);
          break;
        case 14:
          goForwardSonic(30);
          break;
        case 15:
          rotateTheRobot(0);
          break;
        case 16:
          goForwardSonic(5);
          break;
        case 17:
          rotateTheRobot(90);
          break;
        case 18:
          visionPickup();
          break;
        case 19:
          goBackDistance(10);
          break;
        case 20:
          rotateTheRobot(180);
          break;
        case 21:
          goForwardDistance(50);
          break;
        case 22:
          rotateTheRobot(90);
          break;
        case 23:
          goForwardSonic(20);
          break;
        case 24:
          rotateTheRobot(180);
          break;
        case 25:
        goForwardSonic(6);
        break;
        case 26:
        rotateTheRobot(270);
        break;
        case 27:
        goForwardDistance(110);
        break;
        case 28:
        // goBackSonic(7);
        break;
        case 29:
          stateAutomatic++;
        break;
        case 30:
        goForwardDistance(40);
        break;
        case 31:
          closed_Hand_Kub();
          advanceAfterDelay(2.0);
          break;
        case 32:
        goForwardSharp(32);
        break;
        case 33:
        rotateTheRobot(0);
        break;
        case 34:
        goForwardDistance(60);
        break;
        case 35:
        goBackSonic(34);
        break;
        case 36:
          stateAutomatic++;
        break;
        case 37:
          stateAutomatic++;
        break;
        case 38:
          stateAutomatic++;
          break;

        default:
          o_subsystem.stopAllMotors();
          start = false;
          o_subsystem.resetEncoder();
          o_subsystem.resetYaw();
          o_subsystem.setButtonLed("Running", false);
          o_subsystem.setButtonLed("Stopped", true);
          break;
      }
    } 
  }

  // ─── Автоматический захват объекта по Vision ─────────────────────────────

  /**
   * Positions the camera, reads colors for three seconds, then advances to
   * the next case. It does not close the claw or start pickup.
   */
  public void visionColor() {
    runStandaloneVisionRecognition(VisionSubsystem.MODE_COLOR);
  }

  /**
   * Positions the camera, reads ArUco markers for three seconds, then
   * advances to the next case.
   */
  public void visionAruco() {
    runStandaloneVisionRecognition(VisionSubsystem.MODE_MARKER);
  }

  public String getLastDetectedColor() {
    return lastDetectedColor;
  }

  public String getLastDetectedAruco() {
    return lastDetectedAruco;
  }

  private void resetVisionRecognition() {
    visionRecognitionCase = -1;
    visionRecognitionMode = "";
    visionRecognitionCandidate = "None";
    visionRecognitionSeenSince = 0;
    visionRecognitionStartedAt = -1;
  }

  private void runStandaloneVisionRecognition(String mode) {
    double now = Timer.getFPGATimestamp();

    if (visionRecognitionCase != stateAutomatic
        || !mode.equals(visionRecognitionMode)) {
      visionRecognitionCase = stateAutomatic;
      visionRecognitionMode = mode;
      visionRecognitionCandidate = "None";
      visionRecognitionSeenSince = 0;
      visionRecognitionStartedAt = -1;
      if (VisionSubsystem.MODE_COLOR.equals(mode)) {
        lastDetectedColor = "None";
        o_vision.setText("Last Color", "None");
      } else {
        lastDetectedAruco = "None";
        o_vision.setText("Last Marker", "None");
      }

      // Discard an old frame before this case starts. The Python process is
      // kept alive; only its algorithm is switched through the control file.
      o_vision.resetCameraScan();
      o_vision.setMode(mode);
      o_vision.resetROI();
      o_vision.setProcessed(true);
      o_vision.setText("Recognition State", "WAITING_FOR_" + mode);
      o_vision.setText("Recognition Result", "None");
    }

    o_subsystem.stopDrive();
    o_subsystem.setMotorGear(0);

    // The camera and claw are carried by these two servos. Complete the
    // viewing pose before the three-second recognition window begins.
    boolean liftCentered = o_subsystem.moveServoLiftToward(
        VISION_SCAN_LIFT_ANGLE, SERVO_LIFT_STEP);
    boolean armForward = o_subsystem.moveServoArmToward(
        VISION_SCAN_ARM_FORWARD_ANGLE, SERVO_ARM_STEP);
    if (!liftCentered || !armForward) {
      visionRecognitionCandidate = "None";
      visionRecognitionSeenSince = 0;
      o_vision.setText("Recognition State", "POSITIONING_" + mode);
      return;
    }

    if (visionRecognitionStartedAt < 0) {
      visionRecognitionStartedAt = now;
    }

    if (now - visionRecognitionStartedAt >= VISION_RECOGNITION_DURATION_SEC) {
      String result = VisionSubsystem.MODE_COLOR.equals(mode)
          ? lastDetectedColor : lastDetectedAruco;
      o_vision.setText("Recognition Result", result);
      o_vision.setText("Recognition State",
          "None".equals(result) ? "DONE_NO_TARGET" : "DONE");
      visionRecognitionCase = -1;
      stateAutomatic++;
      return;
    }

    boolean cameraReady = VisionSubsystem.MODE_COLOR.equals(mode)
        ? o_vision.isColorCameraReady()
        : o_vision.isMarkerCameraReady();
    if (!cameraReady) {
      visionRecognitionCandidate = "None";
      visionRecognitionSeenSince = 0;
      o_vision.setText("Recognition State", "WAITING_FOR_" + mode);
      return;
    }

    if (!o_vision.isVisionFound()) {
      visionRecognitionCandidate = "None";
      visionRecognitionSeenSince = 0;
      o_vision.setText("Recognition State", "SEARCHING_" + mode);
      return;
    }

    String detected = o_vision.getCurrentDetection();
    if (detected == null || detected.trim().isEmpty()) {
      visionRecognitionCandidate = "None";
      visionRecognitionSeenSince = 0;
      return;
    }

    if (!detected.equals(visionRecognitionCandidate)) {
      visionRecognitionCandidate = detected;
      visionRecognitionSeenSince = now;
      o_vision.setText("Recognition State", "CONFIRMING_" + detected);
      return;
    }

    if (now - visionRecognitionSeenSince
        < VISION_RECOGNITION_CONFIRM_SEC) {
      return;
    }

    if (VisionSubsystem.MODE_COLOR.equals(mode)) {
      lastDetectedColor = detected;
      o_vision.setText("Last Color", detected);
    } else {
      lastDetectedAruco = detected;
      o_vision.setText("Last Marker", detected);
    }
    o_vision.setText("Recognition Result", detected);
    o_vision.setText("Recognition State", "DETECTED_" + detected);
  }

  private void resetTimedActions() {
    timedActionCase = -1;
    timedActionStart = 0;
  }

  private void advanceAfterDelay(double delaySeconds) {
    double now = Timer.getFPGATimestamp();
    if (timedActionCase != stateAutomatic) {
      timedActionCase = stateAutomatic;
      timedActionStart = now;
      return;
    }

    if (now - timedActionStart >= delaySeconds) {
      timedActionCase = -1;
      stateAutomatic++;
    }
  }

  private void resetVisionPickup() {
    visionPickupState = VisionPickupState.IDLE;
    visionPickupStateStart = Timer.getFPGATimestamp();
    visionTargetLastSeen = visionPickupStateStart;
    visionTargetSeenSince = 0;
    visionCenteredSince = 0;
    visionCameraCloseSince = 0;
    pickupCameraCloseFrames = 0;
    pickupScanArmTarget = ARM_SEARCH_ANGLE;
    pickupScanDirection = 1;
    pickupScanNextStepAt = 0;
    pickupTrackedArmTarget = ARM_SEARCH_ANGLE;
    pickupTrackedLiftTarget = LIFT_SEARCH_ANGLE;
    pickupLastArmUpdateAt = 0;
    pickupLastFrameTimestamp = 0;
    pickupFrameUpdated = false;
    pickupFilterReady = false;
    pickupDriveLeft = 0;
    pickupDriveRight = 0;
    pickupDriveLastAt = 0;
    pickupCandidateColor = null;
    pickupLockedColor = null;
    pickupServoStage = 0;
    pickupServoSettledSince = 0;
    visionPickupComplete = false;
    visionPickupCase = -1;
    o_vision.setText("Pickup State", visionPickupState.name());
    o_vision.setText("Pickup Error", "None");
    o_vision.setText("Pickup Warning", "None");
  }

  private boolean isPickupObjectVisible() {
    if (!o_vision.isColorCameraReady()
        || !o_vision.isVisionFound()
        || !o_vision.isVisionFrameRecent(PICKUP_FRAME_MAX_AGE_SEC)) {
      return false;
    }

    String detected = o_vision.getCurrentDetection();
    if (detected == null) {
      return false;
    }

    String name = detected.trim();
    boolean namedObject = !name.isEmpty()
        && !name.equalsIgnoreCase("None")
        && !name.equalsIgnoreCase("Color")
        && !name.equalsIgnoreCase("Color_None");
    return namedObject && (pickupLockedColor == null
        || pickupLockedColor.equalsIgnoreCase(detected));
  }

  private void enterVisionPickupState(VisionPickupState state) {
    visionPickupState = state;
    visionPickupStateStart = Timer.getFPGATimestamp();
    visionCenteredSince = 0;
    visionCameraCloseSince = 0;
    pickupCameraCloseFrames = 0;
    pickupServoSettledSince = 0;
    if (state == VisionPickupState.IDLE) {
      visionTargetSeenSince = 0;
      pickupCandidateColor = null;
      pickupLockedColor = null;
      pickupScanArmTarget = ARM_SEARCH_ANGLE;
      pickupScanDirection = 1;
      pickupScanNextStepAt = visionPickupStateStart + PICKUP_SCAN_DWELL_SEC;
      pickupFilterReady = false;
      pickupTrackedLiftTarget = LIFT_SEARCH_ANGLE;
    }
    if (state == VisionPickupState.REACQUIRING_OBJECT) {
      // Require a new image after changing the camera pose. The old frame
      // describes the first approach and cannot authorize the final grip.
      pickupFilterReady = false;
      pickupLastFrameTimestamp = o_vision.getVisionFrameTimestamp();
      pickupFrameUpdated = false;
      visionTargetSeenSince = 0;
    }
    if (state == VisionPickupState.RAISING_FOR_SEARCH) {
      pickupServoStage = 0;
    }
    o_vision.setText("Pickup State", state.name());
  }

  /**
   * Call this method from an autonomous switch case. It waits for a colored
   * object, performs the complete pickup, then advances stateAutomatic once.
   */
  public void visionPickup() {
    if (visionPickupCase != stateAutomatic) {
      visionPickupState = VisionPickupState.IDLE;
      visionPickupStateStart = Timer.getFPGATimestamp();
      visionTargetLastSeen = visionPickupStateStart;
      visionTargetSeenSince = 0;
      visionCenteredSince = 0;
      visionCameraCloseSince = 0;
      pickupCameraCloseFrames = 0;
      pickupScanArmTarget = ARM_SEARCH_ANGLE;
      pickupScanDirection = 1;
      pickupScanNextStepAt = 0;
      pickupTrackedArmTarget = ARM_SEARCH_ANGLE;
      pickupTrackedLiftTarget = LIFT_SEARCH_ANGLE;
      pickupLastArmUpdateAt = 0;
      pickupLastFrameTimestamp = 0;
      pickupFrameUpdated = false;
      pickupFilterReady = false;
      pickupDriveLeft = 0;
      pickupDriveRight = 0;
      pickupDriveLastAt = 0;
      pickupCandidateColor = null;
      pickupLockedColor = null;
      pickupServoStage = 0;
      pickupServoSettledSince = 0;
      visionPickupComplete = false;
      visionPickupCase = stateAutomatic;
      o_vision.setText("Pickup Error", "None");
      o_vision.setText("Pickup Warning", "None");
      stopPickupDrive();
      o_subsystem.setMotorGear(0);
      // A floor object moves to the bottom edge of the image as the robot
      // approaches. The cropped scan ROI could therefore hide it immediately
      // before pickup.
      o_vision.resetROI();
      enterVisionPickupState(VisionPickupState.CALIBRATING_SERVOS);
    }

    // The persistent Python process changes algorithms without releasing the
    // camera. Confirm COLOR output before moving any claw servo.
    o_vision.setMode(VisionSubsystem.MODE_COLOR);
    o_vision.setProcessed(true);

    if (visionPickupState == VisionPickupState.CALIBRATING_SERVOS) {
      runVisionPickup();
      return;
    }

    if (visionPickupState == VisionPickupState.STARTING_CAMERA) {
      stopPickupDrive();
      o_subsystem.setMotorGear(0);
      if (o_vision.isColorCameraReady()) {
        enterVisionPickupState(VisionPickupState.RAISING_FOR_SEARCH);
      } else {
        o_vision.setText("Pickup State", "WAITING_FOR_CAMERA");
      }
      return;
    }

    if (visionPickupState == VisionPickupState.RAISING_FOR_SEARCH) {
      runVisionPickup();
      return;
    }

    boolean pickupCommitted =
        visionPickupState == VisionPickupState.RAISING_FOR_FINAL_VIEW
        || visionPickupState == VisionPickupState.LOWERING_ARM
        || visionPickupState == VisionPickupState.REACQUIRING_OBJECT
        || visionPickupState == VisionPickupState.LOWERING_LIFT
        || visionPickupState == VisionPickupState.GRABBING
        || visionPickupState == VisionPickupState.RAISING_LIFT
        || visionPickupState == VisionPickupState.RAISING_ARM
        || visionPickupState == VisionPickupState.COMPLETE;
    if (!o_vision.isColorCameraReady() && !pickupCommitted) {
      stopPickupDrive();
      o_subsystem.setMotorGear(0);
      // Freeze the viewing pose on a lost frame. Moving the lift while the
      // camera is unavailable would make reacquisition harder.
      o_vision.setText("Pickup State", "WAITING_FOR_CAMERA");
      return;
    }

    runVisionPickup();

    if (visionPickupState == VisionPickupState.COMPLETE) {
      stopPickupDrive();
      o_subsystem.setMotorGear(0);
      o_vision.setText("Pickup State", "DONE");
      stateAutomatic++;
    }
  }

  /**
   * Runs the complete pickup sequence. Returns true while it owns the drive
   * and manipulator, so the normal autonomous route pauses safely.
   */
  private boolean runVisionPickup() {
    double now = Timer.getFPGATimestamp();
    boolean targetVisible = isPickupObjectVisible();

    if (targetVisible) {
      visionTargetLastSeen = now;
      updatePickupVisionFilter();
    } else {
      pickupFrameUpdated = false;
    }

    switch (visionPickupState) {
      case CALIBRATING_SERVOS:
        stopPickupDrive();
        o_subsystem.setMotorGear(0);
        boolean liftReady = o_subsystem.moveServoLiftToward(
            LIFT_SEARCH_ANGLE, SERVO_LIFT_STEP);
        boolean armReady = o_subsystem.moveServoArmToward(
            ARM_SEARCH_ANGLE, SERVO_ARM_STEP);
        boolean clawReady = o_subsystem.moveServoClawToward(
            CLAW_OPEN_ANGLE, SERVO_CLAW_STEP);
        o_vision.setText("Pickup Step", "POSITIONING_CAMERA_FORWARD_AT_TOP");
        if (!liftReady || !armReady || !clawReady) {
          pickupServoSettledSince = 0;
          return true;
        }
        if (pickupServoSettledSince == 0) {
          pickupServoSettledSince = now;
        } else if (now - pickupServoSettledSince
            >= SERVO_CALIBRATION_SETTLE_SEC) {
          enterVisionPickupState(VisionPickupState.STARTING_CAMERA);
        }
        return true;

      case RAISING_FOR_SEARCH:
        stopPickupDrive();
        o_subsystem.setMotorGear(0);
        if (pickupServoStage == 0) {
          o_vision.setText("Pickup Step", "OPENING_CLAW");
          if (o_subsystem.moveServoClawToward(
              CLAW_OPEN_ANGLE, SERVO_CLAW_STEP)) {
            pickupServoStage = 1;
          }
          return true;
        }

        o_vision.setText("Pickup Step", "RAISING_CAMERA_FOR_SEARCH");
        boolean cameraAtSearchHeight = o_subsystem.moveServoArmToward(
            ARM_SEARCH_ANGLE, SERVO_ARM_STEP);
        boolean liftAtSearchHeight = o_subsystem.moveServoLiftToward(
            LIFT_SEARCH_ANGLE, SERVO_LIFT_STEP);
        if (!cameraAtSearchHeight || !liftAtSearchHeight) {
          pickupServoSettledSince = 0;
          return true;
        }

        if (pickupServoSettledSince == 0) {
          pickupServoSettledSince = now;
        } else if (now - pickupServoSettledSince >= LIFT_LOWER_SETTLE_SEC) {
          o_vision.setText("Pickup Step", "CAMERA_READY_TO_SEARCH");
          enterVisionPickupState(VisionPickupState.IDLE);
        }
        return true;

      case STARTING_CAMERA:
        stopPickupDrive();
        o_subsystem.setMotorGear(0);
        return true;

      case IDLE:
        stopPickupDrive();
        o_subsystem.setMotorGear(0);
        boolean searchLiftReady = o_subsystem.moveServoLiftToward(
            LIFT_SEARCH_ANGLE, SERVO_LIFT_STEP);
        boolean searchClawReady = o_subsystem.moveServoClawToward(
            CLAW_OPEN_ANGLE, SERVO_CLAW_STEP);
        if (!searchLiftReady || !searchClawReady) {
          visionTargetSeenSince = 0;
          o_vision.setText("Pickup Step", "PREPARING_FOR_SEARCH");
          return true;
        }
        o_vision.setText("Pickup Step", "SCANNING_DOWN_FOR_OBJECT");
        if (visionPickupComplete || !targetVisible) {
          visionTargetSeenSince = 0;
          pickupCandidateColor = null;
          scanPickupCamera(now);
          return true;
        }

        String detectedColor = o_vision.getCurrentDetection();
        if (visionTargetSeenSince == 0
            || !detectedColor.equalsIgnoreCase(pickupCandidateColor)) {
          visionTargetSeenSince = now;
          pickupCandidateColor = detectedColor;
          pickupTrackedArmTarget = o_subsystem.getServoArmAngle();
          return true;
        }
        if (now - visionTargetSeenSince < VISION_TARGET_CONFIRM_SEC) {
          return true;
        }

        pickupLockedColor = pickupCandidateColor;
        pickupTrackedArmTarget = o_subsystem.getServoArmAngle();
        pickupLastArmUpdateAt = now;
        o_vision.setText("Pickup Warning", "None");
        enterVisionPickupState(VisionPickupState.CENTERING);
        return true;

      case CENTERING:
        o_subsystem.setMotorGear(0);
        o_subsystem.servo_Hook(CLAW_OPEN_ANGLE);
        o_vision.setText("Pickup Step", "CENTERING_AND_TRACKING_OBJECT");
        if (!keepVisionTargetOrWait(now, targetVisible)) {
          return true;
        }
        followPickupTarget(now);

        double centerOffset = pickupFilteredX;
        if (Math.abs(centerOffset) > VISION_CENTER_TOLERANCE_PX) {
          visionCenteredSince = 0;
          double turn = clampVision(
              centerOffset * VISION_TURN_KP,
              -VISION_MAX_TURN,
              VISION_MAX_TURN);
          setPickupDrive(turn, -turn, now);
        } else {
          setPickupDrive(0, 0, now);
          if (Math.abs(pickupDriveLeft) >= 0.015
              || Math.abs(pickupDriveRight) >= 0.015) {
            visionCenteredSince = 0;
          } else if (visionCenteredSince == 0) {
            visionCenteredSince = now;
          } else if (now - visionCenteredSince >= VISION_CENTER_HOLD_SEC) {
            enterVisionPickupState(VisionPickupState.APPROACHING);
          }
        }
        return true;

      case APPROACHING:
        o_vision.setText("Pickup Step", "APPROACHING_OBJECT");
        o_subsystem.setMotorGear(0);
        o_subsystem.servo_Hook(CLAW_OPEN_ANGLE);
        if (!keepVisionTargetOrWait(now, targetVisible)) {
          o_vision.setText("Pickup Warning", "TARGET_LOST_STOPPED");
          return true;
        }
        followPickupTarget(now);

        double objectWidth = pickupFilteredWidth;
        o_vision.setValue("Pickup Target Width", objectWidth);

        double objectBottom = pickupFilteredBottom;
        o_vision.setValue("Pickup Target Bottom", objectBottom);
        double offset = pickupFilteredX;
        boolean aligned = Math.abs(offset)
            <= VISION_CENTER_TOLERANCE_PX * 1.5;
        // Switch to the second camera view once the object is low and large
        // enough. If it reaches the image edge first, a slightly smaller
        // width is acceptable; the second view still confirms the grip.
        // Require distinct frames so one noisy contour cannot trigger it.
        boolean cameraGrabReady = aligned
            && isPickupZoneReady()
            && ((objectWidth >= PICKUP_CAMERA_GRAB_WIDTH_PX
                    && objectBottom >= PICKUP_CAMERA_GRAB_BOTTOM_Y_PX)
                || (objectWidth >= PICKUP_CAMERA_EDGE_MIN_WIDTH_PX
                    && objectBottom >= PICKUP_CAMERA_EDGE_STOP_BOTTOM_Y_PX));
        if (cameraGrabReady) {
          stopPickupDrive();
          o_vision.setText("Pickup Step", "CONFIRMING_CAMERA_GRAB_ZONE");
          if (visionCameraCloseSince == 0) {
            visionCameraCloseSince = now;
          }
          if (pickupFrameUpdated) {
            pickupCameraCloseFrames++;
          }
          if (pickupCameraCloseFrames >= PICKUP_CAMERA_GRAB_MIN_FRAMES
              && now - visionCameraCloseSince
                  >= VISION_CAMERA_CLOSE_CONFIRM_SEC) {
            o_vision.setText("Pickup Step", "OBJECT_IN_LOWER_GRAB_ZONE");
            o_vision.setText("Pickup Warning", "None");
            enterVisionPickupState(VisionPickupState.RAISING_FOR_FINAL_VIEW);
          }
          return true;
        }
        visionCameraCloseSince = 0;
        pickupCameraCloseFrames = 0;

        double slowProgress = clampVision(
            (objectWidth - 65.0)
                / (VISION_CAMERA_CLOSE_WIDTH_PX - 65.0),
            0.0, 1.0);
        double forward = VISION_APPROACH_SPEED
            + (VISION_APPROACH_SLOW_SPEED - VISION_APPROACH_SPEED)
                * slowProgress;
        // Do not taper drive to zero before the grab-zone thresholds are met:
        // the robot would stop short and require a physical push to continue.
        if (objectBottom >= PICKUP_CAMERA_EDGE_STOP_BOTTOM_Y_PX
            || Math.abs(offset)
            > VISION_CENTER_TOLERANCE_PX * 2.5) {
          forward = 0.0;
        }
        if (objectBottom >= PICKUP_CAMERA_EDGE_STOP_BOTTOM_Y_PX) {
          o_vision.setText("Pickup Warning", "CAMERA_EDGE_NOT_IN_GRAB_ZONE");
        }
        double steering = clampVision(
            (Math.abs(offset) <= VISION_CENTER_TOLERANCE_PX
                ? 0.0
                : offset) * VISION_TURN_KP,
            -VISION_MAX_TURN,
            VISION_MAX_TURN);
        setPickupDrive(forward + steering, forward - steering, now);
        return true;

      case RAISING_FOR_FINAL_VIEW:
        o_vision.setText("Pickup Step", "RAISING_LIFT_FOR_SECOND_LOOK");
        stopPickupDrive();
        o_subsystem.setMotorGear(0);
        o_subsystem.servo_Hook(CLAW_OPEN_ANGLE);
        if (!o_subsystem.moveServoLiftToward(
            LIFT_FINAL_VIEW_ANGLE, SERVO_LIFT_STEP)) {
          pickupServoSettledSince = 0;
          return true;
        }
        if (pickupServoSettledSince == 0) {
          pickupServoSettledSince = now;
        } else if (now - pickupServoSettledSince
            >= LIFT_RAISE_SETTLE_SEC) {
          enterVisionPickupState(VisionPickupState.LOWERING_ARM);
        }
        return true;

      case LOWERING_ARM:
        o_vision.setText("Pickup Step", "LOWERING_ARM_FULLY_BEFORE_GRAB");
        stopPickupDrive();
        o_subsystem.setMotorGear(0);
        o_subsystem.setServoLift(LIFT_FINAL_VIEW_ANGLE);
        o_subsystem.servo_Hook(CLAW_OPEN_ANGLE);
        if (!o_subsystem.moveServoArmToward(
            ARM_SCAN_DOWN_ANGLE, SERVO_ARM_STEP)) {
          pickupServoSettledSince = 0;
          return true;
        }
        if (pickupServoSettledSince == 0) {
          pickupServoSettledSince = now;
        } else if (now - pickupServoSettledSince
            >= ARM_LOWER_SETTLE_SEC) {
          pickupTrackedArmTarget = ARM_SCAN_DOWN_ANGLE;
          enterVisionPickupState(VisionPickupState.REACQUIRING_OBJECT);
        }
        return true;

      case REACQUIRING_OBJECT:
        o_vision.setText("Pickup Step", "SECOND_CAMERA_DETECTION");
        o_subsystem.setMotorGear(0);
        o_subsystem.setServoLift(LIFT_FINAL_VIEW_ANGLE);
        o_subsystem.servo_Hook_Hand(ARM_SCAN_DOWN_ANGLE);
        o_subsystem.servo_Hook(CLAW_OPEN_ANGLE);
        if (!targetVisible || !pickupFilterReady) {
          stopPickupDrive();
          pickupFilterReady = false;
          visionCameraCloseSince = 0;
          pickupCameraCloseFrames = 0;
          o_vision.setText("Pickup Warning", "WAITING_FOR_SECOND_VIEW");
          return true;
        }

        double finalOffset = pickupFilteredX;
        if (Math.abs(finalOffset) > VISION_CENTER_TOLERANCE_PX * 1.5) {
          visionCameraCloseSince = 0;
          pickupCameraCloseFrames = 0;
          o_vision.setText("Pickup Step", "SECOND_VIEW_CENTERING");
          double finalTurn = clampVision(
              finalOffset * VISION_TURN_KP,
              -VISION_MAX_TURN * 0.8,
              VISION_MAX_TURN * 0.8);
          setPickupDrive(finalTurn, -finalTurn, now);
          return true;
        }

        boolean finalGrabZone = Math.abs(pickupFilteredX)
                <= VISION_CENTER_TOLERANCE_PX * 1.5
            && pickupFilteredY >= FINAL_VIEW_GRAB_CENTER_Y_PX
            && pickupFilteredBottom >= FINAL_VIEW_GRAB_BOTTOM_Y_PX
            && pickupFilteredWidth >= FINAL_VIEW_MIN_WIDTH_PX;
        o_vision.setValue("Pickup Second Grab Zone", finalGrabZone ? 1.0 : 0.0);
        if (!finalGrabZone) {
          visionCameraCloseSince = 0;
          pickupCameraCloseFrames = 0;
          if (pickupFilteredBottom >= PICKUP_CAMERA_EDGE_STOP_BOTTOM_Y_PX) {
            stopPickupDrive();
            o_vision.setText("Pickup Warning", "SECOND_VIEW_AT_CAMERA_EDGE");
            return true;
          }

          // Keep the arm fully down and the claw open. Only the wheels move
          // during this second, camera-guided approach.
          double finalSlowdown = clampVision(
              (PICKUP_CAMERA_EDGE_STOP_BOTTOM_Y_PX - pickupFilteredBottom)
                  / 55.0,
              0.70, 1.0);
          double finalForward = FINAL_APPROACH_SPEED * finalSlowdown;
          double finalSteering = Math.abs(finalOffset)
                  <= VISION_CENTER_TOLERANCE_PX
              ? 0.0
              : clampVision(finalOffset * VISION_TURN_KP, -0.035, 0.035);
          o_vision.setText("Pickup Step", "SECOND_VIEW_APPROACHING_OBJECT");
          setPickupDrive(
              finalForward + finalSteering,
              finalForward - finalSteering,
              now);
          return true;
        }

        o_vision.setText("Pickup Step", "SECOND_VIEW_CONFIRMING_GRAB_ZONE");
        setPickupDrive(0, 0, now);
        if (Math.abs(pickupDriveLeft) >= 0.015
            || Math.abs(pickupDriveRight) >= 0.015) {
          visionCameraCloseSince = 0;
          pickupCameraCloseFrames = 0;
          o_vision.setText("Pickup Warning", "SECOND_VIEW_NOT_READY");
          return true;
        }
        if (visionCameraCloseSince == 0) {
          visionCameraCloseSince = now;
        }
        if (pickupFrameUpdated) {
          pickupCameraCloseFrames++;
        }
        if (pickupCameraCloseFrames >= FINAL_VIEW_CONFIRM_FRAMES
            && now - visionCameraCloseSince >= FINAL_VIEW_CONFIRM_SEC) {
          stopPickupDrive();
          o_vision.setText("Pickup Warning", "None");
          enterVisionPickupState(VisionPickupState.LOWERING_LIFT);
        }
        return true;

      case LOWERING_LIFT:
        o_vision.setText("Pickup Step", "LOWERING_LIFT_WITH_ARM_DOWN");
        stopPickupDrive();
        o_subsystem.setMotorGear(0);
        o_subsystem.servo_Hook(CLAW_OPEN_ANGLE);
        // The camera may lose the object while the arm points straight down;
        // the robot is stationary after a confirmed visual grab position.
        o_subsystem.servo_Hook_Hand(ARM_SCAN_DOWN_ANGLE);
        if (!o_subsystem.moveServoLiftToward(
            LIFT_PICKUP_ANGLE, PICKUP_LIFT_TRACK_STEP)) {
          pickupServoSettledSince = 0;
          return true;
        }
        if (pickupServoSettledSince == 0) {
          pickupServoSettledSince = now;
        } else if (now - pickupServoSettledSince
            >= LIFT_LOWER_SETTLE_SEC) {
          enterVisionPickupState(VisionPickupState.GRABBING);
        }
        return true;

      case GRABBING:
        o_vision.setText("Pickup Step", "CLOSING_CLAW");
        stopPickupDrive();
        o_subsystem.setMotorGear(0);
        o_subsystem.setServoLift(LIFT_PICKUP_ANGLE);
        o_subsystem.servo_Hook_Hand(ARM_SCAN_DOWN_ANGLE);
        if (!o_subsystem.moveServoClawToward(
            CLAW_CLOSED_ANGLE, SERVO_CLAW_STEP)) {
          pickupServoSettledSince = 0;
          return true;
        }
        if (pickupServoSettledSince == 0) {
          pickupServoSettledSince = now;
          o_vision.setText("Pickup Step", "HOLDING_OBJECT");
        } else if (now - pickupServoSettledSince >= VISION_GRIP_DELAY_SEC) {
          enterVisionPickupState(VisionPickupState.RAISING_LIFT);
        }
        return true;

      case RAISING_LIFT:
        o_vision.setText("Pickup Step", "RAISING_LIFT_WITH_OBJECT");
        stopPickupDrive();
        o_subsystem.setMotorGear(0);
        o_subsystem.servo_Hook(CLAW_CLOSED_ANGLE);
        o_subsystem.servo_Hook_Hand(ARM_SCAN_DOWN_ANGLE);
        if (!o_subsystem.moveServoLiftToward(
            LIFT_SEARCH_ANGLE, SERVO_LIFT_STEP)) {
          pickupServoSettledSince = 0;
          return true;
        }
        if (pickupServoSettledSince == 0) {
          pickupServoSettledSince = now;
        } else if (now - pickupServoSettledSince >= LIFT_RAISE_SETTLE_SEC) {
          enterVisionPickupState(VisionPickupState.RAISING_ARM);
        }
        return true;

      case RAISING_ARM:
        o_vision.setText("Pickup Step", "RAISING_CAMERA_AFTER_PICKUP");
        stopPickupDrive();
        o_subsystem.setMotorGear(0);
        o_subsystem.setServoLift(LIFT_SEARCH_ANGLE);
        o_subsystem.servo_Hook(CLAW_CLOSED_ANGLE);
        if (!o_subsystem.moveServoArmToward(
            ARM_SEARCH_ANGLE, SERVO_ARM_STEP)) {
          pickupServoSettledSince = 0;
          return true;
        }
        if (pickupServoSettledSince == 0) {
          pickupServoSettledSince = now;
        } else if (now - pickupServoSettledSince >= LIFT_RAISE_SETTLE_SEC) {
          visionPickupComplete = true;
          enterVisionPickupState(VisionPickupState.COMPLETE);
        }
        return true;

      case COMPLETE:
        stopPickupDrive();
        o_subsystem.setMotorGear(0);
        o_subsystem.setServoLift(LIFT_SEARCH_ANGLE);
        o_subsystem.servo_Hook(CLAW_CLOSED_ANGLE);
        o_subsystem.servo_Hook_Hand(ARM_SEARCH_ANGLE);
        return false;

      default:
        return false;
    }
  }

  private boolean isPickupZoneReady() {
    boolean ready = Math.abs(pickupFilteredX)
            <= VISION_CENTER_TOLERANCE_PX * 1.5
        && pickupFilteredY >= PICKUP_ZONE_MIN_CENTER_Y_PX
        && pickupFilteredBottom >= PICKUP_ZONE_MIN_BOTTOM_Y_PX;
    o_vision.setValue("Pickup In Grab Zone", ready ? 1.0 : 0.0);
    return ready;
  }

  private void updatePickupVisionFilter() {
    double frameTimestamp = o_vision.getVisionFrameTimestamp();
    pickupFrameUpdated = frameTimestamp != pickupLastFrameTimestamp;
    if (!pickupFrameUpdated) {
      return;
    }
    pickupLastFrameTimestamp = frameTimestamp;
    double x = o_vision.getVisionOffsetX();
    double y = o_vision.getVisionTargetY();
    double width = o_vision.getVisionObjectWidth();
    double bottom = o_vision.getVisionTargetBottomY();
    if (!pickupFilterReady) {
      pickupFilteredX = x;
      pickupFilteredY = y;
      pickupFilteredWidth = width;
      pickupFilteredBottom = bottom;
      pickupFilterReady = true;
    } else {
      pickupFilteredX += PICKUP_FILTER_ALPHA * (x - pickupFilteredX);
      pickupFilteredY += PICKUP_FILTER_ALPHA * (y - pickupFilteredY);
      pickupFilteredWidth += PICKUP_FILTER_ALPHA
          * (width - pickupFilteredWidth);
      pickupFilteredBottom += PICKUP_FILTER_ALPHA
          * (bottom - pickupFilteredBottom);
    }
    o_vision.setValue("Pickup Filtered X", pickupFilteredX);
    o_vision.setValue("Pickup Filtered Y", pickupFilteredY);
    o_vision.setValue("Pickup Filtered Bottom", pickupFilteredBottom);
    isPickupZoneReady();
  }

  private void setPickupDrive(double left, double right, double now) {
    double dt = pickupDriveLastAt == 0
        ? 0.02 : clampVision(now - pickupDriveLastAt, 0.0, 0.05);
    double leftRate = Math.abs(left) < Math.abs(pickupDriveLeft)
        ? PICKUP_DRIVE_DECEL_PER_SEC : PICKUP_DRIVE_ACCEL_PER_SEC;
    double rightRate = Math.abs(right) < Math.abs(pickupDriveRight)
        ? PICKUP_DRIVE_DECEL_PER_SEC : PICKUP_DRIVE_ACCEL_PER_SEC;
    pickupDriveLeft += clampVision(
        left - pickupDriveLeft, -leftRate * dt, leftRate * dt);
    pickupDriveRight += clampVision(
        right - pickupDriveRight, -rightRate * dt, rightRate * dt);
    pickupDriveLastAt = now;
    o_subsystem.setDrivePower(pickupDriveLeft, pickupDriveRight);
  }

  private void stopPickupDrive() {
    pickupDriveLeft = 0;
    pickupDriveRight = 0;
    pickupDriveLastAt = Timer.getFPGATimestamp();
    o_subsystem.stopDrive();
  }

  private void scanPickupCamera(double now) {
    boolean atScanAngle = o_subsystem.moveServoArmToward(
        pickupScanArmTarget, SERVO_ARM_STEP);
    o_vision.setValue("Pickup Arm Target", pickupScanArmTarget);
    if (!atScanAngle || now < pickupScanNextStepAt) {
      return;
    }

    double next = pickupScanArmTarget
        + pickupScanDirection * PICKUP_SCAN_STEP_DEG;
    if (next >= ARM_SCAN_DOWN_ANGLE) {
      pickupScanArmTarget = ARM_SCAN_DOWN_ANGLE;
      pickupScanDirection = -1;
    } else if (next <= ARM_SEARCH_ANGLE) {
      pickupScanArmTarget = ARM_SEARCH_ANGLE;
      pickupScanDirection = 1;
    } else {
      pickupScanArmTarget = next;
    }
    pickupScanNextStepAt = now + PICKUP_SCAN_DWELL_SEC;
  }

  private void followPickupArm(double now) {
    if (pickupFrameUpdated
        && now - pickupLastArmUpdateAt >= PICKUP_ARM_UPDATE_SEC) {
      double imageError = pickupFilteredY
          - PICKUP_IMAGE_CENTER_Y_PX;
      if (Math.abs(imageError) > PICKUP_IMAGE_Y_DEADBAND_PX) {
        double correction = clampVision(
            imageError * PICKUP_ARM_KP,
            -PICKUP_ARM_MAX_CORRECTION_DEG,
            PICKUP_ARM_MAX_CORRECTION_DEG);
        pickupTrackedArmTarget = clampVision(
            pickupTrackedArmTarget + correction,
            ARM_SEARCH_ANGLE,
            ARM_SCAN_DOWN_ANGLE);
      }
      pickupLastArmUpdateAt = now;
    }
    o_subsystem.moveServoArmToward(
        pickupTrackedArmTarget, SERVO_ARM_STEP);
    o_vision.setValue("Pickup Arm Target", pickupTrackedArmTarget);
  }

  private void followPickupTarget(double now) {
    followPickupArm(now);
    // Approach the floor gradually as the object gets larger/lower in the
    // frame. The final descent is handled after the 5 cm sonar stop.
    double widthProgress = clampVision(
        (pickupFilteredWidth - 45.0)
            / (VISION_CAMERA_CLOSE_WIDTH_PX - 45.0),
        0.0, 1.0);
    double bottomProgress = clampVision(
        (pickupFilteredBottom - 150.0)
            / (PICKUP_CAMERA_EDGE_STOP_BOTTOM_Y_PX - 150.0),
        0.0, 1.0);
    double progress = Math.max(widthProgress, bottomProgress);
    double liftTarget = LIFT_SEARCH_ANGLE
        + (LIFT_PICKUP_ANGLE - LIFT_SEARCH_ANGLE) * 0.70 * progress;
    pickupTrackedLiftTarget = Math.max(pickupTrackedLiftTarget, liftTarget);
    o_subsystem.moveServoLiftToward(
        pickupTrackedLiftTarget, PICKUP_LIFT_TRACK_STEP);
    o_vision.setValue("Pickup Lift Target", pickupTrackedLiftTarget);
  }

  private boolean keepVisionTargetOrWait(double now, boolean targetVisible) {
    if (targetVisible) {
      return true;
    }

    stopPickupDrive();
    pickupFilterReady = false;
    visionCenteredSince = 0;
    visionCameraCloseSince = 0;
    pickupCameraCloseFrames = 0;
    if (now - visionTargetLastSeen > VISION_TARGET_LOST_TIMEOUT_SEC) {
      enterVisionPickupState(VisionPickupState.IDLE);
    }
    return false;
  }

  private double clampVision(double value, double minimum, double maximum) {
    return Math.max(minimum, Math.min(maximum, value));
  }

  // ─── Сортировка ───────────────────────────────────────────────────────────

  public void resetSortCounters() {
    for (String key : ballCount.keySet()) ballCount.put(key, 0);
  }

  private int getServoForColor(String detected) {
    for (String color : ballCount.keySet()) {
      if (detected.contains(color)) {
        int count = ballCount.get(color);
        ballCount.put(color, count + 1);
        return (count < 2) ? SERVO_LEFT : SERVO_RIGHT;
      }
    }
    return SERVO_RIGHT;
  }

  public void sortBall() {
    switch (sortState) {
      case 0:
        o_subsystem.setServoLift(SERVO_CENTER);
        o_vision.setMode(VisionSubsystem.MODE_COLOR);
        o_vision.setROI(0.0, 0.10, 0.45, 0.65);
        o_vision.setProcessed(true);
        sortState = 1;
        break;
      case 1:
        if (o_vision.isVisionFound()) {
          sortTimer = Timer.getFPGATimestamp();
          sortState = 2;
        }
        break;
      case 2:
        if (Timer.getFPGATimestamp() - sortTimer > 0.3) {
          String detected = o_vision.getCurrentDetection();
          int servoPos = getServoForColor(detected);
          o_subsystem.setServoLift(servoPos);
          sortTimer = Timer.getFPGATimestamp();
          sortState = 3;
        }
        break;
      case 3:
        if (Timer.getFPGATimestamp() - sortTimer > 0.5) {
          o_subsystem.setServoLift(SERVO_CENTER);
          sortTimer = Timer.getFPGATimestamp();
          sortState = 4;
        }
        break;
      case 4:
        if (Timer.getFPGATimestamp() - sortTimer > 0.3) {
          sortState = 1;
        }
        break;
    }
  }

  // ─── Движение ─────────────────────────────────────────────────────────────

  private double normalizeYaw(double yaw) {
    return (yaw % 360 + 360) % 360;
  }

  private double headingError() {
    double error = normalizeYaw(angleRobot) - normalizeYaw(o_subsystem.getYaw());
    if (error > 180) error -= 360;
    if (error < -180) error += 360;
    return error;
  }

  public void rotateTheRobot(double targetDeg) {
    double currentAngle = normalizeYaw(o_subsystem.getYaw());
    double speed = 0.3;

    double error = targetDeg - currentAngle;
    if (error > 180)  error -= 360;
    if (error < -180) error += 360;

    if (Math.abs(error) > 0.15) {
      if (Math.abs(error) < 10) speed = 0.15;
      if (error > 0) {
        o_subsystem.setDrivePower(speed / 2, -speed / 2);
      } else {
        o_subsystem.setDrivePower(-speed, speed);
      }
    } else {
      o_subsystem.TimerStop();
      angleRobot = targetDeg;
      stateAutomatic++;
    }
  }

  public void goForwardDistance(double dist) {
    if (dist <= 0) {
      o_subsystem.TimerStop();
      stateAutomatic++;
      return;
    }

    double travelledMm = o_subsystem.getAverageDriveDistance();

    double lSpeed = maxSpeed;
    double rSpeed = maxSpeed;
    double targetMm = dist * 11.1;

    if (travelledMm >= targetMm) {
      o_subsystem.TimerStop();
      stateAutomatic++;
    } else {
      if (targetMm - travelledMm <= 100.0) {
        lSpeed = minSpeed;
        rSpeed = minSpeed;
      }
      double currentAngle = normalizeYaw(o_subsystem.getYaw());
      double angleNorm = normalizeYaw(angleRobot);
      double error = angleNorm - currentAngle;
      if (error > 180)  error -= 360;
      if (error < -180) error += 360;
      if (error > 0)       rSpeed -= error / 90.0;
      else if (error < 0)  lSpeed += error / 90.0;
      o_subsystem.setDrivePower(lSpeed, rSpeed);
    }
  }

  public void goBackDistance(double dist) {
    if (dist <= 0) {
      o_subsystem.TimerStop();
      stateAutomatic++;
      return;
    }

    double travelledMm = -o_subsystem.getAverageDriveDistance();

    double lSpeed = maxSpeed;
    double rSpeed = maxSpeed;
    double targetMm = dist * 11.1;

    if (travelledMm >= targetMm) {
      o_subsystem.TimerStop();
      stateAutomatic++;
    } else {
      if (targetMm - travelledMm <= 100.0) {
        lSpeed = minSpeed;
        rSpeed = minSpeed;
      }
      double currentAngle = normalizeYaw(o_subsystem.getYaw());
      double angleNorm = normalizeYaw(angleRobot);
      double error = angleNorm - currentAngle;
      if (error > 180)  error -= 360;
      if (error < -180) error += 360;
      if (error > 0)       lSpeed -= error / 90.0;
      else if (error < 0)  rSpeed += error / 90.0;
      o_subsystem.setDrivePower(-lSpeed, -rSpeed);
    }
  }

  public void goForwardSharp(double dist) {
    if (dist <= 0) {
      o_subsystem.stopAllMotors();
      stateAutomatic++;
      return;
    }

    double lSpeed = 0.3;
    double rSpeed = 0.3;
    double currentAngle = normalizeYaw(o_subsystem.getYaw());
    double angleNorm = normalizeYaw(angleRobot);
    double error = angleNorm - currentAngle;
    if (error > 180)  error -= 360;
    if (error < -180) error += 360;
    if (error > 0)       rSpeed -= error / 90.0;
    else if (error < 0)  lSpeed += error / 90.0;
    double slowdown_factor = Math.max(0, Math.min(1.0, (o_subsystem.getForwardSharp() - dist + 10) / dist));
    o_subsystem.setDrivePower(
        lSpeed * slowdown_factor,
        rSpeed * slowdown_factor);
    if (o_subsystem.getForwardSharp() < dist) {
      o_subsystem.TimerStop();
      stateAutomatic++;
    }
  }

  public void goBackSharp(double dist) {
    if (dist <= 0) {
      o_subsystem.stopAllMotors();
      stateAutomatic++;
      return;
    }

    double lSpeed = 0.3;
    double rSpeed = 0.3;
    double currentAngle = normalizeYaw(o_subsystem.getYaw());
    double angleNorm = normalizeYaw(angleRobot);
    double error = angleNorm - currentAngle;
    if (error > 180)  error -= 360;
    if (error < -180) error += 360;
    if (error > 0)       lSpeed -= error / 90.0;
    else if (error < 0)  rSpeed += error / 90.0;
    double slowdown_factor = Math.max(0, Math.min(1.0, (o_subsystem.getBackSharp() - dist + 10) / dist));
    o_subsystem.setDrivePower(
        -lSpeed * slowdown_factor,
        -rSpeed * slowdown_factor);
    if (o_subsystem.getBackSharp() < dist) {
      o_subsystem.TimerStop();
      stateAutomatic++;
    }
  }

  public void goLeftSonic(double dist) {
    goForwardSonic(dist);
  }

  public void goForwardSonic(double dist) {
    if (dist <= 0) {
      o_subsystem.stopAllMotors();
      stateAutomatic++;
      return;
    }

    double sonic = o_subsystem.getDistanceSonicFront();
    if (!Double.isFinite(sonic)) {
      o_subsystem.stopDrive();
      return;
    }

    if (sonic <= dist) {
      o_subsystem.TimerStop();
      stateAutomatic++;
      return;
    }

    double speed = sonic - dist <= 11.1 ? minSpeed : maxSpeed / 2.0;
    double leftSpeed = speed;
    double rightSpeed = speed;
    double error = headingError();
    if (error > 0) rightSpeed -= Math.min(0.15, error / 90.0);
    else if (error < 0) leftSpeed -= Math.min(0.15, -error / 90.0);

    o_subsystem.setDrivePower(leftSpeed, rightSpeed);
  }

  public void goRightSonic(double dist) {
    goBackSonic(dist);
  }

  public void goBackSonic(double dist) {
    if (dist <= 0) {
      o_subsystem.stopAllMotors();
      stateAutomatic++;
      return;
    }

    double sonic = o_subsystem.getDistanceSonicRear();
    if (!Double.isFinite(sonic)) {
      o_subsystem.stopDrive();
      return;
    }

    if (sonic <= dist) {
      o_subsystem.TimerStop();
      stateAutomatic++;
      return;
    }

    double speed = sonic - dist <= 11.1 ? minSpeed : maxSpeed / 2.0;
    double leftSpeed = speed;
    double rightSpeed = speed;
    double error = headingError();
    if (error > 0) leftSpeed -= Math.min(0.15, error / 90.0);
    else if (error < 0) rightSpeed -= Math.min(0.15, -error / 90.0);

    o_subsystem.setDrivePower(-leftSpeed, -rightSpeed);
  }

  public void goForwardWithBackSharp(double dist) {
    double lSpeed = maxSpeed;
    double rSpeed = maxSpeed;
    double currentAngle = normalizeYaw(o_subsystem.getYaw());
    double angleNorm = normalizeYaw(angleRobot);
    double error = angleNorm - currentAngle;
    if (error > 180)  error -= 360;
    if (error < -180) error += 360;
    if (error > 0)       rSpeed -= error / 90.0;
    else if (error < 0)  lSpeed += error / 90.0;
    if (o_subsystem.getBackSharp() >= dist - 15) {
      lSpeed = 0.15;
      rSpeed = 0.15;
    }
    o_subsystem.setDrivePower(lSpeed, rSpeed);
    if (o_subsystem.getBackSharp() >= dist) {
      o_subsystem.TimerStop();
      stateAutomatic++;
    }
  }

  public void goLeftWithRightSonic(double dist) {
    double sonic = o_subsystem.getDistanceSonicRear();
    double lSpeed = maxSpeed / 2;
    double rSpeed = maxSpeed / 2;
    if (sonic > dist - 10) {
      lSpeed = maxSpeed / 2;
      rSpeed = maxSpeed / 2;
    }
    o_subsystem.setDrivePower(lSpeed, rSpeed);
    if (sonic >= dist) {
      o_subsystem.TimerStop();
      stateAutomatic++;
    }
  }

  public void goRigtWithLeftSonic(double dist) {
    double sonic = o_subsystem.getDistanceSonicFront();
    double lSpeed = maxSpeed / 2;
    double rSpeed = maxSpeed / 2;
    if (sonic > dist - 10) {
      lSpeed = maxSpeed * 0.25;
      rSpeed = maxSpeed * 0.25;
    }
    o_subsystem.setDrivePower(-lSpeed, -rSpeed);
    if (sonic > dist) {
      o_subsystem.TimerStop();
      stateAutomatic++;
    }
  }

  public void alignToVision(double offsetX) {
    double speed = 0.2;
    double tolerance = 10;
    double leftSpeed = 0;
    double rightSpeed = 0;
    if (Math.abs(offsetX) > tolerance) {
      if (offsetX > 0) {
        leftSpeed  = speed;
        rightSpeed = -speed;
      } else {
        leftSpeed  = -speed;
        rightSpeed = speed;
      }
    } else {
      o_subsystem.TimerStop();
      stateAutomatic++;
      return;
    }
    o_subsystem.setDrivePower(leftSpeed, rightSpeed);
  }

  // ─── Захват ───────────────────────────────────────────────────────────────

  public void open_Hand() {
    o_subsystem.servo_Hook(CLAW_OPEN_ANGLE);
  }

  public void closed_Hand_Kub() {
    o_subsystem.servo_Hook(CLAW_CLOSED_ANGLE);
  }

  private String driveState = "forward";

  private static final double BBOX_CLOSE_RATIO = 0.95;
  private static final int    SERVO_START      = 165;
  private static final int    SERVO_MAX_PUSH   = 30;
  private static final double DEAD_ZONE        = 0.05;

  private int lastServoValue = SERVO_START;
  private static final int SERVO_STEP = 3;

  public void pushCameraToObject() {
    double objectWidth = o_vision.getVisionObjectWidth();
    double frameWidth  = 320.0;
    double ratio       = objectWidth / frameWidth;

    if (objectWidth == 0 || !o_vision.isVisionFound()) {
      lastServoValue = SERVO_START;
      o_subsystem.servo_Hook_Hand(SERVO_START);
      return;
    }

    if (ratio >= (BBOX_CLOSE_RATIO - 0.25)) {
      closed_Hand_Kub();
      return;
    }

    double brakeZone = BBOX_CLOSE_RATIO * 0.7;
    int targetValue;
    if (ratio < brakeZone) {
      targetValue = SERVO_MAX_PUSH;
    } else {
      double t = (ratio - brakeZone) / (BBOX_CLOSE_RATIO - brakeZone);
      targetValue = (int)(SERVO_MAX_PUSH + (SERVO_START - SERVO_MAX_PUSH) * t);
    }

    targetValue = Math.max(SERVO_MAX_PUSH, Math.min(SERVO_START, targetValue));

    if (lastServoValue > targetValue)      lastServoValue = Math.max(lastServoValue - SERVO_STEP, targetValue);
    else if (lastServoValue < targetValue) lastServoValue = Math.min(lastServoValue + SERVO_STEP, targetValue);

    o_subsystem.servo_Hook_Hand(lastServoValue);
  }

  public void autoRobotics() {
    double sharpForward = o_subsystem.getForwardSharp();
    double sonicFront   = o_subsystem.getDistanceSonicFront();
    double sonicRear    = o_subsystem.getDistanceSonicRear();
    double currentYaw   = normalizeYaw(o_subsystem.getYaw());

    if (driveState.equals("forward")) {
      if (sharpForward > 15.0) {
        double baseSpeed = 0.28;
        double lSpeed = baseSpeed;
        double rSpeed = baseSpeed;
        double angleNorm = normalizeYaw(angleRobot);
        if (currentYaw < angleNorm - 0.5) {
          double angleDifference = angleNorm - currentYaw;
          rSpeed -= angleDifference / 80.0;
        } else if (currentYaw > angleNorm + 0.5) {
          double angleDifference = currentYaw - angleNorm;
          lSpeed -= angleDifference / 80.0;
        }
        lSpeed = Math.max(0.14, Math.min(0.50, lSpeed));
        rSpeed = Math.max(0.14, Math.min(0.50, rSpeed));
        o_subsystem.setDrivePower(lSpeed, rSpeed);
      } else {
        o_subsystem.stopDrive();
        double turnDegrees = 90.0;
        if (sonicFront < 28 && sonicRear < 28) turnDegrees = 180.0;
        targetAngle = normalizeYaw(currentYaw + turnDegrees);
        angleRobot = targetAngle;
        driveState = "turning";
      }
    }

    if (driveState.equals("turning")) {
      double error = ((targetAngle - currentYaw + 180) % 360) - 180;
      if (Math.abs(error) > 1.8) {
        double rotateSpeed = error * 0.0042;
        if (Math.abs(rotateSpeed) < 0.15) rotateSpeed = Math.signum(error) * 0.15;
        rotateSpeed = Math.max(-0.45, Math.min(0.45, rotateSpeed));
        o_subsystem.setDrivePower(rotateSpeed, -rotateSpeed);
      } else {
        o_subsystem.stopDrive();
        driveState = "forward";
      }
    }
  }

  @Override
  public void end(final boolean interrupted) {
    o_subsystem.stopAllMotors();
    o_vision.setModeButtonEnabled(true);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
