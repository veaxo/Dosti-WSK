package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.util.RobotDiagnostics;
import frc.robot.util.ThreeWheelDrive;
import frc.robot.util.VisionPickupGate;
import frc.robot.util.PickupGeometry;
import frc.robot.util.PickupMotion;
import frc.robot.util.PickupApproachGuard;
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
  private boolean firstExecute = true;
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
    ALIGNING_AT_PICKUP_HEIGHT,
    GRABBING,
    RAISING_LIFT,
    RAISING_ARM,
    COMPLETE,
    FAILED
  }

  private static final double VISION_CENTER_TOLERANCE_PX = 12.0;
  private static final double PICKUP_FRAME_MAX_AGE_SEC = 0.35;
  private static final double PICKUP_FILTER_ALPHA = 0.25;
  private static final double VISION_TARGET_LOST_TIMEOUT_SEC = 0.75;
  private static final double VISION_CENTER_HOLD_SEC = 0.25;
  private static final double VISION_TARGET_CONFIRM_SEC = 0.15;
  // First view only prepares the second camera pose. Scan-angle-dependent Y
  // cannot establish distance or authorize closing the claw.
  private static final double PICKUP_CAMERA_GRAB_WIDTH_PX = PickupGeometry.FIRST_WIDTH;
  // Stop before the object becomes clipped; this boundary never authorizes grip.
  private static final double PICKUP_CAMERA_EDGE_STOP_BOTTOM_Y_PX = PickupGeometry.EDGE_BOTTOM_Y;
  private static final double VISION_GRIP_DELAY_SEC = 1.50;
  private static final double SERVO_CALIBRATION_SETTLE_SEC = 1.50;
  private static final double LIFT_LOWER_SETTLE_SEC = 0.25;
  private static final double ARM_LOWER_SETTLE_SEC = 0.50;
  private static final double FINAL_VIEW_CONFIRM_SEC = 0.30;
  private static final int FINAL_VIEW_CONFIRM_FRAMES = 3;
  private static final double FINAL_APPROACH_SPEED = 0.12;
  // Second view: confirm size and center while the whole object is still visible.
  private static final double FINAL_VIEW_GRAB_CENTER_Y_PX = 170.0;
  // The bottom edge is a safety boundary, not a goal to push the object toward.
  private static final double FINAL_VIEW_GRAB_BOTTOM_Y_PX = FINAL_VIEW_GRAB_CENTER_Y_PX;
  private static final double FINAL_VIEW_MIN_WIDTH_PX = 110.0;
  // Recheck after lowering the lift: the old elevated-camera image is invalid.
  private static final double PICKUP_HEIGHT_GRAB_CENTER_Y_PX = 175.0;
  private static final double PICKUP_HEIGHT_GRAB_BOTTOM_Y_PX = PICKUP_HEIGHT_GRAB_CENTER_Y_PX;
  private static final double PICKUP_HEIGHT_MIN_WIDTH_PX = 120.0;
  private static final double PICKUP_HEIGHT_APPROACH_SPEED = 0.08;
  private static final double PICKUP_MAX_STRAFE = 0.08;
  private static final double LIFT_RAISE_SETTLE_SEC = 0.35;
  private static final double SERVO_LIFT_STEP = 0.8;
  private static final double PICKUP_LIFT_TRACK_STEP = 0.4;
  private static final double SERVO_ARM_STEP = 0.8;
  private static final double SERVO_CLAW_STEP = 2.0;
  private static final double VISION_TURN_KP = 0.004;
  private static final double VISION_MAX_TURN = 0.04;
  // Working pickup commit 562bf2b used 0.12 -> 0.08, never a 0.02 crawl.
  private static final double VISION_APPROACH_SPEED = 0.08;
  private static final double VISION_APPROACH_SLOW_SPEED = 0.06;
  private static final double PICKUP_SCAN_STEP_DEG = 5.0;
  private static final double PICKUP_SCAN_DWELL_SEC = 0.45;
  // The camera pose remains fixed while driving; adjustments require a stop.
  private static final double PICKUP_IMAGE_CENTER_Y_PX = 170.0;
  // User-calibrated physical positions. Do not infer direction from the
  // numeric angle or swap these values automatically.
  private static final int CLAW_OPEN_ANGLE = 18;
  private static final int CLAW_CLOSED_ANGLE = 170;
  // 80 degrees looks forward; increasing the angle aims down at the floor.
  // The arm servo's 15-degree safety margin clamps its 180-degree endpoint
  // to 165 degrees in ExampleSubsystem.
  private static final double ARM_SEARCH_ANGLE = 80.0;
  private static final double ARM_SCAN_DOWN_ANGLE = 165.0;
  private static final int LIFT_PICKUP_ANGLE = 130;
  private static final int LIFT_SEARCH_ANGLE = 50;
  // Lift position during the second camera-guided approach; tune separately.
  private static final int LIFT_FINAL_VIEW_ANGLE = 15;

  private VisionPickupState visionPickupState = VisionPickupState.IDLE;
  private double visionPickupStateStart = 0;
  private double visionTargetLastSeen = 0;
  private double visionTargetSeenSince = 0;
  private final VisionPickupGate pickupPositionGate = new VisionPickupGate(
      FINAL_VIEW_CONFIRM_FRAMES, FINAL_VIEW_CONFIRM_SEC);
  private final VisionPickupGate pickupCenterGate = new VisionPickupGate(4, VISION_CENTER_HOLD_SEC);
  private final PickupMotion pickupMotion = new PickupMotion();
  private final PickupApproachGuard pickupApproachGuard = new PickupApproachGuard();
  private double pickupServoCycleTime = Double.NaN;
  private double pickupServoDt = 0.02;
  private boolean pickupGripStarted;
  private double pickupPoseReadyTimestamp;
  private double pickupScanArmTarget = ARM_SEARCH_ANGLE;
  private int pickupScanDirection = 1;
  private double pickupScanNextStepAt = 0;
  private double pickupTrackedArmTarget = ARM_SEARCH_ANGLE;
  private double pickupLastFrameTimestamp = 0;
  private boolean pickupFrameUpdated = false;
  private boolean pickupFilterReady = false;
  private double pickupFilteredX = 0;
  private double pickupFilteredY = PICKUP_IMAGE_CENTER_Y_PX;
  private double pickupFilteredWidth = 0;
  private double pickupFilteredBottom = PICKUP_IMAGE_CENTER_Y_PX;
  private double pickupDriveLeft = 0;
  private double pickupDriveRight = 0;
  private double pickupDriveStrafe = 0;
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
    o_subsystem.setServoLift(15);
    firstExecute = true;
    RobotDiagnostics.stage("DriveMotor.initialize: disabling vision mode button");
    // During autonomous the physical Start button starts this command. Do not
    // let the same press also toggle camera ownership between Python and Java.
    o_vision.setModeButtonEnabled(false);
    RobotDiagnostics.stage("DriveMotor.initialize: resetting camera scan");
    o_vision.resetCameraScan();
    // o_subsystem.servo_Hook(300);
    RobotDiagnostics.stage("DriveMotor.initialize: resetting navX yaw");
    o_subsystem.resetYaw();
    RobotDiagnostics.stage("DriveMotor.initialize: resetting Titan encoders");
    o_subsystem.resetEncoder();
    RobotDiagnostics.stage("DriveMotor.initialize: setting button LEDs");
    o_subsystem.setButtonLed("Running", false);
    o_subsystem.setButtonLed("Stopped", true);
    o_subsystem.setButtonLed("Start",   true);
    o_subsystem.setButtonLed("Stop",    true);
    RobotDiagnostics.stage("DriveMotor.initialize: reading yaw");
    angleRobot = normalizeYaw(o_subsystem.getYaw()); 
    previousRobotStart = false;
    RobotDiagnostics.stage("DriveMotor.initialize: resetting pickup state");
    resetVisionPickup();
    resetVisionRecognition();
    resetTimedActions();
    RobotDiagnostics.stage("DriveMotor.initialize completed");
  }

  @Override
  public void execute() {
    if (firstExecute) RobotDiagnostics.stage("DriveMotor first execute: reading buttons");
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
        visionPickup();
        // goForwardDistance(100);
          // rotateTheRobot(90);
          break;
        case 1:
          // goBackSonic(6);
          break;
        case 2:
          // rotateTheRobot(0);
          break;
        case 3:
          // goForwardDistance(60);
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
    if (firstExecute) {
      firstExecute = false;
      RobotDiagnostics.stage("DriveMotor first execute completed; start="
          + start + "; case=" + stateAutomatic);
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
    pickupScanArmTarget = ARM_SEARCH_ANGLE;
    pickupScanDirection = 1;
    pickupScanNextStepAt = 0;
    pickupTrackedArmTarget = ARM_SEARCH_ANGLE;
    pickupLastFrameTimestamp = 0;
    pickupFrameUpdated = false;
    pickupFilterReady = false;
    pickupDriveLeft = 0;
    pickupDriveRight = 0;
    pickupDriveStrafe = 0;
    pickupGripStarted = false;
    pickupCandidateColor = null;
    pickupLockedColor = null;
    pickupServoStage = 0;
    pickupServoSettledSince = 0;
    pickupPositionGate.reset(o_vision.getVisionFrameTimestamp());
    pickupCenterGate.reset(o_vision.getVisionFrameTimestamp());
    pickupServoCycleTime = Double.NaN;
    pickupMotion.reset(Timer.getFPGATimestamp());
    visionPickupComplete = false;
    visionPickupCase = -1;
    o_vision.setText("Pickup State", visionPickupState.name());
    o_vision.setText("Pickup Error", "None");
    o_vision.setText("Pickup Failed Stage", "None");
    o_vision.setText("Pickup Warning", "None");
    o_vision.setValue("Pickup In Grab Zone", 0);
    o_vision.setValue("Pickup Second Grab Zone", 0);
    o_vision.setValue("Pickup Final Grab Zone", 0);
    o_vision.setValue("Pickup Strafe", 0);
  }

  private boolean isPickupObjectVisible() {
    if (!o_vision.isColorCameraReady()
        || !o_vision.isVisionFound()
        || !o_vision.isVisionFrameRecent(PICKUP_FRAME_MAX_AGE_SEC)) {
      return false;
    }
    if ((visionPickupState == VisionPickupState.REACQUIRING_OBJECT
        || visionPickupState == VisionPickupState.ALIGNING_AT_PICKUP_HEIGHT)
        && o_vision.getVisionFrameTimestamp() <= pickupPoseReadyTimestamp) {
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
        || pickupLockedColor.equalsIgnoreCase(o_vision.getVisionColorIdentity()));
  }

  private void enterVisionPickupState(VisionPickupState state) {
    visionPickupState = state;
    visionPickupStateStart = Timer.getFPGATimestamp();
    pickupServoSettledSince = 0;
    pickupPositionGate.reset(o_vision.getVisionFrameTimestamp());
    pickupCenterGate.reset(o_vision.getVisionFrameTimestamp());
    pickupServoCycleTime = Double.NaN;
    pickupMotion.reset(Timer.getFPGATimestamp());
    if (state != VisionPickupState.REACQUIRING_OBJECT) {
      o_vision.setValue("Pickup Second Grab Zone", 0);
    }
    if (state != VisionPickupState.ALIGNING_AT_PICKUP_HEIGHT
        && state != VisionPickupState.GRABBING) {
      o_vision.setValue("Pickup Final Grab Zone", 0);
    }
    if (state == VisionPickupState.IDLE) {
      visionTargetSeenSince = 0;
      pickupCandidateColor = null;
      pickupLockedColor = null;
      pickupScanArmTarget = ARM_SEARCH_ANGLE;
      pickupScanDirection = 1;
      pickupScanNextStepAt = visionPickupStateStart + PICKUP_SCAN_DWELL_SEC;
      pickupFilterReady = false;
      }
    if (state == VisionPickupState.REACQUIRING_OBJECT
        || state == VisionPickupState.ALIGNING_AT_PICKUP_HEIGHT) {
      // Require a new image after changing the camera pose. The old frame
      // describes the first approach and cannot authorize the final grip.
      pickupFilterReady = false;
      pickupLastFrameTimestamp = o_vision.getVisionFrameTimestamp();
      pickupFrameUpdated = false;
      visionTargetSeenSince = 0;
      pickupPoseReadyTimestamp = System.currentTimeMillis() / 1000.0;
    }
    if (state == VisionPickupState.RAISING_FOR_SEARCH) {
      pickupServoStage = 0;
    }
    if (state == VisionPickupState.APPROACHING
        || state == VisionPickupState.REACQUIRING_OBJECT
        || state == VisionPickupState.ALIGNING_AT_PICKUP_HEIGHT) {
      pickupApproachGuard.reset(visionPickupStateStart,
          state == VisionPickupState.APPROACHING ? o_vision.getVisionObjectWidth() : 0.0,
          state == VisionPickupState.APPROACHING ? 12.0 : 3.0);
    }
    if (state == VisionPickupState.GRABBING) pickupGripStarted = false;
    o_vision.setText("Pickup State", state.name());
  }

  /**
   * Call this method from an autonomous switch case. It waits for a colored
   * object, performs the complete pickup, then advances stateAutomatic once.
   */
  public void visionPickup() {
    if (visionPickupCase != stateAutomatic) {
      resetVisionPickup();
      visionPickupCase = stateAutomatic;
      stopPickupDrive();
      // A floor object moves to the bottom edge of the image as the robot
      // approaches. The cropped scan ROI could therefore hide it immediately
      // before pickup.
      o_vision.resetROI();
      enterVisionPickupState(VisionPickupState.CALIBRATING_SERVOS);
    }

    if (visionPickupState == VisionPickupState.FAILED) {
      stopPickupDrive();
      return;
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
        || visionPickupState == VisionPickupState.ALIGNING_AT_PICKUP_HEIGHT
        || visionPickupState == VisionPickupState.GRABBING
        || visionPickupState == VisionPickupState.RAISING_LIFT
        || visionPickupState == VisionPickupState.RAISING_ARM
        || visionPickupState == VisionPickupState.COMPLETE;
    if (!o_vision.isColorCameraReady() && !pickupCommitted) {
      stopPickupDrive();
      // Freeze the viewing pose on a lost frame. Moving the lift while the
      // camera is unavailable would make reacquisition harder.
      o_vision.setText("Pickup State", "WAITING_FOR_CAMERA");
      return;
    }

    runVisionPickup();

    if (visionPickupState == VisionPickupState.COMPLETE) {
      stopPickupDrive();
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
    o_vision.setText("Pickup State", visionPickupState.name());
    o_vision.setValue("Pickup Arm Command Angle", o_subsystem.getServoArmAngle());
    pickupServoDt = Double.isFinite(pickupServoCycleTime)
        ? clampVision(now - pickupServoCycleTime, 0.0, 0.02) : 0.02;
    pickupServoCycleTime = now;
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
        boolean liftReady = o_subsystem.moveServoLiftToward(
            LIFT_SEARCH_ANGLE, pickupServoStep(SERVO_LIFT_STEP));
        boolean armReady = o_subsystem.moveServoArmToward(
            ARM_SEARCH_ANGLE, pickupServoStep(SERVO_ARM_STEP));
        boolean clawReady = o_subsystem.moveServoClawToward(
            CLAW_OPEN_ANGLE, pickupServoStep(SERVO_CLAW_STEP));
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
        if (pickupServoStage == 0) {
          o_vision.setText("Pickup Step", "OPENING_CLAW");
          if (o_subsystem.moveServoClawToward(
              CLAW_OPEN_ANGLE, pickupServoStep(SERVO_CLAW_STEP))) {
            pickupServoStage = 1;
          }
          return true;
        }

        o_vision.setText("Pickup Step", "RAISING_CAMERA_FOR_SEARCH");
        boolean cameraAtSearchHeight = o_subsystem.moveServoArmToward(
            ARM_SEARCH_ANGLE, pickupServoStep(SERVO_ARM_STEP));
        boolean liftAtSearchHeight = o_subsystem.moveServoLiftToward(
            LIFT_SEARCH_ANGLE, pickupServoStep(SERVO_LIFT_STEP));
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
        return true;

      case IDLE:
        stopPickupDrive();
        boolean searchLiftReady = o_subsystem.moveServoLiftToward(
            LIFT_SEARCH_ANGLE, pickupServoStep(SERVO_LIFT_STEP));
        boolean searchClawReady = o_subsystem.moveServoClawToward(
            CLAW_OPEN_ANGLE, pickupServoStep(SERVO_CLAW_STEP));
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

        pickupLockedColor = o_vision.getVisionColorIdentity();
        pickupTrackedArmTarget = o_subsystem.getServoArmAngle();
        o_vision.setText("Pickup Warning", "None");
        enterVisionPickupState(VisionPickupState.CENTERING);
        return true;

      case CENTERING:
        o_subsystem.servo_Hook(CLAW_OPEN_ANGLE);
        o_vision.setText("Pickup Step", "CENTERING_AND_TRACKING_OBJECT");
        if (!keepVisionTargetOrWait(now, targetVisible)) {
          return true;
        }
        holdPickupViewingPose();

        double centerOffset = pickupControlOffset();
        boolean quiet = Math.abs(centerOffset) < PickupMotion.CENTER_ACCEPTANCE_PX;
        if (quiet) {
          // Explicitly finish centering inside its acceptance zone. Steering
          // hysteresis can otherwise retain a tiny turn until offset <= 10,
          // keeping an already centered robot out of APPROACHING forever.
          stopPickupDrive();
          o_vision.setText("Pickup Step", "CONFIRMING_CENTER_BEFORE_APPROACH");
        } else {
          double turn = pickupMotion.steering(centerOffset, VISION_TURN_KP, VISION_MAX_TURN);
          setPickupDrive(turn, -turn, now);
        }
        if (pickupCenterGate.update(quiet, o_vision.getVisionFrameTimestamp(), now)) {
          stopPickupDrive();
          enterVisionPickupState(VisionPickupState.APPROACHING);
        }
        return true;

      case APPROACHING:
        o_vision.setText("Pickup Step", "APPROACHING_OBJECT");
        o_subsystem.servo_Hook(CLAW_OPEN_ANGLE);
        if (!keepVisionTargetOrWait(now, targetVisible)) {
          o_vision.setText("Pickup Warning", "TARGET_LOST_STOPPED");
          return true;
        }
        holdPickupViewingPose();

        double objectWidth = pickupFilteredWidth;
        double offset = pickupControlOffset();
        boolean aligned = Math.abs(offset)
            <= VISION_CENTER_TOLERANCE_PX * 1.5;
        // Confirm width and alignment before preparing a better view.
        // Actual grip needs separate confirmation in both later poses.
        boolean cameraGrabReady = aligned && isPickupZoneReady();
        boolean currentRequestsSecondView = Math.abs(o_vision.getVisionOffsetX())
                <= VISION_CENTER_TOLERANCE_PX * 1.5
            && PickupGeometry.firstView(o_vision.getVisionOffsetX(),
                o_vision.getVisionTargetY(), o_vision.getVisionTargetBottomY(),
                o_vision.getVisionObjectWidth());
        if (currentRequestsSecondView) {
          // Stop on the current frame. Waiting for the EMA to catch up while
          // still driving would move the robot unnecessarily closer.
          stopPickupDrive();
          o_vision.setText("Pickup Step", "CONFIRMING_SECOND_VIEW_START");
          if (pickupPositionGate.update(cameraGrabReady,
              o_vision.getVisionFrameTimestamp(), now)) {
            o_vision.setText("Pickup Step", "PREPARING_SECOND_CAMERA_VIEW");
            o_vision.setText("Pickup Warning", "None");
            enterVisionPickupState(VisionPickupState.RAISING_FOR_FINAL_VIEW);
          }
          return true;
        }
        pickupPositionGate.update(false, o_vision.getVisionFrameTimestamp(), now);

        double forward = PickupMotion.approachSpeed(VISION_APPROACH_SPEED,
            VISION_APPROACH_SLOW_SPEED,
            objectWidth, PICKUP_CAMERA_GRAB_WIDTH_PX, offset);
        if (o_vision.getVisionTargetBottomY() >= PICKUP_CAMERA_EDGE_STOP_BOTTOM_Y_PX) {
          // The scan angle may initially place a small distant object here.
          // Keep approaching a fresh visible target by width; missing frames
          // stop the drive and the progress guard prevents prolonged pushing.
          o_vision.setText("Pickup Step", "APPROACHING_OBJECT_BY_WIDTH");
          o_vision.setText("Pickup Warning", "FIRST_VIEW_EDGE_NOT_A_DISTANCE");
        }
        if (!checkPickupApproach(now)) return true;
        double steering = pickupMotion.steering(offset, VISION_TURN_KP, VISION_MAX_TURN);
        setPickupDrive(forward + steering, forward - steering, now);
        return true;

      case RAISING_FOR_FINAL_VIEW:
        o_vision.setText("Pickup Step", "RAISING_LIFT_FOR_SECOND_LOOK");
        stopPickupDrive();
        o_subsystem.servo_Hook(CLAW_OPEN_ANGLE);
        if (!o_subsystem.moveServoLiftToward(
            LIFT_FINAL_VIEW_ANGLE, pickupServoStep(SERVO_LIFT_STEP))) {
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
        o_subsystem.setServoLift(LIFT_FINAL_VIEW_ANGLE);
        o_subsystem.servo_Hook(CLAW_OPEN_ANGLE);
        if (!o_subsystem.moveServoArmToward(
            ARM_SCAN_DOWN_ANGLE, pickupServoStep(SERVO_ARM_STEP))) {
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
        o_subsystem.setServoLift(LIFT_FINAL_VIEW_ANGLE);
        o_subsystem.servo_Hook_Hand(ARM_SCAN_DOWN_ANGLE);
        o_subsystem.servo_Hook(CLAW_OPEN_ANGLE);
        if (!targetVisible || !pickupFilterReady) {
          stopPickupDrive();
          o_vision.setValue("Pickup Second Grab Zone", 0);
          pickupFilterReady = false;
          pickupPositionGate.update(false, o_vision.getVisionFrameTimestamp(), now);
          o_vision.setText("Pickup Warning", "WAITING_FOR_SECOND_VIEW");
          return true;
        }

        double finalOffset = pickupControlOffset();
        if (Math.abs(finalOffset) > VISION_CENTER_TOLERANCE_PX * 1.5) {
          o_vision.setValue("Pickup Second Grab Zone", 0);
          pickupPositionGate.update(false, o_vision.getVisionFrameTimestamp(), now);
          o_vision.setText("Pickup Step", "SECOND_VIEW_CENTERING");
          double finalTurn = pickupMotion.steering(
              finalOffset, VISION_TURN_KP, VISION_MAX_TURN * 0.8);
          setPickupTranslation(0.0, finalTurn, now);
          return true;
        }

        boolean finalGrabZone = isPickupZoneReady(
            FINAL_VIEW_MIN_WIDTH_PX, FINAL_VIEW_GRAB_CENTER_Y_PX,
            FINAL_VIEW_GRAB_BOTTOM_Y_PX);
        o_vision.setValue("Pickup Second Grab Zone", finalGrabZone ? 1.0 : 0.0);
        if (!finalGrabZone) {
          pickupPositionGate.update(false, o_vision.getVisionFrameTimestamp(), now);
          if (o_vision.getVisionTargetBottomY() >= PICKUP_CAMERA_EDGE_STOP_BOTTOM_Y_PX) {
            failPickup("SECOND_VIEW_AT_CAMERA_EDGE");
            return true;
          }

          // Keep the arm fully down and the claw open. Only the wheels move
          // during this second, camera-guided approach.
          if (!checkPickupApproach(now)) return true;
          // Final corrections never accelerate beyond the first approach.
          double finalForward = PickupMotion.approachSpeed(
              Math.min(FINAL_APPROACH_SPEED, VISION_APPROACH_SPEED),
              VISION_APPROACH_SLOW_SPEED,
              pickupFilteredWidth, FINAL_VIEW_MIN_WIDTH_PX, finalOffset);
          double finalSteering = pickupMotion.steering(finalOffset, VISION_TURN_KP, 0.035);
          o_vision.setText("Pickup Step", "SECOND_VIEW_APPROACHING_OBJECT");
          setPickupTranslation(finalForward, finalSteering, now);
          return true;
        }

        o_vision.setText("Pickup Step", "SECOND_VIEW_CONFIRMING_GRAB_ZONE");
        stopPickupDrive();
        if (pickupPositionGate.update(true, o_vision.getVisionFrameTimestamp(), now)) {
          stopPickupDrive();
          o_vision.setText("Pickup Warning", "None");
          enterVisionPickupState(VisionPickupState.LOWERING_LIFT);
        }
        return true;

      case LOWERING_LIFT:
        o_vision.setText("Pickup Step", "LOWERING_LIFT_WITH_ARM_DOWN");
        stopPickupDrive();
        o_subsystem.servo_Hook(CLAW_OPEN_ANGLE);
        // Changing lift height invalidates the confirmed image. Reacquire
        // and finish approaching in this actual pickup pose before closing.
        o_subsystem.servo_Hook_Hand(ARM_SCAN_DOWN_ANGLE);
        if (!o_subsystem.moveServoLiftToward(
            LIFT_PICKUP_ANGLE, pickupServoStep(PICKUP_LIFT_TRACK_STEP))) {
          pickupServoSettledSince = 0;
          return true;
        }
        if (pickupServoSettledSince == 0) {
          pickupServoSettledSince = now;
        } else if (now - pickupServoSettledSince
            >= LIFT_LOWER_SETTLE_SEC) {
          enterVisionPickupState(VisionPickupState.ALIGNING_AT_PICKUP_HEIGHT);
        }
        return true;

      case ALIGNING_AT_PICKUP_HEIGHT:
        o_subsystem.setServoLift(LIFT_PICKUP_ANGLE);
        o_subsystem.servo_Hook_Hand(ARM_SCAN_DOWN_ANGLE);
        o_subsystem.servo_Hook(CLAW_OPEN_ANGLE);
        if (!targetVisible || !pickupFilterReady) {
          stopPickupDrive();
          o_vision.setValue("Pickup Final Grab Zone", 0);
          pickupFilterReady = false;
          pickupPositionGate.update(false, o_vision.getVisionFrameTimestamp(), now);
          o_vision.setText("Pickup Step", "WAITING_FOR_OBJECT_AT_PICKUP_HEIGHT");
          o_vision.setText("Pickup Warning", "NO_FRESH_TARGET_NO_GRIP");
          return true;
        }

        boolean pickupHeightZone = isPickupZoneReady(
            PICKUP_HEIGHT_MIN_WIDTH_PX, PICKUP_HEIGHT_GRAB_CENTER_Y_PX,
            PICKUP_HEIGHT_GRAB_BOTTOM_Y_PX);
        o_vision.setValue("Pickup Final Grab Zone", pickupHeightZone ? 1.0 : 0.0);
        if (pickupHeightZone) {
          stopPickupDrive();
          o_vision.setText("Pickup Step", "CONFIRMING_OBJECT_AT_PICKUP_HEIGHT");
          if (pickupPositionGate.update(true, o_vision.getVisionFrameTimestamp(), now)) {
            o_vision.setText("Pickup Warning", "None");
            enterVisionPickupState(VisionPickupState.GRABBING);
          }
          return true;
        }

        pickupPositionGate.update(false, o_vision.getVisionFrameTimestamp(), now);
        double pickupHeightOffset = pickupControlOffset();
        if (o_vision.getVisionTargetBottomY() >= PICKUP_CAMERA_EDGE_STOP_BOTTOM_Y_PX) {
          failPickup("PICKUP_HEIGHT_AT_CAMERA_EDGE");
          return true;
        }
        if (!checkPickupApproach(now)) return true;
        double pickupStrafe = pickupMotion.steering(
            pickupHeightOffset, VISION_TURN_KP, PICKUP_MAX_STRAFE);
        double pickupForward = PickupMotion.approachSpeed(PICKUP_HEIGHT_APPROACH_SPEED,
            Math.min(PICKUP_HEIGHT_APPROACH_SPEED, VISION_APPROACH_SLOW_SPEED),
            pickupFilteredWidth, PICKUP_HEIGHT_MIN_WIDTH_PX, pickupHeightOffset);
        o_vision.setText("Pickup Step", "APPROACHING_AT_PICKUP_HEIGHT");
        setPickupTranslation(pickupForward, pickupStrafe, now);
        return true;

      case GRABBING:
        o_vision.setText("Pickup Step", "CLOSING_CLAW");
        stopPickupDrive();
        o_subsystem.setServoLift(LIFT_PICKUP_ANGLE);
        o_subsystem.servo_Hook_Hand(ARM_SCAN_DOWN_ANGLE);
        if (!pickupGripStarted) {
          if (!targetVisible || !isPickupZoneReady(PICKUP_HEIGHT_MIN_WIDTH_PX,
              PICKUP_HEIGHT_GRAB_CENTER_Y_PX, PICKUP_HEIGHT_GRAB_BOTTOM_Y_PX)) {
            enterVisionPickupState(VisionPickupState.ALIGNING_AT_PICKUP_HEIGHT);
            return true;
          }
          pickupGripStarted = true;
        }
        if (!o_subsystem.moveServoClawToward(
            CLAW_CLOSED_ANGLE, pickupServoStep(SERVO_CLAW_STEP))) {
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
        o_subsystem.servo_Hook(CLAW_CLOSED_ANGLE);
        o_subsystem.servo_Hook_Hand(ARM_SCAN_DOWN_ANGLE);
        if (!o_subsystem.moveServoLiftToward(
            LIFT_SEARCH_ANGLE, pickupServoStep(SERVO_LIFT_STEP))) {
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
        o_subsystem.setServoLift(LIFT_SEARCH_ANGLE);
        o_subsystem.servo_Hook(CLAW_CLOSED_ANGLE);
        if (!o_subsystem.moveServoArmToward(
            ARM_SEARCH_ANGLE, pickupServoStep(SERVO_ARM_STEP))) {
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
        o_subsystem.setServoLift(LIFT_SEARCH_ANGLE);
        o_subsystem.servo_Hook(CLAW_CLOSED_ANGLE);
        o_subsystem.servo_Hook_Hand(ARM_SEARCH_ANGLE);
        return false;

      default:
        return false;
    }
  }

  private boolean isPickupZoneReady() {
    boolean ready = pickupFilterReady
        // Consecutive current frames confirm size. Applying the width EMA
        // here can never reach a boundary value approached from below.
        && Math.abs(pickupFilteredX) <= VISION_CENTER_TOLERANCE_PX * 1.5
        && PickupGeometry.firstView(o_vision.getVisionOffsetX(),
            o_vision.getVisionTargetY(), o_vision.getVisionTargetBottomY(),
            o_vision.getVisionObjectWidth());
    o_vision.setValue("Pickup In Grab Zone", ready ? 1.0 : 0.0);
    return ready;
  }

  private boolean isPickupZoneReady(double minWidth, double minCenterY, double minBottomY) {
    return pickupFilterReady
        && PickupGeometry.inView(pickupFilteredX, pickupFilteredY,
            pickupFilteredBottom, pickupFilteredWidth, minWidth, minCenterY, minBottomY)
        && PickupGeometry.inView(o_vision.getVisionOffsetX(),
            o_vision.getVisionTargetY(), o_vision.getVisionTargetBottomY(),
            o_vision.getVisionObjectWidth(), minWidth, minCenterY, minBottomY);
  }

  private void updatePickupVisionFilter() {
    double frameTimestamp = o_vision.getVisionFrameTimestamp();
    pickupFrameUpdated = Double.isFinite(frameTimestamp)
        && frameTimestamp > pickupLastFrameTimestamp;
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
    // Keep these readings live in every pose, rather than freezing at the
    // first approach. Raw Target Width is published by VisionSubsystem.
    o_vision.setValue("Pickup Target Width", pickupFilteredWidth);
    o_vision.setValue("Pickup Target Bottom", pickupFilteredBottom);
    isPickupZoneReady();
  }

  private void setPickupDrive(double left, double right, double now) {
    pickupDriveStrafe = 0;
    if (Math.abs(o_vision.getVisionOffsetX()) >= 28.0) pickupMotion.stopForward();
    double[] command = pickupMotion.update((left + right) / 2.0, (left - right) / 2.0, now);
    pickupDriveLeft = command[0] + command[1];
    pickupDriveRight = command[0] - command[1];
    o_subsystem.setDrivePower(pickupDriveLeft, pickupDriveRight);
    o_vision.setValue("Pickup Forward Requested", (left + right) / 2.0);
    o_vision.setValue("Pickup Forward Command", command[0]);
    o_vision.setValue("Pickup Turn Command", command[1]);
  }

  /** At pickup height translate laterally without swinging the claw. */
  private void setPickupTranslation(double forward, double strafe, double now) {
    if (Math.abs(o_vision.getVisionOffsetX()) >= 28.0) pickupMotion.stopForward();
    double[] command = pickupMotion.update(forward,
        clampVision(strafe, -PICKUP_MAX_STRAFE, PICKUP_MAX_STRAFE), now);
    pickupDriveStrafe = command[1];
    pickupDriveLeft = pickupDriveRight = command[0];
    o_subsystem.holonomicDrive(pickupDriveStrafe,
        command[0] / ThreeWheelDrive.FORWARD_PROJECTION, 0.0);
    o_vision.setValue("Pickup Strafe", pickupDriveStrafe);
    o_vision.setValue("Pickup Forward Requested", forward);
    o_vision.setValue("Pickup Forward Command", command[0]);
    o_vision.setValue("Pickup Turn Command", 0);
  }

  private void stopPickupDrive() {
    pickupDriveLeft = pickupDriveRight = pickupDriveStrafe = 0;
    pickupMotion.reset(Timer.getFPGATimestamp());
    o_subsystem.stopDrive();
    o_vision.setValue("Pickup Strafe", 0);
    o_vision.setValue("Pickup Forward Requested", 0);
    o_vision.setValue("Pickup Forward Command", 0);
    o_vision.setValue("Pickup Turn Command", 0);
  }

  private double pickupServoStep(double maximumStep) {
    return maximumStep * pickupServoDt / 0.02;
  }

  private void holdPickupViewingPose() {
    // A moving camera changes apparent proximity even on a stationary robot.
    o_subsystem.servo_Hook_Hand(pickupTrackedArmTarget);
    o_subsystem.setServoLift(LIFT_SEARCH_ANGLE);
  }

  private boolean checkPickupApproach(double now) {
    // At 5 FPS the position EMA lags by several frames. Use the current
    // measured width for progress; retain filtering for smooth speed control.
    boolean valid = pickupApproachGuard.update(now, o_vision.getVisionObjectWidth(),
        pickupFrameUpdated, (pickupDriveLeft + pickupDriveRight) / 2.0);
    if (!valid) failPickup(pickupApproachGuard.getFailure());
    return valid;
  }

  private void failPickup(String reason) {
    o_vision.setText("Pickup Failed Stage", visionPickupState.name());
    o_vision.setValue("Pickup Failure X", o_vision.getVisionOffsetX());
    o_vision.setValue("Pickup Failure Y", o_vision.getVisionTargetY());
    o_vision.setValue("Pickup Failure Bottom", o_vision.getVisionTargetBottomY());
    o_vision.setValue("Pickup Failure Width", o_vision.getVisionObjectWidth());
    o_vision.setValue("Pickup Failure Forward", (pickupDriveLeft + pickupDriveRight) / 2.0);
    stopPickupDrive();
    enterVisionPickupState(VisionPickupState.FAILED);
    o_vision.setText("Pickup Error", reason);
    o_vision.setText("Pickup Step", "STOPPED_REQUIRE_NEW_START");
  }

  private void scanPickupCamera(double now) {
    boolean atScanAngle = o_subsystem.moveServoArmToward(
        pickupScanArmTarget, pickupServoStep(SERVO_ARM_STEP));
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

  private boolean keepVisionTargetOrWait(double now, boolean targetVisible) {
    if (targetVisible) {
      o_vision.setText("Pickup Warning", "None");
      return true;
    }

    stopPickupDrive();
    pickupFilterReady = false;
    pickupPositionGate.update(false, o_vision.getVisionFrameTimestamp(), now);
    pickupCenterGate.update(false, o_vision.getVisionFrameTimestamp(), now);
    if (now - visionTargetLastSeen > VISION_TARGET_LOST_TIMEOUT_SEC) {
      enterVisionPickupState(VisionPickupState.IDLE);
    }
    return false;
  }

  private double pickupControlOffset() {
    return PickupMotion.controlOffset(pickupFilteredX, o_vision.getVisionOffsetX());
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

  public void rotateTheRobot(double targetDeg) {
    if (!o_subsystem.isGyroReady()) {
      o_subsystem.stopDrive();
      return;
    }
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

    double speed = maxSpeed;
    double targetMm = dist * 11.1;

    if (travelledMm >= targetMm) {
      o_subsystem.TimerStop();
      stateAutomatic++;
    } else {
      if (targetMm - travelledMm <= 100.0) {
        speed = minSpeed;
      }
      o_subsystem.driveStraightWithHeading(speed, angleRobot);
    }
  }

  public void goBackDistance(double dist) {
    if (dist <= 0) {
      o_subsystem.TimerStop();
      stateAutomatic++;
      return;
    }

    double travelledMm = -o_subsystem.getAverageDriveDistance();

    double speed = maxSpeed;
    double targetMm = dist * 11.1;

    if (travelledMm >= targetMm) {
      o_subsystem.TimerStop();
      stateAutomatic++;
    } else {
      if (targetMm - travelledMm <= 100.0) {
        speed = minSpeed;
      }
      o_subsystem.driveStraightWithHeading(-speed, angleRobot);
    }
  }

  public void goForwardSharp(double dist) {
    if (dist <= 0) {
      o_subsystem.stopAllMotors();
      stateAutomatic++;
      return;
    }

    double slowdown_factor = Math.max(0, Math.min(1.0, (o_subsystem.getForwardSharp() - dist + 10) / dist));
    o_subsystem.driveStraightWithHeading(0.3 * slowdown_factor, angleRobot);
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

    double slowdown_factor = Math.max(0, Math.min(1.0, (o_subsystem.getBackSharp() - dist + 10) / dist));
    o_subsystem.driveStraightWithHeading(-0.3 * slowdown_factor, angleRobot);
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
    o_subsystem.driveStraightWithHeading(speed, angleRobot);
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
    o_subsystem.driveStraightWithHeading(-speed, angleRobot);
  }

  public void goForwardWithBackSharp(double dist) {
    double speed = maxSpeed;
    if (o_subsystem.getBackSharp() >= dist - 15) {
      speed = 0.15;
    }
    o_subsystem.driveStraightWithHeading(speed, angleRobot);
    if (o_subsystem.getBackSharp() >= dist) {
      o_subsystem.TimerStop();
      stateAutomatic++;
    }
  }

  public void goLeftWithRightSonic(double dist) {
    double sonic = o_subsystem.getDistanceSonicRear();
    o_subsystem.driveStraightWithHeading(maxSpeed / 2, angleRobot);
    if (sonic >= dist) {
      o_subsystem.TimerStop();
      stateAutomatic++;
    }
  }

  public void goRigtWithLeftSonic(double dist) {
    double sonic = o_subsystem.getDistanceSonicFront();
    double speed = maxSpeed / 2;
    if (sonic > dist - 10) {
      speed = maxSpeed * 0.25;
    }
    o_subsystem.driveStraightWithHeading(-speed, angleRobot);
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
    if (!o_subsystem.isGyroReady()) {
      o_subsystem.stopDrive();
      return;
    }
    double sharpForward = o_subsystem.getForwardSharp();
    double sonicFront   = o_subsystem.getDistanceSonicFront();
    double sonicRear    = o_subsystem.getDistanceSonicRear();
    double currentYaw   = normalizeYaw(o_subsystem.getYaw());

    if (driveState.equals("forward")) {
      if (sharpForward > 15.0) {
        o_subsystem.driveStraightWithHeading(0.28, angleRobot);
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
