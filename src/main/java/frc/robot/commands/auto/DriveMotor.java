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

  // ── Pickup geometry (one set for every object height) ──
  // First view (camera forward, lift up): approach until the object is this
  // wide in the image, then switch to the second view with the arm down.
  private static final double FIRST_VIEW_WIDTH_PX = 160.0;
  // Second view (arm down, lift up): drive until the object sits this low
  // and is this wide in the image.
  private static final double FINAL_VIEW_CENTER_Y_PX = 150.0;
  private static final double FINAL_VIEW_BOTTOM_Y_PX = 225.0;
  private static final double FINAL_VIEW_WIDTH_PX = 110.0;
  // ── Camera-guided lift descent ──
  // The lift goes down while the camera watches the object grow. It stops
  // when the object is GRAB_WIDTH_PX wide: the claw is then at the same
  // distance from it whether it lies on the floor or on the podium.
  // Calibrate: hold an object in the open claw, read "Pickup Descent Width".
  private static final double GRAB_WIDTH_PX = 170.0;
  // Object filling this part of the frame width also counts as reached.
  private static final double GRAB_FRAME_FILL = 0.85;
  // Lift step per 20 ms cycle: fast far away, slow just before the object.
  private static final double DESCENT_MAX_STEP = 0.4;
  private static final double DESCENT_MIN_STEP = 0.12;
  // Lowest safe lift angle (floor pickup). Never go below it.
  private static final double LIFT_LOWEST_ANGLE = 160.0;
  // Object vanished while already this close (fraction of GRAB_WIDTH_PX):
  // it is too close for the camera, finish the last few degrees blind.
  private static final double DESCENT_LOST_ACCEPT_FACTOR = 0.70;
  private static final double DESCENT_BLIND_EXTRA_DEG = 8.0;
  // Object vanished far away: go back to searching after this time.
  private static final double DESCENT_LOST_TIMEOUT_SEC = 1.0;
  // Where the object must be in the image (Y, px) when it is under the
  // claw. During the descent the robot creeps forward/back to keep it there,
  // so floor and podium objects both end up in the claw.
  // Calibrate together with GRAB_WIDTH_PX: object in the open claw, read
  // "Pickup Filtered Y".
  private static final double GRAB_TARGET_Y_PX = 150.0;
  private static final double GRAB_Y_DEADBAND_PX = 12.0;
  // Width-vs-lift samples used to predict the lift angle at which the
  // object will be GRAB_WIDTH_PX wide, for when it leaves the view early.
  // Only the most recent samples: the lift is a rotating arm, so the camera
  // height is not linear in the lift angle and old samples bend the line.
  private static final int DESCENT_SAMPLE_SIZE = 10;
  private static final int DESCENT_MIN_SAMPLES = 4;
  // The straight-line prediction stops a little high on a rotating lift;
  // go this much lower to compensate.
  private static final double DESCENT_PREDICT_MARGIN_DEG = 3.0;

  private static final double VISION_CENTER_TOLERANCE_PX = 14.0;
  private static final double PICKUP_FRAME_MAX_AGE_SEC = 0.35;
  private static final double PICKUP_FILTER_ALPHA = 0.5;
  private static final double PICKUP_DRIVE_ACCEL_PER_SEC = 0.50;
  private static final double PICKUP_DRIVE_DECEL_PER_SEC = 1.50;
  private static final double VISION_TARGET_LOST_TIMEOUT_SEC = 0.75;
  private static final double VISION_CENTER_HOLD_SEC = 0.25;
  private static final double VISION_TARGET_CONFIRM_SEC = 0.15;
  private static final double VISION_CAMERA_CLOSE_CONFIRM_SEC = 0.30;
  private static final double VISION_CAMERA_CLOSE_WIDTH_PX = 300.0;
  private static final double PICKUP_ZONE_MIN_CENTER_Y_PX = 145.0;
  private static final double PICKUP_ZONE_MIN_BOTTOM_Y_PX = 125.0;
  // Stay in the first camera-guided approach until the profile-specific width
  // is reached, then switch to the second view with the arm lowered.
  private static final double PICKUP_CAMERA_GRAB_BOTTOM_Y_PX = 150.0;
  // The camera frame is 240 px high. The old 300 px threshold could never be
  // reached, so every "object at the bottom edge" branch was dead code. The
  // edge is now taken from the frame height reported by the vision process.
  private static final double PICKUP_BOTTOM_EDGE_MARGIN_PX = 6.0;
  // Approach: if the centered object drops out of the bottom of the frame,
  // it is under the camera, not lost. Go to the second view instead of
  // restarting the search.
  private static final double PICKUP_EXIT_BOTTOM_WAIT_SEC = 0.20;
  // Second view: an object cut by the bottom edge is accepted for the grab
  // when it is centered and at least this fraction of the final width.
  private static final double PICKUP_EDGE_ACCEPT_WIDTH_FACTOR = 0.85;
  private static final double PICKUP_EDGE_BACKOFF_SPEED = 0.20;
  // Second view: no object for this long returns to the search pose instead
  // of waiting forever with the arm down.
  private static final double PICKUP_SECOND_VIEW_TIMEOUT_SEC = 2.0;
  // ── Drive during pickup ──
  // Pickup steers only with forward motion and rotation, the same motions the
  // autonomous route already uses. Sideways strafing needs the rear wheel's
  // direction to be right, and the robot drove the wrong way with it.
  //
  // Each camera frame is turned into a gyro heading: the yaw the robot had
  // when that frame was captured plus the pixel offset converted to degrees.
  // The gyro then turns the robot to that heading without waiting for the
  // slow, delayed camera, so it no longer overshoots and swings.
  // Set to -1 only if the camera image is mirrored (robot turns away).
  private static final double VISION_OFFSET_SIGN = 1.0;
  // Horizontal field of view of the USB camera across 320 px. An error here
  // only changes how many frames it takes to center, not the direction.
  private static final double CAMERA_HFOV_DEG = 60.0;
  private static final double CAMERA_DEG_PER_PX = CAMERA_HFOV_DEG / 320.0;
  // A floor object is seen from above: the camera-to-object distance is
  // longer than the horizontal distance, so it looks smaller and the robot
  // drove too close and pushed it. The width is divided by the cosine of
  // the camera's downward angle (arm tilt + position in the frame), which
  // makes floor and podium objects stop at the same horizontal distance.
  // Arm servo angle at which the camera looks straight ahead:
  private static final double ARM_HORIZONTAL_ANGLE = 80.0;
  private static final double CAMERA_DEG_PER_PX_V = CAMERA_DEG_PER_PX;
  private static final double MIN_DOWN_COS = 0.25;
  private static final double PICKUP_AIM_TOLERANCE_DEG = 1.5;
  // Turning in place: same 0.15..0.30 power range as rotateTheRobot().
  private static final double PICKUP_TURN_KP = 0.015;
  private static final double PICKUP_MIN_TURN = 0.15;
  private static final double PICKUP_MAX_TURN = 0.30;
  // Steering while driving forward.
  private static final double PICKUP_STEER_KP = 0.012;
  private static final double PICKUP_MAX_STEER = 0.12;
  // Drive straight only when nearly aimed; otherwise turn in place first.
  private static final double PICKUP_APPROACH_MAX_AIM_ERROR_DEG = 6.0;
  // The route finishes every move at minSpeed = 0.2, so below that the
  // robot may not move at all. That was the "drives up and just stands".
  private static final double PICKUP_MIN_FORWARD = 0.20;
  // If the wheels are commanded but the encoders barely move, add power in
  // small steps (worn battery, carpet, heavier load).
  private static final double PICKUP_STALL_CHECK_SEC = 0.6;
  private static final double PICKUP_STALL_MIN_TRAVEL_MM = 8.0;
  private static final double PICKUP_STALL_BOOST_STEP = 0.03;
  private static final double PICKUP_STALL_BOOST_MAX = 0.12;
  private static final int YAW_HISTORY_SIZE = 64;
  // ── Lost target: keep searching where it was last seen ──
  // Last position in the outer part of the frame (fraction of the size)
  // gives the search direction: bottom → tilt camera down, top → up,
  // left/right → turn the robot that way.
  private static final double LOST_EDGE_ZONE = 0.30;
  private static final double PICKUP_LOST_SEARCH_SEC = 2.0;
  private static final double PICKUP_LOST_ARM_STEP = 0.6;
  private static final int PICKUP_CAMERA_GRAB_MIN_FRAMES = 3;
  private static final double VISION_GRIP_DELAY_SEC = 1.50;
  private static final double SERVO_CALIBRATION_SETTLE_SEC = 1.50;
  private static final double LIFT_LOWER_SETTLE_SEC = 0.25;
  private static final double ARM_LOWER_SETTLE_SEC = 0.50;
  private static final double FINAL_VIEW_CONFIRM_SEC = 0.30;
  private static final int FINAL_VIEW_CONFIRM_FRAMES = 3;
  private static final double FINAL_APPROACH_SPEED = 0.20;
  private static final double LIFT_RAISE_SETTLE_SEC = 0.35;
  private static final double SERVO_LIFT_STEP = 0.8;
  private static final double SERVO_ARM_STEP = 0.8;
  private static final double PICKUP_ARM_TRACK_STEP = 1.2;
  private static final double SERVO_CLAW_STEP = 2.0;
  // Close the last part of the claw travel slowly so it does not push the
  // object away before the jaws meet it.
  private static final double SERVO_CLAW_SLOW_STEP = 1.0;
  private static final double CLAW_SLOW_ZONE_DEG = 40.0;
  // Raise the lift and arm with an ease-in/ease-out profile (average speed,
  // deg/s). The object starts and stops moving smoothly inside the claw.
  private static final double PICKUP_LIFT_RAISE_DEG_PER_SEC = 35.0;
  private static final double PICKUP_ARM_RAISE_DEG_PER_SEC = 35.0;
  private static final double VISION_APPROACH_SPEED = 0.13;
  private static final double VISION_APPROACH_SLOW_SPEED = 0.09;
  private static final double PICKUP_SCAN_STEP_DEG = 5.0;
  private static final double PICKUP_SCAN_DWELL_SEC = 0.45;
  private static final double PICKUP_ARM_UPDATE_SEC = 0.05;
  // During approach keep the target in the lower half of the 240 px frame.
  private static final double PICKUP_IMAGE_CENTER_Y_PX = 170.0;
  private static final double PICKUP_IMAGE_Y_DEADBAND_PX = 18.0;
  private static final double PICKUP_ARM_KP = 0.015;
  private static final double PICKUP_ARM_MAX_CORRECTION_DEG = 1.2;
  // User-calibrated physical positions. Do not infer direction from the
  // numeric angle or swap these values automatically.
  private static final int CLAW_OPEN_ANGLE = 18;
  private static final int CLAW_CLOSED_ANGLE = 170;
  // 80 degrees looks forward; increasing the angle aims down at the floor.
  // The arm servo's 15-degree safety margin clamps its 180-degree endpoint
  // to 165 degrees in ExampleSubsystem.
  private static final double ARM_SEARCH_ANGLE = 80.0;
  private static final double ARM_SCAN_DOWN_ANGLE = 165.0;
  private static final int LIFT_SEARCH_ANGLE = 15;
  // Lift position during the second camera-guided approach; tune separately.
  private static final int LIFT_FINAL_VIEW_ANGLE = 15;

  private VisionPickupState visionPickupState = VisionPickupState.IDLE;
  private double pickupGrabLiftAngle = LIFT_SEARCH_ANGLE;
  private double pickupBlindLiftTarget = Double.NaN;
  private double pickupDescentLastWidth = 0;
  private final double[] descentSampleLift = new double[DESCENT_SAMPLE_SIZE];
  private final double[] descentSampleInvWidth = new double[DESCENT_SAMPLE_SIZE];
  private int descentSampleNext = 0;
  private int descentSampleCount = 0;
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
  private double pickupLastArmUpdateAt = 0;
  private double pickupLastFrameTimestamp = 0;
  private boolean pickupFrameUpdated = false;
  private boolean pickupFilterReady = false;
  private double pickupFilteredX = 0;
  private double pickupFilteredY = PICKUP_IMAGE_CENTER_Y_PX;
  private double pickupFilteredWidth = 0;
  private double pickupFilteredBottom = PICKUP_IMAGE_CENTER_Y_PX;
  private double pickupDriveForward = 0;
  private double pickupDriveTurn = 0;
  private double pickupAimYaw = 0;
  private boolean pickupAimValid = false;
  private double pickupStallBoost = 0;
  private double pickupStallRefAt = 0;
  private double pickupStallRefLeft = 0;
  private double pickupStallRefRight = 0;
  private double pickupStallRefRear = 0;
  private final double[] yawHistoryTime = new double[YAW_HISTORY_SIZE];
  private final double[] yawHistoryValue = new double[YAW_HISTORY_SIZE];
  private int yawHistoryNext = 0;
  private int yawHistoryCount = 0;
  private double pickupDriveLastAt = 0;
  private String pickupCandidateColor = null;
  private String pickupLockedColor = null;
  private int pickupServoStage = 0;
  private double pickupServoSettledSince = 0;
  private boolean visionPickupComplete = false;
  private int visionPickupCase = -1;
  private boolean pickupLastSeenAtBottom = false;
  private double pickupProfileStartAngle = 0;
  private int pickupLostDirX = 0;   // -1 left, 1 right
  private int pickupLostDirY = 0;   // -1 top, 1 bottom
  private double pickupIdleLiftTarget = LIFT_SEARCH_ANGLE;

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
          visionPickup();
          break;
        case 1:
          rotateTheRobot(90);
          break;
        case 2:
          o_subsystem.servo_Hook(18);
          stateAutomatic++;
          break;
        case 3:
          rotateTheRobot(0);
          break;
        case 4:
          visionPickup();
          break;
        case 5:
          // rotateTheRobot(0);
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
    pickupLastArmUpdateAt = 0;
    pickupLastFrameTimestamp = 0;
    pickupFrameUpdated = false;
    pickupFilterReady = false;
    pickupDriveForward = 0;
    pickupDriveTurn = 0;
    pickupDriveLastAt = 0;
    pickupAimValid = false;
    pickupStallBoost = 0;
    pickupStallRefAt = 0;
    pickupCandidateColor = null;
    pickupLockedColor = null;
    pickupServoStage = 0;
    pickupServoSettledSince = 0;
    visionPickupComplete = false;
    visionPickupCase = -1;
    pickupLastSeenAtBottom = false;
    pickupLostDirX = 0;
    pickupLostDirY = 0;
    o_vision.setColorLock(null);
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
      if (pickupLostDirY != 0) {
        // Lost at the top/bottom: scan on from the current camera angle in
        // that direction and keep the lift where it is.
        pickupScanArmTarget = o_subsystem.getServoArmAngle();
        pickupScanDirection = pickupLostDirY;
        pickupIdleLiftTarget = o_subsystem.getServoLiftAngle();
      } else {
        pickupScanArmTarget = ARM_SEARCH_ANGLE;
        pickupScanDirection = 1;
        pickupIdleLiftTarget = LIFT_SEARCH_ANGLE;
      }
      pickupLostDirX = 0;
      pickupLostDirY = 0;
      pickupScanNextStepAt = visionPickupStateStart + PICKUP_SCAN_DWELL_SEC;
      pickupFilterReady = false;
      pickupLastSeenAtBottom = false;
      pickupAimValid = false;
      o_vision.setColorLock(null);
    }
    if (state == VisionPickupState.LOWERING_LIFT) {
      pickupBlindLiftTarget = Double.NaN;
      pickupDescentLastWidth = pickupFilteredWidth;
      descentSampleNext = 0;
      descentSampleCount = 0;
    }
    if (state == VisionPickupState.RAISING_LIFT) {
      pickupProfileStartAngle = o_subsystem.getServoLiftAngle();
    }
    if (state == VisionPickupState.RAISING_ARM) {
      pickupProfileStartAngle = o_subsystem.getServoArmAngle();
    }
    if (state == VisionPickupState.COMPLETE) {
      o_vision.setColorLock(null);
    }
    if (state == VisionPickupState.REACQUIRING_OBJECT) {
      // Require a new image after changing the camera pose. The old frame
      // describes the first approach and cannot authorize the final grip.
      pickupFilterReady = false;
      pickupLastFrameTimestamp = o_vision.getVisionFrameTimestamp();
      pickupFrameUpdated = false;
      visionTargetSeenSince = 0;
      pickupAimValid = false;
      pickupLostDirX = 0;
      pickupLostDirY = 0;
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
      pickupLastArmUpdateAt = 0;
      pickupLastFrameTimestamp = 0;
      pickupFrameUpdated = false;
      pickupFilterReady = false;
      pickupDriveForward = 0;
      pickupDriveTurn = 0;
      pickupDriveLastAt = 0;
      pickupAimValid = false;
      pickupStallBoost = 0;
      pickupStallRefAt = 0;
      pickupCandidateColor = null;
      pickupLockedColor = null;
      pickupServoStage = 0;
      pickupServoSettledSince = 0;
      visionPickupComplete = false;
      visionPickupCase = stateAutomatic;
      pickupLastSeenAtBottom = false;
      pickupLostDirX = 0;
      pickupLostDirY = 0;
      o_vision.setColorLock(null);
      o_vision.setText("Pickup Error", "None");
      o_vision.setText("Pickup Warning", "None");
      stopPickupDrive();
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
    boolean targetVisible = isPickupObjectVisible();
    recordYawHistory();

    if (targetVisible) {
      visionTargetLastSeen = now;
      updatePickupVisionFilter();
      if (pickupFrameUpdated) {
        updatePickupAim();
      }
    } else {
      pickupFrameUpdated = false;
    }

    switch (visionPickupState) {
      case CALIBRATING_SERVOS:
        stopPickupDrive();
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
        return true;

      case IDLE:
        stopPickupDrive();
        boolean searchLiftReady = o_subsystem.moveServoLiftToward(
            pickupIdleLiftTarget, SERVO_LIFT_STEP);
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
        o_vision.setColorLock(pickupLockedColor);
        pickupTrackedArmTarget = o_subsystem.getServoArmAngle();
        pickupLastArmUpdateAt = now;
        o_vision.setText("Pickup Warning", "None");
        enterVisionPickupState(VisionPickupState.CENTERING);
        return true;

      case CENTERING:
        o_subsystem.servo_Hook(CLAW_OPEN_ANGLE);
        o_vision.setText("Pickup Step", "CENTERING_AND_TRACKING_OBJECT");
        if (!keepVisionTargetOrWait(now, targetVisible)) {
          return true;
        }
        followPickupTarget(now);

        double centerTurn = aimTurnInPlace();
        boolean centered = centerTurn == 0.0
            && Math.abs(getRawPickupOffset()) <= VISION_CENTER_TOLERANCE_PX;
        if (!centered) {
          visionCenteredSince = 0;
          setPickupDrive(0, centerTurn, now);
        } else {
          stopPickupDrive();
          if (visionCenteredSince == 0) {
            visionCenteredSince = now;
          } else if (now - visionCenteredSince >= VISION_CENTER_HOLD_SEC) {
            enterVisionPickupState(VisionPickupState.APPROACHING);
          }
        }
        return true;

      case APPROACHING:
        o_vision.setText("Pickup Step", "APPROACHING_OBJECT");
        o_subsystem.servo_Hook(CLAW_OPEN_ANGLE);
        if (!targetVisible && pickupLastSeenAtBottom
            && Math.abs(pickupFilteredX) <= VISION_CENTER_TOLERANCE_PX * 1.5
            && (getLevelWidth() >= FIRST_VIEW_WIDTH_PX
                || isPickupArmAtLowerLimit())
            && now - visionTargetLastSeen >= PICKUP_EXIT_BOTTOM_WAIT_SEC) {
          stopPickupDrive();
          o_vision.setText("Pickup Warning", "OBJECT_LEFT_FRAME_AT_BOTTOM");
          o_vision.setText("Pickup Exit Reason", "LEFT_FRAME_AT_BOTTOM");
          enterVisionPickupState(VisionPickupState.RAISING_FOR_FINAL_VIEW);
          return true;
        }
        if (!keepVisionTargetOrWait(now, targetVisible)) {
          o_vision.setText("Pickup Warning", "TARGET_LOST_STOPPED");
          return true;
        }
        followPickupTarget(now);

        // Height-independent width, see ARM_HORIZONTAL_ANGLE.
        double objectWidth = getLevelWidth();
        o_vision.setValue("Pickup Target Width", objectWidth);

        double objectBottom = pickupFilteredBottom;
        o_vision.setValue("Pickup Target Bottom", objectBottom);
        double offset = pickupFilteredX;
        boolean aligned = Math.abs(offset)
            <= VISION_CENTER_TOLERANCE_PX * 1.5;
        boolean approachAtBottomEdge = isPickupTargetAtBottomEdge();
        // Switch to the second camera view once the object is low and large
        // enough (firstViewWidthPx of the GROUND/PODIUM profile). Require
        // distinct frames so one noisy contour cannot trigger it.
        // Extra exits keep the robot from standing still:
        // - object at the bottom edge and already wide enough, or the arm is
        //   fully down and the camera cannot look any lower;
        // - object very wide (camera at its upper limit, podium).
        boolean widthReached = objectWidth >= FIRST_VIEW_WIDTH_PX;
        String exitReason = "None";
        if (isPickupZoneReady() && widthReached
            && (objectBottom >= PICKUP_CAMERA_GRAB_BOTTOM_Y_PX
                || isPickupArmAtUpperLimit())) {
          exitReason = "WIDTH_REACHED";
        } else if (approachAtBottomEdge
            && (widthReached || isPickupArmAtLowerLimit())) {
          exitReason = widthReached
              ? "BOTTOM_EDGE_WIDTH_REACHED" : "BOTTOM_EDGE_ARM_DOWN";
        } else if (objectWidth >= VISION_CAMERA_CLOSE_WIDTH_PX) {
          exitReason = "VERY_WIDE";
        }
        boolean cameraGrabReady = aligned && !"None".equals(exitReason);
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
            o_vision.setText("Pickup Exit Reason", exitReason);
            enterVisionPickupState(VisionPickupState.RAISING_FOR_FINAL_VIEW);
          }
          return true;
        }
        visionCameraCloseSince = 0;
        pickupCameraCloseFrames = 0;

        // Slow down over the second half of the approach so the robot
        // reaches the stop width at the lowest speed, without overshoot.
        double slowProgress = clampVision(
            (objectWidth - FIRST_VIEW_WIDTH_PX * 0.5)
                / (FIRST_VIEW_WIDTH_PX * 0.5),
            0.0, 1.0);
        double forward = VISION_APPROACH_SPEED
            + (VISION_APPROACH_SLOW_SPEED - VISION_APPROACH_SPEED)
                * slowProgress;
        if (Math.abs(aimYawError()) > PICKUP_APPROACH_MAX_AIM_ERROR_DEG) {
          // Far off the line to the object: turn in place first.
          o_vision.setText("Pickup Step", "APPROACH_TURNING_TO_OBJECT");
          setPickupDrive(0, aimTurnInPlace(), now);
        } else {
          setPickupDrive(forward, aimSteer(), now);
        }
        return true;

      case RAISING_FOR_FINAL_VIEW:
        o_vision.setText("Pickup Step", "RAISING_LIFT_FOR_SECOND_LOOK");
        stopPickupDrive();
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
        o_subsystem.setServoLift(LIFT_FINAL_VIEW_ANGLE);
        o_subsystem.servo_Hook_Hand(ARM_SCAN_DOWN_ANGLE);
        o_subsystem.servo_Hook(CLAW_OPEN_ANGLE);
        if (!targetVisible || !pickupFilterReady) {
          pickupFilterReady = false;
          visionCameraCloseSince = 0;
          pickupCameraCloseFrames = 0;
          o_vision.setText("Pickup Warning", "WAITING_FOR_SECOND_VIEW");
          if ((pickupLostDirX != 0 || pickupLostDirY != 0)
              && now - visionTargetLastSeen <= PICKUP_LOST_SEARCH_SEC) {
            // Arm is already fully down here, so move the robot instead:
            // lost at the bottom → back off, at the top → creep forward,
            // at a side → turn toward it.
            double creep = -pickupLostDirY * PICKUP_MIN_FORWARD;
            double turn = pickupLostDirX * PICKUP_MIN_TURN;
            setPickupDrive(creep, creep == 0.0 ? turn : turn * 0.5, now);
            o_vision.setText("Pickup Warning", "SECOND_VIEW_LOST_SEARCHING");
            return true;
          }
          stopPickupDrive();
          double lastSeen = Math.max(visionPickupStateStart, visionTargetLastSeen);
          if (now - lastSeen >= PICKUP_SECOND_VIEW_TIMEOUT_SEC) {
            o_vision.setText("Pickup Warning", "SECOND_VIEW_LOST_RESEARCHING");
            enterVisionPickupState(VisionPickupState.RAISING_FOR_SEARCH);
          }
          return true;
        }

        double finalOffset = pickupFilteredX;
        if (Math.abs(finalOffset) > VISION_CENTER_TOLERANCE_PX * 1.5) {
          visionCameraCloseSince = 0;
          pickupCameraCloseFrames = 0;
          o_vision.setText("Pickup Step", "SECOND_VIEW_CENTERING");
          setPickupDrive(0, aimTurnInPlace(), now);
          return true;
        }

        boolean finalCentered = Math.abs(pickupFilteredX)
            <= VISION_CENTER_TOLERANCE_PX * 1.5;
        boolean atBottomEdge = isPickupTargetAtBottomEdge();
        // An object cut by the bottom edge has a truncated center Y, so the
        // center check is replaced by a slightly relaxed width check.
        boolean edgeGrabZone = finalCentered && atBottomEdge
            && pickupFilteredWidth
                >= FINAL_VIEW_WIDTH_PX * PICKUP_EDGE_ACCEPT_WIDTH_FACTOR;
        boolean finalGrabZone = edgeGrabZone
            || (finalCentered
                && pickupFilteredY >= FINAL_VIEW_CENTER_Y_PX
                && pickupFilteredBottom >= FINAL_VIEW_BOTTOM_Y_PX
                && pickupFilteredWidth >= FINAL_VIEW_WIDTH_PX);
        o_vision.setValue("Pickup Second Grab Zone", finalGrabZone ? 1.0 : 0.0);
        if (!finalGrabZone) {
          visionCameraCloseSince = 0;
          pickupCameraCloseFrames = 0;
          if (atBottomEdge) {
            // Too small to grab yet already at the edge: driving on would
            // push it out of view. Back off slowly to see it whole again.
            o_vision.setText("Pickup Warning", "SECOND_VIEW_AT_CAMERA_EDGE");
            setPickupDrive(-PICKUP_EDGE_BACKOFF_SPEED, 0, now);
            return true;
          }

          // Keep the arm fully down and the claw open. Only the wheels move
          // during this second, camera-guided approach.
          double finalSlowdown = clampVision(
              (getPickupBottomEdgeY() - pickupFilteredBottom) / 55.0,
              0.70, 1.0);
          double finalForward = FINAL_APPROACH_SPEED * finalSlowdown;
          o_vision.setText("Pickup Step", "SECOND_VIEW_APPROACHING_OBJECT");
          setPickupDrive(finalForward, aimSteer(), now);
          return true;
        }

        o_vision.setText("Pickup Step", "SECOND_VIEW_CONFIRMING_GRAB_ZONE");
        stopPickupDrive();
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
        o_vision.setText("Pickup Step", "LOWERING_LIFT_BY_CAMERA");
        o_subsystem.servo_Hook(CLAW_OPEN_ANGLE);
        o_subsystem.servo_Hook_Hand(ARM_SCAN_DOWN_ANGLE);
        runCameraDescent(now, targetVisible);
        return true;

      case GRABBING:
        o_vision.setText("Pickup Step", "CLOSING_CLAW");
        stopPickupDrive();
        o_subsystem.setServoLift(pickupGrabLiftAngle);
        o_subsystem.servo_Hook_Hand(ARM_SCAN_DOWN_ANGLE);
        double clawRemaining = Math.abs(
            CLAW_CLOSED_ANGLE - o_subsystem.getServoClawAngle());
        double clawStep = clawRemaining > CLAW_SLOW_ZONE_DEG
            ? SERVO_CLAW_STEP : SERVO_CLAW_SLOW_STEP;
        if (!o_subsystem.moveServoClawToward(CLAW_CLOSED_ANGLE, clawStep)) {
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
        if (!runProfiledServo(true, LIFT_SEARCH_ANGLE,
            PICKUP_LIFT_RAISE_DEG_PER_SEC, now)) {
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
        if (!runProfiledServo(false, ARM_SEARCH_ANGLE,
            PICKUP_ARM_RAISE_DEG_PER_SEC, now)) {
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
    boolean ready = Math.abs(pickupFilteredX)
            <= VISION_CENTER_TOLERANCE_PX * 1.5
        && ((pickupFilteredY >= PICKUP_ZONE_MIN_CENTER_Y_PX
                && pickupFilteredBottom >= PICKUP_ZONE_MIN_BOTTOM_Y_PX)
            // An object on the podium can stay high in the image because
            // the camera cannot tilt above its forward-looking limit.
            || isPickupArmAtUpperLimit());
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
    pickupLastSeenAtBottom = isPickupTargetAtBottomEdge();
    updateLostDirection();
    isPickupZoneReady();
  }

  private double getPickupBottomEdgeY() {
    return o_vision.getVisionFrameHeight() - PICKUP_BOTTOM_EDGE_MARGIN_PX;
  }

  private boolean isPickupTargetAtBottomEdge() {
    return o_vision.isVisionTouchingBottom()
        || pickupFilteredBottom >= getPickupBottomEdgeY();
  }

  /**
   * Moves the lift (or arm) from the angle it had when the current state
   * started to the target with a smoothstep profile. Returns true when done.
   */
  private boolean runProfiledServo(
      boolean lift, double target, double avgDegPerSec, double now) {
    double distance = Math.abs(target - pickupProfileStartAngle);
    double duration = Math.max(0.2, distance / avgDegPerSec);
    double t = clampVision((now - visionPickupStateStart) / duration, 0.0, 1.0);
    double eased = t * t * (3.0 - 2.0 * t);
    double angle = pickupProfileStartAngle
        + (target - pickupProfileStartAngle) * eased;
    if (lift) {
      o_subsystem.setServoLift(angle);
    } else {
      o_subsystem.servo_Hook_Hand(angle);
    }
    return t >= 1.0;
  }

  /**
   * Drives with forward power and rotation only (no strafe). Forward power
   * is ramped up, but never below the power that actually moves the robot.
   */
  private void setPickupDrive(double forward, double turn, double now) {
    double dt = pickupDriveLastAt == 0
        ? 0.02 : clampVision(now - pickupDriveLastAt, 0.0, 0.05);
    if (forward == 0.0) {
      pickupDriveForward = 0.0;
    } else {
      pickupDriveForward += clampVision(forward - pickupDriveForward,
          -PICKUP_DRIVE_DECEL_PER_SEC * dt, PICKUP_DRIVE_ACCEL_PER_SEC * dt);
    }
    pickupDriveLastAt = now;

    double outForward = 0.0;
    if (Math.abs(pickupDriveForward) > 0.01) {
      outForward = Math.copySign(Math.max(Math.abs(pickupDriveForward),
          PICKUP_MIN_FORWARD + pickupStallBoost), pickupDriveForward);
    }
    double outTurn = turn;
    if (outForward == 0.0 && turn != 0.0) {
      outTurn = Math.copySign(
          Math.max(Math.abs(turn), PICKUP_MIN_TURN + pickupStallBoost), turn);
    }
    pickupDriveTurn = outTurn;
    updateStallBoost(outForward != 0.0 || outTurn != 0.0, now);
    o_vision.setValue("Pickup Drive Forward", outForward);
    o_vision.setValue("Pickup Drive Turn", outTurn);
    o_subsystem.driveOmni(outForward, 0, outTurn);
  }

  private void stopPickupDrive() {
    pickupDriveForward = 0;
    pickupDriveTurn = 0;
    pickupDriveLastAt = Timer.getFPGATimestamp();
    pickupStallRefAt = 0;
    o_subsystem.stopDrive();
  }

  /** Raises the minimum power when the encoders show the robot is stuck. */
  private void updateStallBoost(boolean commanded, double now) {
    double left = o_subsystem.getEncoderLeft();
    double right = o_subsystem.getEncoderRight();
    double rear = o_subsystem.getEncoderRear();
    if (!commanded || pickupStallRefAt == 0) {
      pickupStallRefAt = commanded ? now : 0;
      pickupStallRefLeft = left;
      pickupStallRefRight = right;
      pickupStallRefRear = rear;
      return;
    }
    if (now - pickupStallRefAt < PICKUP_STALL_CHECK_SEC) {
      return;
    }
    double travel = Math.abs(left - pickupStallRefLeft)
        + Math.abs(right - pickupStallRefRight)
        + Math.abs(rear - pickupStallRefRear);
    if (travel < PICKUP_STALL_MIN_TRAVEL_MM) {
      pickupStallBoost = Math.min(PICKUP_STALL_BOOST_MAX,
          pickupStallBoost + PICKUP_STALL_BOOST_STEP);
      o_vision.setText("Pickup Warning", "WHEELS_STALLED_ADDING_POWER");
    }
    o_vision.setValue("Pickup Stall Boost", pickupStallBoost);
    pickupStallRefAt = now;
    pickupStallRefLeft = left;
    pickupStallRefRight = right;
    pickupStallRefRear = rear;
  }

  private double getRawPickupOffset() {
    return o_vision.getVisionOffsetX() * VISION_OFFSET_SIGN;
  }

  /** Camera angle below horizontal, degrees (0 = looking straight ahead). */
  private double getCameraDownDeg() {
    double down = (o_subsystem.getServoArmAngle() - ARM_HORIZONTAL_ANGLE)
        + (pickupFilteredY - o_vision.getVisionFrameHeight() / 2.0)
            * CAMERA_DEG_PER_PX_V;
    return clampVision(down, 0.0, 85.0);
  }

  /** Object width as if seen at the camera's height (floor = podium). */
  private double getLevelWidth() {
    double down = getCameraDownDeg();
    double width = pickupFilteredWidth
        / Math.max(MIN_DOWN_COS, Math.cos(Math.toRadians(down)));
    o_vision.setValue("Pickup Camera Down Deg", down);
    o_vision.setValue("Pickup Level Width", width);
    return width;
  }

  private boolean isPickupArmAtLowerLimit() {
    return o_subsystem.getServoArmAngle() >= ARM_SCAN_DOWN_ANGLE - 1.0;
  }

  private boolean isPickupArmAtUpperLimit() {
    return o_subsystem.getServoArmAngle() <= ARM_SEARCH_ANGLE + 1.0;
  }

  private void recordYawHistory() {
    yawHistoryTime[yawHistoryNext] = System.currentTimeMillis() / 1000.0;
    yawHistoryValue[yawHistoryNext] = normalizeYaw(o_subsystem.getYaw());
    yawHistoryNext = (yawHistoryNext + 1) % YAW_HISTORY_SIZE;
    yawHistoryCount = Math.min(YAW_HISTORY_SIZE, yawHistoryCount + 1);
  }

  /** Gyro yaw at a wall-clock time, so the camera delay is compensated. */
  private double yawAtTime(double wallTimeSec) {
    double bestYaw = normalizeYaw(o_subsystem.getYaw());
    double bestDelta = Double.POSITIVE_INFINITY;
    for (int i = 0; i < yawHistoryCount; i++) {
      double delta = Math.abs(yawHistoryTime[i] - wallTimeSec);
      if (delta < bestDelta) {
        bestDelta = delta;
        bestYaw = yawHistoryValue[i];
      }
    }
    return bestYaw;
  }

  /** Converts the newest frame into the heading that points at the object. */
  private void updatePickupAim() {
    double frameYaw = yawAtTime(o_vision.getVisionFrameTimestamp());
    pickupAimYaw = normalizeYaw(
        frameYaw + getRawPickupOffset() * CAMERA_DEG_PER_PX);
    pickupAimValid = true;
    o_vision.setValue("Pickup Aim Yaw", pickupAimYaw);
  }

  /** Signed heading error to the object; positive means turn right. */
  private double aimYawError() {
    if (!pickupAimValid) {
      return 0.0;
    }
    double error = pickupAimYaw - normalizeYaw(o_subsystem.getYaw());
    if (error > 180) error -= 360;
    if (error < -180) error += 360;
    o_vision.setValue("Pickup Aim Error", error);
    return error;
  }

  /** Rotation for turning in place toward the object; 0 when aimed. */
  private double aimTurnInPlace() {
    double error = aimYawError();
    if (Math.abs(error) <= PICKUP_AIM_TOLERANCE_DEG) {
      return 0.0;
    }
    return clampVision(error * PICKUP_TURN_KP,
        -PICKUP_MAX_TURN, PICKUP_MAX_TURN);
  }

  /** Gentle steering while driving forward. */
  private double aimSteer() {
    double error = aimYawError();
    if (Math.abs(error) <= PICKUP_AIM_TOLERANCE_DEG * 0.5) {
      return 0.0;
    }
    return clampVision(error * PICKUP_STEER_KP,
        -PICKUP_MAX_STEER, PICKUP_MAX_STEER);
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
        pickupTrackedArmTarget, PICKUP_ARM_TRACK_STEP);
    o_vision.setValue("Pickup Arm Target", pickupTrackedArmTarget);
  }

  private void followPickupTarget(double now) {
    // Only the camera arm follows the object here. The lift height is no
    // longer guessed in advance: the descent before the grab is measured by
    // the camera (runCameraDescent).
    followPickupArm(now);
  }

  /**
   * Lowers the lift while the down-looking camera watches the object. The
   * object grows in the image as the claw gets closer, so its width tells
   * when to stop. Works for any object height (floor, podium, ...).
   */
  private void runCameraDescent(double now, boolean targetVisible) {
    double lift = o_subsystem.getServoLiftAngle();
    boolean atLowest = lift >= LIFT_LOWEST_ANGLE - 0.5
        || o_subsystem.isLiftLowerLimitPressed();
    o_vision.setValue("Pickup Descent Lift", lift);

    if (!Double.isNaN(pickupBlindLiftTarget)) {
      stopPickupDrive();
      boolean done = o_subsystem.moveServoLiftToward(
          pickupBlindLiftTarget, DESCENT_MAX_STEP);
      if (done || atLowest) {
        finishCameraDescent("PREDICTED_AFTER_LOSS");
      }
      return;
    }

    if (targetVisible && pickupFilterReady) {
      double width = pickupFilteredWidth;
      pickupDescentLastWidth = width;
      if (pickupFrameUpdated) {
        addDescentSample(lift, width);
      }
      o_vision.setValue("Pickup Descent Width", width);
      boolean sizeReached = width >= GRAB_WIDTH_PX
          || width >= o_vision.getVisionFrameWidth() * GRAB_FRAME_FILL;

      // 1) Keep the object where the claw is: turn for X, creep for Y.
      //    The lift waits while the robot moves.
      double turn = Math.abs(getRawPickupOffset())
          > VISION_CENTER_TOLERANCE_PX * 1.5 ? aimTurnInPlace() : 0.0;
      double yError = pickupFilteredY - GRAB_TARGET_Y_PX;
      if (Math.abs(yError) > GRAB_Y_DEADBAND_PX) {
        // Driving forward moves the object down the image (Y grows).
        double creep = yError < 0 ? PICKUP_MIN_FORWARD : -PICKUP_MIN_FORWARD;
        setPickupDrive(creep, turn * 0.5, now);
        o_vision.setText("Pickup Warning",
            yError < 0 ? "DESCENT_CREEP_FORWARD" : "DESCENT_CREEP_BACK");
        return;
      }
      if (turn != 0.0) {
        setPickupDrive(0, turn, now);
        o_vision.setText("Pickup Warning", "DESCENT_TURNING_TO_CENTER");
        return;
      }
      stopPickupDrive();
      o_vision.setText("Pickup Warning", "None");

      // 2) Aligned: stop at the grab size or keep going down.
      if (sizeReached) {
        finishCameraDescent("CAMERA_SIZE_REACHED");
        return;
      }
      if (atLowest) {
        finishCameraDescent("LIFT_AT_LOWEST");
        return;
      }
      double remaining = clampVision(1.0 - width / GRAB_WIDTH_PX, 0.0, 1.0);
      double step = DESCENT_MIN_STEP
          + (DESCENT_MAX_STEP - DESCENT_MIN_STEP) * remaining;
      o_subsystem.setServoLift(Math.min(LIFT_LOWEST_ANGLE, lift + step));
      return;
    }

    stopPickupDrive();
    if (atLowest) {
      finishCameraDescent("LIFT_AT_LOWEST");
      return;
    }
    // Lost from view during the descent. Predict from how the object grew
    // where the lift has to stop, and finish there without the camera.
    double predicted = predictGrabLiftAngle();
    o_vision.setValue("Pickup Predicted Lift", Double.isNaN(predicted) ? -1 : predicted);
    if (!Double.isNaN(predicted) && predicted > lift) {
      pickupBlindLiftTarget = Math.min(LIFT_LOWEST_ANGLE, predicted);
      o_vision.setText("Pickup Warning", "LOST_LOWERING_TO_PREDICTED_LIFT");
      return;
    }
    if (pickupDescentLastWidth >= GRAB_WIDTH_PX * DESCENT_LOST_ACCEPT_FACTOR) {
      pickupBlindLiftTarget = Math.min(LIFT_LOWEST_ANGLE,
          lift + DESCENT_BLIND_EXTRA_DEG);
      o_vision.setText("Pickup Warning", "TOO_CLOSE_TO_SEE_FINISHING_BLIND");
      return;
    }
    o_vision.setText("Pickup Warning", "DESCENT_WAITING_FOR_OBJECT");
    double lastSeen = Math.max(visionPickupStateStart, visionTargetLastSeen);
    if (now - lastSeen >= DESCENT_LOST_TIMEOUT_SEC) {
      o_vision.setText("Pickup Warning", "DESCENT_LOST_RESEARCHING");
      enterVisionPickupState(VisionPickupState.RAISING_FOR_SEARCH);
    }
  }

  private void addDescentSample(double lift, double width) {
    if (width <= 1.0) {
      return;
    }
    descentSampleLift[descentSampleNext] = lift;
    descentSampleInvWidth[descentSampleNext] = 1.0 / width;
    descentSampleNext = (descentSampleNext + 1) % DESCENT_SAMPLE_SIZE;
    descentSampleCount = Math.min(DESCENT_SAMPLE_SIZE, descentSampleCount + 1);
  }

  /**
   * Apparent width is inversely proportional to distance, so 1/width falls
   * roughly linearly as the lift goes down. A line through the recent
   * samples gives the lift angle where width reaches GRAB_WIDTH_PX.
   * Returns NaN when there is not enough reliable data.
   */
  private double predictGrabLiftAngle() {
    if (descentSampleCount < DESCENT_MIN_SAMPLES) {
      return Double.NaN;
    }
    double meanX = 0, meanY = 0;
    for (int i = 0; i < descentSampleCount; i++) {
      meanX += descentSampleLift[i];
      meanY += descentSampleInvWidth[i];
    }
    meanX /= descentSampleCount;
    meanY /= descentSampleCount;
    double sxx = 0, sxy = 0;
    for (int i = 0; i < descentSampleCount; i++) {
      double dx = descentSampleLift[i] - meanX;
      sxx += dx * dx;
      sxy += dx * (descentSampleInvWidth[i] - meanY);
    }
    // Need a few degrees of lift travel and a shrinking 1/width.
    if (sxx < 4.0 * descentSampleCount || sxy >= 0) {
      return Double.NaN;
    }
    double slope = sxy / sxx;
    double intercept = meanY - slope * meanX;
    return (1.0 / GRAB_WIDTH_PX - intercept) / slope
        + DESCENT_PREDICT_MARGIN_DEG;
  }

  private void finishCameraDescent(String reason) {
    pickupGrabLiftAngle = o_subsystem.getServoLiftAngle();
    o_vision.setText("Pickup Descent Stop", reason);
    o_vision.setValue("Pickup Grab Lift Angle", pickupGrabLiftAngle);
    enterVisionPickupState(VisionPickupState.GRABBING);
  }

  private boolean keepVisionTargetOrWait(double now, boolean targetVisible) {
    if (targetVisible) {
      return true;
    }

    pickupFilterReady = false;
    pickupAimValid = false;
    visionCenteredSince = 0;
    visionCameraCloseSince = 0;
    pickupCameraCloseFrames = 0;
    double lostFor = now - visionTargetLastSeen;
    if (lostFor <= PICKUP_LOST_SEARCH_SEC && searchTowardLostSide(now)) {
      return false;
    }
    stopPickupDrive();
    if (lostFor > VISION_TARGET_LOST_TIMEOUT_SEC) {
      enterVisionPickupState(VisionPickupState.IDLE);
    }
    return false;
  }

  /** Remembers which outer part of the frame the object was last seen in. */
  private void updateLostDirection() {
    double width = o_vision.getVisionFrameWidth();
    double height = o_vision.getVisionFrameHeight();
    double offsetX = getRawPickupOffset();
    double y = o_vision.getVisionTargetY();
    double bottom = o_vision.getVisionTargetBottomY();

    if (offsetX > width * (0.5 - LOST_EDGE_ZONE)) {
      pickupLostDirX = 1;
    } else if (offsetX < -width * (0.5 - LOST_EDGE_ZONE)) {
      pickupLostDirX = -1;
    } else {
      pickupLostDirX = 0;
    }

    if (o_vision.isVisionTouchingBottom()
        || bottom >= height * (1.0 - LOST_EDGE_ZONE * 0.5)
        || y >= height * (1.0 - LOST_EDGE_ZONE)) {
      pickupLostDirY = 1;
    } else if (y <= height * LOST_EDGE_ZONE) {
      pickupLostDirY = -1;
    } else {
      pickupLostDirY = 0;
    }
  }

  /**
   * Moves the camera/robot toward the side where the object disappeared.
   * Returns false when there is nothing left to do in that direction.
   */
  private boolean searchTowardLostSide(double now) {
    boolean acting = false;
    String where = "";

    if (pickupLostDirY != 0) {
      double limit = pickupLostDirY > 0 ? ARM_SCAN_DOWN_ANGLE : ARM_SEARCH_ANGLE;
      double arm = o_subsystem.getServoArmAngle();
      if (Math.abs(limit - arm) > 0.5) {
        pickupTrackedArmTarget = clampVision(
            arm + pickupLostDirY * PICKUP_LOST_ARM_STEP,
            ARM_SEARCH_ANGLE, ARM_SCAN_DOWN_ANGLE);
        o_subsystem.servo_Hook_Hand(pickupTrackedArmTarget);
        acting = true;
      }
      where += pickupLostDirY > 0 ? "DOWN" : "UP";
    }

    if (pickupLostDirX != 0) {
      setPickupDrive(0, pickupLostDirX * PICKUP_MIN_TURN, now);
      acting = true;
      where += pickupLostDirX > 0 ? "_RIGHT" : "_LEFT";
    } else {
      stopPickupDrive();
    }

    if (acting) {
      o_vision.setText("Pickup Warning", "LOST_SEARCHING_" + where);
    }
    return acting;
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
      // A pure turn uses every wheel. Positive turn commands
      // left +, right -, rear -; negative turn reverses all three.
      o_subsystem.driveOmni(0, 0, Math.copySign(speed, error));
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
    if (Math.abs(offsetX) > tolerance) {
      o_subsystem.driveOmni(0, 0, Math.copySign(speed, offsetX));
    } else {
      o_subsystem.TimerStop();
      stateAutomatic++;
      return;
    }
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
        o_subsystem.driveOmni(0, 0, rotateSpeed);
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
