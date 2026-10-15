package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.util.Tunable;
import java.io.FileWriter;
import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

/*
 * ===================== ИНСТРУКЦИЯ: АВТОМАТИЧЕСКИЙ ЗАХВАТ =====================
 *
 * visionPickup() делает всё сам по камере, без ручных профилей:
 *
 *  1. ПОИСК (IDLE): рука качается вверх-вниз, ищет объект. Пятно шире 70%
 *     кадра (освещённый подиум) целью не считается.
 *  2. ПЕРВАЯ ФАЗА (CENTERING / APPROACHING): доворот на объект по гироскопу,
 *     подъезд, рука-камера следит за объектом. Останов, когда объект шириной
 *     FIRST_VIEW_WIDTH_PX (с поправкой на наклон камеры).
 *  3. ЗАМЕР ВЫСОТЫ (MEASURING_HEIGHT): робот стоит, камера считает индекс
 *     высоты. Меньше HEIGHT_INDEX_PODIUM — подиум, больше HEIGHT_INDEX_FLOOR —
 *     пол. Индекс видно в "Pickup Height Index" и в журнале.
 *  4. ПРЯМОЙ ЗАХВАТ, всегда первым: рука прямо (ARM_FRONT_GRAB_ANGLE),
 *     лифт опускается (FRONT_SEARCH_LIFT_STEP), пока камера НЕ видит объект.
 *     Как только видит (FRONT_SEEN_CONFIRM_FRAMES кадров подряд) — лифт
 *     стоит, робот центрируется и подъезжает, пока объект не станет шириной
 *     FRONT_GRAB_WIDTH_PX (больше = ближе), затем клешня закрывается (можно
 *     опустить ещё на FRONT_LIFT_EXTRA_DOWN_DEG). Дошёл до LIFT_LOWEST_ANGLE
 *     и не увидел — переход к захвату сверху.
 *  5. ПРОВЕРКА (VERIFYING_GRAB): клешня закрыта, робот отъезжает на
 *     VERIFY_BACKOFF_MM. Объект в клешне не уменьшается в кадре — успех.
 *     Промах — клешня открывается и всё заново, но с захватом СВЕРХУ.
 *  6. ЗАХВАТ СВЕРХУ (запасной): рука вниз, подъезд, пока верхняя грань
 *     объекта не ниже SECOND_VIEW_MIN_TOP_Y, лифт опускается до угла по
 *     высоте (LIFT_GRAB_ANGLE_FLOOR ... LIFT_GRAB_ANGLE_PODIUM).
 *
 * НАСТРОЙКА БЕЗ ПЕРЕЗАЛИВКИ:
 *  - Вкладка Shuffleboard "Pickup Tuning": все параметры ниже, которые
 *    объявлены как Tunable. Меняйте там на лету.
 *  - Значения из Shuffleboard сбрасываются при перезапуске программы:
 *    подобрали — перепишите число в код (второй аргумент new Tunable(...)).
 *  - Что происходит сейчас — вкладка "Vision": "Pickup State",
 *    "Pickup Step", "Pickup Warning", "Pickup Grab Mode".
 *  - Журнал каждого захвата: /home/lvuser/pickup_log.csv
 *    (scp pi@raspberrypi.local:/home/lvuser/pickup_log.csv .)
 *
 * ЧТО КРУТИТЬ, ЕСЛИ...
 *  - останавливается слишком далеко / близко перед объектом:
 *      FIRST_VIEW_WIDTH_PX меньше = дальше, больше = ближе;
 *  - едет медленно / не трогается:   PICKUP_MIN_FORWARD, PICKUP_MIN_TURN;
 *  - толкает объект в первой фазе:  PUSH_CHECK_MM меньше (раньше заметит,
 *      что объект не растёт в кадре, и остановится);
 *  - путает пол и подиум:           HEIGHT_INDEX_PODIUM / HEIGHT_INDEX_FLOOR
 *      (возьмите "Pickup Height Index" для пола и для подиума, порог между);
 *  - прямой захват: клешня закрывается слишком высоко -> FRONT_LIFT_EXTRA_DOWN_DEG
 *      больше (лифт опустится ещё на столько градусов после того, как увидел);
 *  - прямой захват: лифт опускается слишком медленно/быстро -> FRONT_SEARCH_LIFT_STEP;
 *  - прямой захват срабатывает на ложные блики -> FRONT_SEEN_CONFIRM_FRAMES больше;
 *  - прямой захват: подъехать ближе/дальше перед клешнёй -> FRONT_GRAB_WIDTH_PX
 *      больше/меньше ("Pickup Front Width" с объектом в клешне);
 *  - прямой захват не дотягивается вниз -> LIFT_LOWEST_ANGLE больше;
 *  - захват сверху: лифт не доходит / упирается -> LIFT_GRAB_ANGLE_FLOOR,
 *      LIFT_GRAB_ANGLE_PODIUM;
 *  - захват сверху хватает рано -> SECOND_VIEW_MIN_TOP_Y больше;
 *  - клешня не дожимает / не открывается -> CLAW_CLOSED_ANGLE / CLAW_OPEN_ANGLE.
 * =============================================================================
 */
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
    MEASURING_HEIGHT,
    VERIFYING_GRAB,
    FRONT_ALIGNING,
    FRONT_APPROACHING,
    GRABBING,
    RAISING_LIFT,
    RAISING_ARM,
    COMPLETE
  }

  // ── Pickup geometry (one set for every object height) ──
  // First view (camera forward, lift up): approach until the object is this
  // wide in the image, then switch to the second view with the arm down.:
  private static final Tunable FIRST_VIEW_WIDTH_PX = new Tunable("FIRST_VIEW_WIDTH_PX", 110.0);
  // Second view (arm down, lift up): drive until the object reaches the
  // bottom edge of the camera frame (since the camera is mounted ABOVE the claw).
  private static final double FINAL_VIEW_CENTER_Y_PX = 195.0;
  private static final double FINAL_VIEW_WIDTH_PX = 110.0;
  // ── Object height measured by the camera ──
  // After the first phase the robot stops, the arm freezes and the camera
  // takes a "height index" averaged over several frames:
  //   index = 1000 * sin(angle down to the object's TOP edge) / object width
  // The top edge is used because the bottom edge is usually cut off by the
  // frame. Dividing by the width removes the effect of the stop distance,
  // which varied between runs and made the plain angle useless
  // (podium 45..54 deg overlapped the floor 58 deg).
  // Measured on the robot (pickup_log.csv): podium 5.35..5.68, floor
  // 6.30..9.67. Between the two thresholds the lift angle is interpolated;
  // a doubtful object is treated as the podium, so the lift can never be
  // driven into the podium (worst case it stops short above the floor).
  private static final Tunable HEIGHT_INDEX_PODIUM = new Tunable("HEIGHT_INDEX_PODIUM", 5.9);
  private static final Tunable HEIGHT_INDEX_FLOOR = new Tunable("HEIGHT_INDEX_FLOOR", 6.2);
  // Far below the podium value = object higher than the podium: front grab.
  private static final Tunable HEIGHT_INDEX_TOO_HIGH = new Tunable("HEIGHT_INDEX_TOO_HIGH", 3.5);
  private static final double MEASURE_SETTLE_SEC = 0.4;
  private static final int MEASURE_FRAMES = 6;
  private static final double MEASURE_TIMEOUT_SEC = 1.5;
  // Above this the measurement is nonsense (66.97 in the log came from a
  // frame-filling false target). Such a result restarts the search.
  private static final double HEIGHT_INDEX_MAX_VALID = 15.0;
  // While searching, a "target" wider than this part of the frame is not an
  // object but a lit surface (the white podium looked like a 319 px
  // "Yellow ball" in the log). It is never locked as a target.
  private static final double SEARCH_MAX_TARGET_WIDTH_FRACTION = 0.7;
  // Second view: the claw is under the lower part of the image. The grab
  // zone needs the object's TOP edge inside the frame and at least this
  // low (successful floor grabs: top edge ~155 px). An object touching the
  // top edge (e.g. a cube filling the whole frame on the podium) is still
  // ahead of the claw, so the robot keeps creeping forward.
  private static final Tunable SECOND_VIEW_MIN_TOP_Y = new Tunable("SECOND_VIEW_MIN_TOP_Y", 130.0);
  private static final Tunable SECOND_VIEW_MAX_APPROACH_MM = new Tunable("SECOND_VIEW_MAX_APPROACH_MM", 300.0);
  // Second view on the podium: with the arm down the object sits at the top
  // edge of the image (y ~ 20 px in the log) and is barely visible. The
  // robot then creeps forward until it comes into view, at most this far.
  private static final double SECOND_VIEW_MAX_CREEP_MM = 200.0;
  private static final double SECOND_VIEW_TOP_ZONE = 0.35;
  // Lift angle at which the open claw surrounds the object (tested values).
  private static final Tunable LIFT_GRAB_ANGLE_FLOOR = new Tunable("LIFT_GRAB_ANGLE_FLOOR", 160.0);
  private static final Tunable LIFT_GRAB_ANGLE_PODIUM = new Tunable("LIFT_GRAB_ANGLE_PODIUM", 50.0);
  // Second view: how low in the image the object bottom must be when the
  // claw is above it. A closer (podium) object looks bigger and reaches the
  // same line earlier, so it needs a lower line (tested values).
  private static final double SECOND_VIEW_BOTTOM_Y_FLOOR = 225.0;
  private static final double SECOND_VIEW_BOTTOM_Y_PODIUM = 235.0;
  // Lowest the lift may go in any pickup mode (bigger = lower). This is a
  // hard limit: 120 here kept the lift from going lower. The physical end
  // is 185 (Teleop drives it there).
  private static final Tunable LIFT_LOWEST_ANGLE = new Tunable("LIFT_LOWEST_ANGLE", 300.0);
  // ── Front grab (always tried first) ──
  // The arm goes straight, then the lift goes down while the object is NOT
  // in view. The moment the camera (on the lift, above the claw) sees the
  // object, the claw closes. Lift at its lowest and still nothing seen:
  // switch to the grab from above.
  // Arm angle at which the arm and claw point straight ahead.
  private static final Tunable ARM_FRONT_GRAB_ANGLE = new Tunable("ARM_FRONT_GRAB_ANGLE", 80.0);
  // Lift degrees per 20 ms cycle while lowering to find the object.
  private static final Tunable FRONT_SEARCH_LIFT_STEP = new Tunable("FRONT_SEARCH_LIFT_STEP", 0.8);
  // Consecutive camera frames with the object before the claw closes
  // (2 = ignore a single false flash).
  private static final Tunable FRONT_SEEN_CONFIRM_FRAMES = new Tunable("FRONT_SEEN_CONFIRM_FRAMES", 2.0);
  // After the object is seen the lift stops; the robot centers on it and
  // drives until it is this wide in the image (bigger = closer), then the
  // claw closes.
  private static final Tunable FRONT_GRAB_WIDTH_PX = new Tunable("FRONT_GRAB_WIDTH_PX", 197.0);
  // Lost during the approach while already this close (part of the grab
  // width): it is in front of the claw, close it.
  private static final double FRONT_LOST_ACCEPT_FACTOR = 0.8;
  // Extra degrees the lift goes down after the object is seen, before the
  // claw closes (0 = close right where it was seen).
  private static final Tunable FRONT_LIFT_EXTRA_DOWN_DEG = new Tunable("FRONT_LIFT_EXTRA_DOWN_DEG", 0.0);
  // Wait after straightening the arm so old frames are not used.
  private static final double FRONT_ARM_SETTLE_SEC = 0.4;
  // ── Order of grabs: straight first, from above as the second try ──
  // After a straight grab the robot checks whether the object is really in
  // the claw: it backs off a little with the claw closed. An object in the
  // claw moves with the robot, so its size in the image stays the same; an
  // object left behind gets smaller or shows up ahead. On a miss the claw
  // opens and the whole pickup restarts with the grab from above.
  private static final Tunable VERIFY_BACKOFF_MM = new Tunable("VERIFY_BACKOFF_MM", 80.0);
  private static final double VERIFY_BACKOFF_TIMEOUT_SEC = 2.0;
  private static final double VERIFY_SETTLE_SEC = 0.3;
  private static final int VERIFY_FRAMES = 4;
  private static final double VERIFY_SAMPLE_TIMEOUT_SEC = 0.8;
  // Held if the width after backing off is at least this part of before.
  private static final Tunable VERIFY_HELD_WIDTH_RATIO = new Tunable("VERIFY_HELD_WIDTH_RATIO", 0.8);
  // Lift step per 20 ms cycle: fast far away, slow for the last degrees.
  private static final Tunable DESCENT_MAX_STEP = new Tunable("DESCENT_MAX_STEP", 0.4);
  private static final double DESCENT_MIN_STEP = 0.12;
  private static final double DESCENT_SLOW_ZONE_DEG = 20.0;
  // The lower limit switch is trusted only after the lift has really gone
  // down this far, so a miswired or noisy switch cannot stop it at the top.
  private static final double LIFT_LIMIT_SWITCH_MIN_ANGLE = 50.0;

  private static final Tunable VISION_CENTER_TOLERANCE_PX = new Tunable("VISION_CENTER_TOLERANCE_PX", 14.0);
  private static final double PICKUP_FRAME_MAX_AGE_SEC = 0.35;
  private static final double PICKUP_FILTER_ALPHA = 0.5;
  private static final double PICKUP_DRIVE_ACCEL_PER_SEC = 0.50;
  private static final double PICKUP_DRIVE_DECEL_PER_SEC = 1.50;
  private static final double VISION_TARGET_LOST_TIMEOUT_SEC = 0.75;
  private static final double VISION_CENTER_HOLD_SEC = 0.25;
  private static final double VISION_TARGET_CONFIRM_SEC = 0.15;
  private static final double VISION_CAMERA_CLOSE_CONFIRM_SEC = 0.30;
  private static final double VISION_CAMERA_CLOSE_WIDTH_PX = 300.0;
  private static final double PICKUP_ZONE_MIN_CENTER_Y_PX = 180.0;
  private static final double PICKUP_ZONE_MIN_BOTTOM_Y_PX = 205.0;
  // Stay in the first camera-guided approach until the object reaches the
  // bottom edge of the frame and the target width, then switch to second view.
  private static final double PICKUP_CAMERA_GRAB_BOTTOM_Y_PX = 215.0;
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
  private static final Tunable ARM_HORIZONTAL_ANGLE = new Tunable("ARM_HORIZONTAL_ANGLE", 80.0);
  private static final double CAMERA_DEG_PER_PX_V = CAMERA_DEG_PER_PX;
  // At most x2: a steep camera made a small, far object look wide enough
  // and the robot jumped to the second phase right after seeing it.
  private static final double MIN_DOWN_COS = 0.5;
  // Any exit from the first phase also needs the object itself to be at
  // least this part of FIRST_VIEW_WIDTH_PX in the image (really close).
  private static final double EXIT_MIN_RAW_WIDTH_FACTOR = 0.6;
  // Pushing detection: the robot drove this far forward while the object
  // hardly grew in the image -> the bumper is pushing it along (seen on the
  // floor: the cube moved with the robot and never reached the stop width).
  // Stop and go on to the second phase. The window must be long enough for
  // the filtered width (~0.2 s lag) to show a real approach: 60 mm made it
  // trigger falsely in front of the podium.
  private static final Tunable PUSH_CHECK_MM = new Tunable("PUSH_CHECK_MM", 120.0);
  private static final double PUSH_MIN_GROWTH = 0.03;
  // ── Driving sideways by the side ultrasonics ──
  // Strafe power, slower for the last SIDE_SLOW_ZONE_CM, and gyro heading
  // hold so the robot goes strictly sideways instead of turning.
  private static final Tunable SIDE_STRAFE_SPEED = new Tunable("SIDE_STRAFE_SPEED", 0.35);
  private static final Tunable SIDE_STRAFE_SLOW_SPEED = new Tunable("SIDE_STRAFE_SLOW_SPEED", 0.22);
  private static final double SIDE_SLOW_ZONE_CM = 10.0;
  private static final double SIDE_HEADING_KP = 0.01;
  private static final double SIDE_HEADING_MAX_TURN = 0.12;
  private static final double PUSH_MIN_WIDTH_FACTOR = 0.5;
  // Every pickup state change is appended here (copy it off the robot to
  // see exactly where a pickup went wrong).
  private static final String PICKUP_LOG_PATH = "/home/lvuser/pickup_log.csv";
  private static final double PICKUP_AIM_TOLERANCE_DEG = 1.5;
  // Turning in place: same 0.15..0.30 power range as rotateTheRobot().
  private static final double PICKUP_TURN_KP = 0.010;
  private static final Tunable PICKUP_MIN_TURN = new Tunable("PICKUP_MIN_TURN", 0.08);
  private static final Tunable PICKUP_MAX_TURN = new Tunable("PICKUP_MAX_TURN", 0.20);
  // Steering while driving forward.
  private static final double PICKUP_STEER_KP = 0.012;
  private static final double PICKUP_MAX_STEER = 0.12;
  // Drive straight only when nearly aimed; otherwise turn in place first.
  private static final double PICKUP_APPROACH_MAX_AIM_ERROR_DEG = 6.0;
  // The route finishes every move at minSpeed = 0.2, so below that the
  // robot may not move at all. That was the "drives up and just stands".
  private static final Tunable PICKUP_MIN_FORWARD = new Tunable("PICKUP_MIN_FORWARD", 0.20);
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
  private static final Tunable VISION_GRIP_DELAY_SEC = new Tunable("VISION_GRIP_DELAY_SEC", 1.50);
  private static final double SERVO_CALIBRATION_SETTLE_SEC = 1.50;
  private static final double LIFT_LOWER_SETTLE_SEC = 0.25;
  private static final double ARM_LOWER_SETTLE_SEC = 0.50;
  private static final double FINAL_VIEW_CONFIRM_SEC = 0.30;
  private static final int FINAL_VIEW_CONFIRM_FRAMES = 3;
  private static final Tunable FINAL_APPROACH_SPEED = new Tunable("FINAL_APPROACH_SPEED", 0.20);
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
  private static final Tunable VISION_APPROACH_SPEED = new Tunable("VISION_APPROACH_SPEED", 0.13);
  private static final Tunable VISION_APPROACH_SLOW_SPEED = new Tunable("VISION_APPROACH_SLOW_SPEED", 0.09);
  private static final double PICKUP_SCAN_STEP_DEG = 5.0;
  private static final double PICKUP_SCAN_DWELL_SEC = 0.45;
  private static final double PICKUP_ARM_UPDATE_SEC = 0.05;
  // During approach keep the target's bottom edge near the bottom edge of the 240 px frame.
  private static final double PICKUP_IMAGE_CENTER_Y_PX = 215.0;
  private static final double PICKUP_IMAGE_Y_DEADBAND_PX = 14.0;
  private static final double PICKUP_ARM_KP = 0.015;
  private static final double PICKUP_ARM_MAX_CORRECTION_DEG = 1.2;
  // User-calibrated physical positions. Do not infer direction from the
  // numeric angle or swap these values automatically.
  private static final Tunable CLAW_OPEN_ANGLE = new Tunable("CLAW_OPEN_ANGLE", 18);
  private static final Tunable CLAW_CLOSED_ANGLE = new Tunable("CLAW_CLOSED_ANGLE", 170);
  // 80 degrees looks forward; increasing the angle aims down at the floor.
  // The arm servo's 15-degree safety margin clamps its 180-degree endpoint
  // to 165 degrees in ExampleSubsystem.
  private static final double ARM_SEARCH_ANGLE = 80.0;
  private static final Tunable ARM_SCAN_DOWN_ANGLE = new Tunable("ARM_SCAN_DOWN_ANGLE", 165.0);
  private static final int LIFT_SEARCH_ANGLE = 15;
  // Lift position during the second camera-guided approach; tune separately.
  private static final int LIFT_FINAL_VIEW_ANGLE = 15;

  private VisionPickupState visionPickupState = VisionPickupState.IDLE;
  private double pickupGrabLiftAngle = LIFT_SEARCH_ANGLE;
  // 0 = floor, 1 = podium, measured at the end of the first phase.
  private double pickupHeightFactor = 0.0;
  private double pickupSecondViewStartMm = 0;
  private double pickupSecondViewLastTopY = -1;
  private double pickupRawLiftGrabAngle = LIFT_GRAB_ANGLE_FLOOR.get();
  private double pickupHeightIndexSum = 0;
  private int pickupHeightIndexCount = 0;
  private double pickupGrabArmAngle = ARM_SCAN_DOWN_ANGLE.get();
  private double pickupFrontArmAngle = ARM_FRONT_GRAB_ANGLE.get();
  private boolean pickupGrabIsFront = false;
  // Straight grab already tried and missed in this pickup: use from above.
  private boolean pickupFrontFailed = false;
  private double frontArmStraightSince = 0;
  private boolean pushRefValid = false;
  private double pushRefMm = 0;
  private double pushRefWidth = 0;
  private int frontSeenFrames = 0;
  private double frontLastWidth = 0;
  private int verifyStage = 0;
  private double verifyStageStart = 0;
  private double verifyWidthSum = 0;
  private int verifyWidthCount = 0;
  private double verifyWidthBefore = 0;
  private boolean verifyVisibleBefore = false;
  private double verifyStartMm = 0;
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
    // Create the calibration fields right away, not only when a pickup
    // reaches that step.
    o_vision.setValue("Pickup Front Width", 0.0);
    o_vision.setValue("Pickup Height Index", 0.0);
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
          goLeftSonic(7);
          break;
        case 1:
          goForwardDistance(100);
          break;
        case 2:
          visionPickup();
          break;
        case 3:
          goBackDistance(60);
          break;
        case 4:
          rotateTheRobot(90);
          break;
        case 5:
          goRightSonic(10);
          break;
        case 6:
          o_subsystem.servo_Hook(200);
          stateAutomatic++;
          break;
        case 7:
        // goBackSonic(7);
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

  /**
   * True when the robot has driven PUSH_CHECK_MM forward but the object
   * grew less than PUSH_MIN_GROWTH in the image: it is being pushed.
   */
  private boolean isPushingObject() {
    double distance = o_subsystem.getAverageDriveDistance();
    // Far away the object grows only a few percent per 60 mm; check only
    // when it is already fairly close.
    if (pickupFilteredWidth < FIRST_VIEW_WIDTH_PX.get() * PUSH_MIN_WIDTH_FACTOR) {
      pushRefValid = false;
      return false;
    }
    if (pickupDriveForward <= 0.01 || !pickupFrameUpdated) {
      if (pickupDriveForward <= 0.01) {
        pushRefValid = false;
      }
      return false;
    }
    if (!pushRefValid) {
      pushRefValid = true;
      pushRefMm = distance;
      pushRefWidth = pickupFilteredWidth;
      return false;
    }
    if (Math.abs(distance - pushRefMm) < PUSH_CHECK_MM.get()) {
      return false;
    }
    boolean pushing = pickupFilteredWidth < pushRefWidth * (1.0 + PUSH_MIN_GROWTH);
    o_vision.setValue("Pickup Push Growth",
        pushRefWidth > 0 ? pickupFilteredWidth / pushRefWidth : 0);
    pushRefMm = distance;
    pushRefWidth = pickupFilteredWidth;
    return pushing;
  }

  private boolean isRawWidthCloseEnough() {
    return pickupFilteredWidth
        >= FIRST_VIEW_WIDTH_PX.get() * EXIT_MIN_RAW_WIDTH_FACTOR;
  }

  /** Appends one line per state change; never stops the robot on errors. */
  private void logPickup(String event) {
    boolean newFile = !new java.io.File(PICKUP_LOG_PATH).exists();
    try (FileWriter out = new FileWriter(PICKUP_LOG_PATH, true)) {
      if (newFile) {
        out.write("time,autoCase,state,event,color,width,levelWidth,offsetX,"
            + "y,bottomY,arm,lift,cameraDownDeg,heightFactor,neededLift\n");
      }
      out.write(String.format(java.util.Locale.ROOT,
          "%.2f,%d,%s,%s,%s,%.0f,%.0f,%.0f,%.0f,%.0f,%.1f,%.1f,%.1f,%.2f,%.1f%n",
          Timer.getFPGATimestamp(), stateAutomatic, visionPickupState.name(),
          event, pickupLockedColor,
          pickupFilteredWidth, getLevelWidth(), pickupFilteredX,
          pickupFilteredY, pickupFilteredBottom,
          o_subsystem.getServoArmAngle(), o_subsystem.getServoLiftAngle(),
          getCameraDownDeg(), pickupHeightFactor, pickupRawLiftGrabAngle));
    } catch (IOException | RuntimeException ignored) {
      // Logging is diagnostics only.
    }
  }

  private void enterVisionPickupState(VisionPickupState state) {
    logPickup("-> " + state.name());
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
    if (state == VisionPickupState.VERIFYING_GRAB) {
      verifyStage = 0;
      verifyStageStart = visionPickupStateStart;
      verifyWidthSum = 0;
      verifyWidthCount = 0;
    }
    if (state == VisionPickupState.APPROACHING) {
      pushRefValid = false;
    }
    if (state == VisionPickupState.MEASURING_HEIGHT) {
      pickupHeightIndexSum = 0;
      pickupHeightIndexCount = 0;
    }
    if (state == VisionPickupState.FRONT_APPROACHING) {
      frontLastWidth = pickupFilteredWidth;
      pushRefValid = false;
    }
    if (state == VisionPickupState.FRONT_ALIGNING) {
      frontArmStraightSince = 0;
      frontSeenFrames = 0;
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
      // Where the object went while the arm swung down: near the top edge
      // means it is still ahead of the claw.
      pickupLostDirY = pickupFilteredBottom
          < o_vision.getVisionFrameHeight() * SECOND_VIEW_TOP_ZONE ? -1 : 0;
      pickupSecondViewStartMm = o_subsystem.getAverageDriveDistance();
      pickupSecondViewLastTopY = -1;
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
      pickupFrontFailed = false;
      logPickup("START_PICKUP");
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
            CLAW_OPEN_ANGLE.get(), SERVO_CLAW_STEP);
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
              CLAW_OPEN_ANGLE.get(), SERVO_CLAW_STEP)) {
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
            CLAW_OPEN_ANGLE.get(), SERVO_CLAW_STEP);
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
        if (o_vision.getVisionObjectWidth() > o_vision.getVisionFrameWidth()
            * SEARCH_MAX_TARGET_WIDTH_FRACTION) {
          visionTargetSeenSince = 0;
          pickupCandidateColor = null;
          o_vision.setText("Pickup Warning", "IGNORING_FRAME_FILLING_BLOB");
          scanPickupCamera(now);
          return true;
        }
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
        o_subsystem.servo_Hook(CLAW_OPEN_ANGLE.get());
        o_vision.setText("Pickup Step", "CENTERING_AND_TRACKING_OBJECT");
        if (!keepVisionTargetOrWait(now, targetVisible)) {
          return true;
        }
        followPickupTarget(now);

        double centerTurn = aimTurnInPlace();
        boolean centered = centerTurn == 0.0
            && Math.abs(getRawPickupOffset()) <= VISION_CENTER_TOLERANCE_PX.get();
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
        o_subsystem.servo_Hook(CLAW_OPEN_ANGLE.get());
        if (!targetVisible && pickupLastSeenAtBottom
            && Math.abs(pickupFilteredX) <= VISION_CENTER_TOLERANCE_PX.get() * 1.5
            && isRawWidthCloseEnough()
            && (getLevelWidth() >= FIRST_VIEW_WIDTH_PX.get()
                || isPickupArmAtLowerLimit())
            && now - visionTargetLastSeen >= PICKUP_EXIT_BOTTOM_WAIT_SEC) {
          stopPickupDrive();
          o_vision.setText("Pickup Warning", "OBJECT_LEFT_FRAME_AT_BOTTOM");
          o_vision.setText("Pickup Exit Reason", "LEFT_FRAME_AT_BOTTOM");
          startSecondPhase();
          return true;
        }
        if (!keepVisionTargetOrWait(now, targetVisible)) {
          o_vision.setText("Pickup Warning", "TARGET_LOST_STOPPED");
          pushRefValid = false;
          return true;
        }
        followPickupTarget(now);
        if (isPushingObject()) {
          stopPickupDrive();
          o_vision.setText("Pickup Warning", "PUSHING_OBJECT_STOPPED");
          o_vision.setText("Pickup Exit Reason", "PUSHING_OBJECT");
          logPickup("EXIT_PUSHING_OBJECT");
          startSecondPhase();
          return true;
        }

        // Height-independent width, see ARM_HORIZONTAL_ANGLE.
        double objectWidth = getLevelWidth();
        o_vision.setValue("Pickup Target Width", objectWidth);

        double objectBottom = pickupFilteredBottom;
        o_vision.setValue("Pickup Target Bottom", objectBottom);
        double offset = pickupFilteredX;
        boolean aligned = Math.abs(offset)
            <= VISION_CENTER_TOLERANCE_PX.get() * 1.5;
        boolean approachAtBottomEdge = isPickupTargetAtBottomEdge();
        // Switch to the second camera view once the object is low and large
        // enough (firstViewWidthPx of the GROUND/PODIUM profile). Require
        // distinct frames so one noisy contour cannot trigger it.
        // Extra exits keep the robot from standing still:
        // - object at the bottom edge and already wide enough, or the arm is
        //   fully down and the camera cannot look any lower;
        // - object very wide (camera at its upper limit, podium).
        boolean widthReached = objectWidth >= FIRST_VIEW_WIDTH_PX.get();
        String exitReason = "None";
        if (!isRawWidthCloseEnough()) {
          exitReason = "None";
        } else if (isPickupZoneReady() && widthReached
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
            logPickup("EXIT_" + exitReason);
            startSecondPhase();
          }
          return true;
        }
        visionCameraCloseSince = 0;
        pickupCameraCloseFrames = 0;

        // Slow down over the second half of the approach so the robot
        // reaches the stop width at the lowest speed, without overshoot.
        double slowProgress = clampVision(
            (objectWidth - FIRST_VIEW_WIDTH_PX.get() * 0.5)
                / (FIRST_VIEW_WIDTH_PX.get() * 0.5),
            0.0, 1.0);
        double forward = VISION_APPROACH_SPEED.get()
            + (VISION_APPROACH_SLOW_SPEED.get() - VISION_APPROACH_SPEED.get())
                * slowProgress;
        if (Math.abs(aimYawError()) > PICKUP_APPROACH_MAX_AIM_ERROR_DEG) {
          // Turn strictly in place so all three motors spin at the same speed.
          o_vision.setText("Pickup Step", "APPROACH_TURNING_TO_OBJECT");
          setPickupDrive(0, aimTurnInPlace(), now);
        } else {
          setPickupDrive(forward, 0, now);
        }
        return true;

      case RAISING_FOR_FINAL_VIEW:
        o_vision.setText("Pickup Step", "RAISING_LIFT_FOR_SECOND_LOOK");
        stopPickupDrive();
        o_subsystem.servo_Hook(CLAW_OPEN_ANGLE.get());
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
        o_subsystem.servo_Hook(CLAW_OPEN_ANGLE.get());
        if (!o_subsystem.moveServoArmToward(
            ARM_SCAN_DOWN_ANGLE.get(), SERVO_ARM_STEP)) {
          pickupServoSettledSince = 0;
          return true;
        }
        if (pickupServoSettledSince == 0) {
          pickupServoSettledSince = now;
        } else if (now - pickupServoSettledSince
            >= ARM_LOWER_SETTLE_SEC) {
          pickupTrackedArmTarget = ARM_SCAN_DOWN_ANGLE.get();
          enterVisionPickupState(VisionPickupState.REACQUIRING_OBJECT);
        }
        return true;

      case REACQUIRING_OBJECT:
        o_vision.setText("Pickup Step", "SECOND_CAMERA_DETECTION");
        o_subsystem.setServoLift(LIFT_FINAL_VIEW_ANGLE);
        o_subsystem.servo_Hook_Hand(ARM_SCAN_DOWN_ANGLE.get());
        o_subsystem.servo_Hook(CLAW_OPEN_ANGLE.get());
        if (!targetVisible || !pickupFilterReady) {
          if (pickupLastSeenAtBottom
              && pickupSecondViewLastTopY >= SECOND_VIEW_MIN_TOP_Y.get()
              && Math.abs(pickupFilteredX) <= VISION_CENTER_TOLERANCE_PX.get() * 1.5
              && now - visionTargetLastSeen >= PICKUP_EXIT_BOTTOM_WAIT_SEC) {
            // Object slipped under the bottom edge of the camera, which means
            // it is directly below the camera inside the claw zone.
            stopPickupDrive();
            o_vision.setText("Pickup Warning", "OBJECT_AT_BOTTOM_UNDER_CLAW");
            enterVisionPickupState(VisionPickupState.LOWERING_LIFT);
            return true;
          }
          pickupFilterReady = false;
          visionCameraCloseSince = 0;
          pickupCameraCloseFrames = 0;
          o_vision.setText("Pickup Warning", "WAITING_FOR_SECOND_VIEW");
          double creptMm = Math.abs(
              o_subsystem.getAverageDriveDistance() - pickupSecondViewStartMm);
          if (pickupLostDirY < 0 && pickupLostDirX == 0
              && creptMm < SECOND_VIEW_MAX_CREEP_MM) {
            setPickupDrive(PICKUP_MIN_FORWARD.get(), 0, now);
            o_vision.setText("Pickup Warning", "SECOND_VIEW_OBJECT_AHEAD_CREEPING");
            return true;
          }
          if ((pickupLostDirX != 0 || pickupLostDirY != 0)
              && now - visionTargetLastSeen <= PICKUP_LOST_SEARCH_SEC) {
            // Arm is already fully down here, so move the robot instead:
            // if lost at a side, rotate in place first with equal motor speeds;
            // if lost at the top, creep forward so the object moves to the bottom edge.
            if (pickupLostDirX != 0) {
              setPickupDrive(0, pickupLostDirX * PICKUP_MIN_TURN.get(), now);
            } else if (pickupLostDirY < 0) {
              setPickupDrive(PICKUP_MIN_FORWARD.get(), 0, now);
            } else {
              stopPickupDrive();
            }
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
        if (Math.abs(finalOffset) > VISION_CENTER_TOLERANCE_PX.get() * 1.5) {
          visionCameraCloseSince = 0;
          pickupCameraCloseFrames = 0;
          o_vision.setText("Pickup Step", "SECOND_VIEW_CENTERING");
          setPickupDrive(0, aimTurnInPlace(), now);
          return true;
        }

        boolean finalCentered = Math.abs(pickupFilteredX)
            <= VISION_CENTER_TOLERANCE_PX.get() * 1.5;
        boolean atBottomEdge = isPickupTargetAtBottomEdge();
        double topY = o_vision.getVisionTargetBottomY()
            - o_vision.getVisionObjectHeight();
        boolean objectAhead = topY < SECOND_VIEW_MIN_TOP_Y.get();
        double approachedMm = Math.abs(
            o_subsystem.getAverageDriveDistance() - pickupSecondViewStartMm);
        o_vision.setValue("Pickup Target Top Y", topY);
        pickupSecondViewLastTopY = topY;
        if (objectAhead && approachedMm >= SECOND_VIEW_MAX_APPROACH_MM.get()) {
          // Drove the whole allowed distance and the top edge is still not
          // where the claw is: do not grab blind, search again.
          stopPickupDrive();
          logPickup(String.format(java.util.Locale.ROOT,
              "SECOND_VIEW_LIMIT_TOP_%.0f", topY));
          o_vision.setText("Pickup Warning", "SECOND_VIEW_TOO_FAR_RESEARCHING");
          enterVisionPickupState(VisionPickupState.RAISING_FOR_SEARCH);
          return true;
        }
        // Because the camera is ABOVE the claw, the object is in the grab zone
        // when it sits at the bottom edge of the camera frame.
        boolean edgeGrabZone = finalCentered && atBottomEdge;
        boolean finalGrabZone = !objectAhead && (edgeGrabZone
            || (finalCentered
                && pickupFilteredBottom >= getSecondViewBottomTarget()
                && pickupFilteredWidth
                    >= FINAL_VIEW_WIDTH_PX * PICKUP_EDGE_ACCEPT_WIDTH_FACTOR));
        o_vision.setValue("Pickup Second Grab Zone", finalGrabZone ? 1.0 : 0.0);
        if (!finalGrabZone) {
          visionCameraCloseSince = 0;
          pickupCameraCloseFrames = 0;

          // Keep the arm fully down and the claw open. Drive forward until the
          // object reaches the bottom edge of the camera (under the claw).
          if (Math.abs(aimYawError()) > PICKUP_APPROACH_MAX_AIM_ERROR_DEG) {
            o_vision.setText("Pickup Step", "SECOND_VIEW_CENTERING");
            setPickupDrive(0, aimTurnInPlace(), now);
            return true;
          }
          double finalSlowdown = clampVision(
              (getPickupBottomEdgeY() - pickupFilteredBottom) / 55.0,
              0.70, 1.0);
          double finalForward = FINAL_APPROACH_SPEED.get() * finalSlowdown;
          o_vision.setText("Pickup Step", "SECOND_VIEW_APPROACHING_OBJECT");
          setPickupDrive(finalForward, 0, now);
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
          logPickup(String.format(java.util.Locale.ROOT,
              "SECOND_VIEW_OK_TOP_%.0f", topY));
          enterVisionPickupState(VisionPickupState.LOWERING_LIFT);
        }
        return true;

      case MEASURING_HEIGHT:
        runHeightMeasurement(now, targetVisible);
        return true;

      case FRONT_ALIGNING:
        runFrontAligning(now, targetVisible);
        return true;

      case FRONT_APPROACHING:
        runFrontApproaching(now, targetVisible);
        return true;

      case LOWERING_LIFT:
        o_vision.setText("Pickup Step", "LOWERING_LIFT_BY_CAMERA");
        o_subsystem.servo_Hook(CLAW_OPEN_ANGLE.get());
        o_subsystem.servo_Hook_Hand(ARM_SCAN_DOWN_ANGLE.get());
        runCameraDescent(now, targetVisible);
        return true;

      case GRABBING:
        o_vision.setText("Pickup Step", "CLOSING_CLAW");
        stopPickupDrive();
        o_subsystem.servo_Hook_Hand(pickupGrabArmAngle);
        // Reach the grab height first (front grab adds
        // FRONT_LIFT_EXTRA_DOWN_DEG), only then close the claw.
        if (!o_subsystem.moveServoLiftToward(
            pickupGrabLiftAngle, DESCENT_MAX_STEP.get())) {
          o_vision.setText("Pickup Step", "LOWERING_LIFT_BEFORE_CLAW");
          pickupServoSettledSince = 0;
          return true;
        }
        double clawRemaining = Math.abs(
            CLAW_CLOSED_ANGLE.get() - o_subsystem.getServoClawAngle());
        double clawStep = clawRemaining > CLAW_SLOW_ZONE_DEG
            ? SERVO_CLAW_STEP : SERVO_CLAW_SLOW_STEP;
        if (!o_subsystem.moveServoClawToward(CLAW_CLOSED_ANGLE.get(), clawStep)) {
          pickupServoSettledSince = 0;
          return true;
        }
        if (pickupServoSettledSince == 0) {
          pickupServoSettledSince = now;
          o_vision.setText("Pickup Step", "HOLDING_OBJECT");
        } else if (now - pickupServoSettledSince >= VISION_GRIP_DELAY_SEC.get()) {
          enterVisionPickupState(VisionPickupState.RAISING_LIFT);
        }
        return true;

      case RAISING_LIFT:
        o_vision.setText("Pickup Step", "RAISING_LIFT_WITH_OBJECT");
        stopPickupDrive();
        o_subsystem.servo_Hook(CLAW_CLOSED_ANGLE.get());
        o_subsystem.servo_Hook_Hand(pickupGrabArmAngle);
        if (!runProfiledServo(true, LIFT_SEARCH_ANGLE,
            PICKUP_LIFT_RAISE_DEG_PER_SEC, now)) {
          pickupServoSettledSince = 0;
          return true;
        }
        if (pickupServoSettledSince == 0) {
          pickupServoSettledSince = now;
        } else if (now - pickupServoSettledSince >= LIFT_RAISE_SETTLE_SEC) {
          enterVisionPickupState(
              pickupGrabIsFront && !pickupFrontFailed
                  ? VisionPickupState.VERIFYING_GRAB
                  : VisionPickupState.RAISING_ARM);
        }
        return true;

      case VERIFYING_GRAB:
        runGrabVerification(now, targetVisible);
        return true;

      case RAISING_ARM:
        o_vision.setText("Pickup Step", "RAISING_CAMERA_AFTER_PICKUP");
        stopPickupDrive();
        o_subsystem.setServoLift(LIFT_SEARCH_ANGLE);
        o_subsystem.servo_Hook(CLAW_CLOSED_ANGLE.get());
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
        o_subsystem.servo_Hook(CLAW_CLOSED_ANGLE.get());
        o_subsystem.servo_Hook_Hand(ARM_SEARCH_ANGLE);
        return false;

      default:
        return false;
    }
  }

  private boolean isPickupZoneReady() {
    boolean ready = Math.abs(pickupFilteredX)
            <= VISION_CENTER_TOLERANCE_PX.get() * 1.5
        && (isPickupTargetAtBottomEdge()
            || pickupFilteredBottom >= PICKUP_ZONE_MIN_BOTTOM_Y_PX
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
   * Drives with forward power or pure in-place rotation. Whenever a turn is
   * commanded (turn != 0), forward motion is stopped so all three motors
   * spin at the exact same speed to rotate on the spot.
   */
  private void setPickupDrive(double forward, double turn, double now) {
    double dt = pickupDriveLastAt == 0
        ? 0.02 : clampVision(now - pickupDriveLastAt, 0.0, 0.05);
    pickupDriveLastAt = now;

    if (Math.abs(turn) > 1e-4) {
      pickupDriveForward = 0.0;
      double outTurn = Math.copySign(
          Math.max(Math.abs(turn), PICKUP_MIN_TURN.get() + pickupStallBoost), turn);
      pickupDriveTurn = outTurn;
      updateStallBoost(true, now);
      o_vision.setValue("Pickup Drive Forward", 0.0);
      o_vision.setValue("Pickup Drive Turn", outTurn);
      o_subsystem.rotateInPlace(outTurn);
      return;
    }

    if (forward == 0.0) {
      pickupDriveForward = 0.0;
    } else {
      pickupDriveForward += clampVision(forward - pickupDriveForward,
          -PICKUP_DRIVE_DECEL_PER_SEC * dt, PICKUP_DRIVE_ACCEL_PER_SEC * dt);
    }

    double outForward = 0.0;
    if (Math.abs(pickupDriveForward) > 0.01) {
      outForward = Math.copySign(Math.max(Math.abs(pickupDriveForward),
          PICKUP_MIN_FORWARD.get() + pickupStallBoost), pickupDriveForward);
    }
    pickupDriveTurn = 0.0;
    updateStallBoost(outForward != 0.0, now);
    o_vision.setValue("Pickup Drive Forward", outForward);
    o_vision.setValue("Pickup Drive Turn", 0.0);
    o_subsystem.driveOmni(outForward, 0, 0.0);
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
    double down = (o_subsystem.getServoArmAngle() - ARM_HORIZONTAL_ANGLE.get())
        + (pickupFilteredY - o_vision.getVisionFrameHeight() / 2.0)
            * CAMERA_DEG_PER_PX_V;
    // Negative = looking up at an object above the camera.
    return clampVision(down, -45.0, 85.0);
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
    return o_subsystem.getServoArmAngle() >= ARM_SCAN_DOWN_ANGLE.get() - 1.0;
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
        -PICKUP_MAX_TURN.get(), PICKUP_MAX_TURN.get());
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
    if (next >= ARM_SCAN_DOWN_ANGLE.get()) {
      pickupScanArmTarget = ARM_SCAN_DOWN_ANGLE.get();
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
      double imageError = pickupFilteredBottom
          - PICKUP_IMAGE_CENTER_Y_PX;
      if (Math.abs(imageError) > PICKUP_IMAGE_Y_DEADBAND_PX) {
        double correction = clampVision(
            imageError * PICKUP_ARM_KP,
            -PICKUP_ARM_MAX_CORRECTION_DEG,
            PICKUP_ARM_MAX_CORRECTION_DEG);
        pickupTrackedArmTarget = clampVision(
            pickupTrackedArmTarget + correction,
            ARM_SEARCH_ANGLE,
            ARM_SCAN_DOWN_ANGLE.get());
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
   * Lowers the lift while the down-looking camera watches the object at the
   * bottom edge of the frame (since the camera is mounted above the claw).
   */
  /**
   * Lowers the lift to the angle that matches the object height measured by
   * the camera. While the object is still visible the robot keeps it under
   * the claw (turn for X, creep forward for Y); when it slides out of view
   * on the rotating lift, the descent simply continues to the same angle.
   */
  private void runCameraDescent(double now, boolean targetVisible) {
    double lift = o_subsystem.getServoLiftAngle();
    double goal = Math.min(LIFT_LOWEST_ANGLE.get(), getLiftGrabAngle());
    o_vision.setValue("Pickup Descent Lift", lift);
    o_vision.setValue("Pickup Descent Goal", goal);

    if (lift >= LIFT_LIMIT_SWITCH_MIN_ANGLE
        && o_subsystem.isLiftLowerLimitPressed()) {
      stopPickupDrive();
      finishCameraDescent("LIFT_LOWER_LIMIT");
      return;
    }

    // No "object fills the camera" stop here: with the arm down a floor
    // object already fills the frame while the lift is still at the top
    // (318 px wide in pickup_log.csv), so that check closed the claw in the
    // air. The measured height alone decides how far to go.
    // Wheels only turn, and only while the claw is still high. No driving
    // forward here: the lift swings the camera along an arc, the object
    // drifts up in the image, and creeping toward it rammed the object.
    // The second view already put the robot in place with the lift up.
    boolean stillHigh = lift < goal - DESCENT_SLOW_ZONE_DEG;
    if (stillHigh && targetVisible && pickupFilterReady) {
      double turn = Math.abs(getRawPickupOffset())
          > VISION_CENTER_TOLERANCE_PX.get() * 1.5 ? aimTurnInPlace() : 0.0;
      if (turn != 0.0) {
        setPickupDrive(0, turn, now);
        o_vision.setText("Pickup Warning", "DESCENT_TURNING_TO_CENTER");
        return;
      }
    }

    stopPickupDrive();
    o_vision.setText("Pickup Warning",
        targetVisible ? "None" : "DESCENT_OBJECT_OUT_OF_VIEW_CONTINUING");
    if (lift >= goal - 0.5) {
      finishCameraDescent("MEASURED_HEIGHT_REACHED");
      return;
    }
    double remaining = goal - lift;
    double step = remaining > DESCENT_SLOW_ZONE_DEG
        ? DESCENT_MAX_STEP.get()
        : DESCENT_MIN_STEP + (DESCENT_MAX_STEP.get() - DESCENT_MIN_STEP)
            * remaining / DESCENT_SLOW_ZONE_DEG;
    o_subsystem.setServoLift(Math.min(goal, lift + step));
  }

  /** Height index of the newest frame, see HEIGHT_INDEX_PODIUM. */
  private double computeHeightIndex() {
    double top = o_vision.getVisionTargetBottomY()
        - o_vision.getVisionObjectHeight();
    double down = (o_subsystem.getServoArmAngle() - ARM_HORIZONTAL_ANGLE.get())
        + (top - o_vision.getVisionFrameHeight() / 2.0) * CAMERA_DEG_PER_PX_V;
    down = clampVision(down, -45.0, 85.0);
    double width = Math.max(1.0, o_vision.getVisionObjectWidth());
    return 1000.0 * Math.sin(Math.toRadians(down)) / width;
  }

  /** Robot and arm stand still; average the height index over frames. */
  private void runHeightMeasurement(double now, boolean targetVisible) {
    o_vision.setText("Pickup Step", "MEASURING_OBJECT_HEIGHT");
    stopPickupDrive();
    double elapsed = now - visionPickupStateStart;
    if (elapsed < MEASURE_SETTLE_SEC) {
      return;
    }
    if (targetVisible && pickupFrameUpdated) {
      pickupHeightIndexSum += computeHeightIndex();
      pickupHeightIndexCount++;
    }
    boolean enough = pickupHeightIndexCount >= MEASURE_FRAMES;
    if (!enough && elapsed < MEASURE_TIMEOUT_SEC) {
      return;
    }
    double index = pickupHeightIndexCount > 0
        ? pickupHeightIndexSum / pickupHeightIndexCount
        : Double.NaN;
    if (Double.isNaN(index) || index < 0 || index > HEIGHT_INDEX_MAX_VALID) {
      // No frames or a nonsense value: do not guess the lift height.
      logPickup(String.format(java.util.Locale.ROOT,
          "HEIGHT_INVALID_%.2f_FRAMES_%d", index, pickupHeightIndexCount));
      o_vision.setText("Pickup Warning", "HEIGHT_MEASURE_FAILED_RESEARCHING");
      enterVisionPickupState(VisionPickupState.RAISING_FOR_SEARCH);
      return;
    }
    decideGrabFromHeight(index);
  }

  private void decideGrabFromHeight(double index) {
    pickupHeightFactor = clampVision(
        (HEIGHT_INDEX_FLOOR.get() - index)
            / (HEIGHT_INDEX_FLOOR.get() - HEIGHT_INDEX_PODIUM.get()), 0.0, 1.0);
    pickupRawLiftGrabAngle = getLiftGrabAngle();
    o_vision.setValue("Pickup Height Index", index);
    o_vision.setValue("Pickup Height Factor", pickupHeightFactor);
    o_vision.setValue("Pickup Lift Grab Angle", pickupRawLiftGrabAngle);
    logPickup(String.format(java.util.Locale.ROOT,
        "HEIGHT_INDEX_%.2f_FRAMES_%d", index, pickupHeightIndexCount));
    if (!pickupFrontFailed) {
      pickupFrontArmAngle = ARM_FRONT_GRAB_ANGLE.get();
      o_vision.setText("Pickup Grab Mode", "FRONT_FIRST_TRY");
      enterVisionPickupState(VisionPickupState.FRONT_ALIGNING);
    } else if (index < HEIGHT_INDEX_TOO_HIGH.get()) {
      o_vision.setText("Pickup Grab Mode", "FRONT_ARM_STRAIGHT");
      enterVisionPickupState(VisionPickupState.FRONT_ALIGNING);
    } else {
      o_vision.setText("Pickup Grab Mode",
          pickupHeightFactor >= 0.5 ? "TOP_PODIUM" : "TOP_FLOOR");
      enterVisionPickupState(VisionPickupState.RAISING_FOR_FINAL_VIEW);
    }
  }

  /**
   * Stops after the first phase and measures the object height; the
   * grab type (from above / straight ahead) is chosen from that height.
   */
  private void startSecondPhase() {
    logPickup("FIRST_PHASE_DONE");
    enterVisionPickupState(VisionPickupState.MEASURING_HEIGHT);
  }


  /**
   * Front grab: arm straight, lower the lift until the camera sees the
   * object, then close the claw right there.
   */
  private void runFrontAligning(double now, boolean targetVisible) {
    o_vision.setText("Pickup Step", "FRONT_LOWERING_UNTIL_SEEN");
    stopPickupDrive();
    o_subsystem.servo_Hook(CLAW_OPEN_ANGLE.get());
    pickupFrontArmAngle = ARM_FRONT_GRAB_ANGLE.get();
    if (!o_subsystem.moveServoArmToward(pickupFrontArmAngle, SERVO_ARM_STEP)) {
      frontArmStraightSince = 0;
      frontSeenFrames = 0;
      return;
    }
    if (frontArmStraightSince == 0) {
      frontArmStraightSince = now;
    }
    if (now - frontArmStraightSince < FRONT_ARM_SETTLE_SEC) {
      return;
    }

    if (targetVisible) {
      if (pickupFrameUpdated) {
        frontSeenFrames++;
      }
      // Lift holds still while the sighting is confirmed.
      if (frontSeenFrames >= Math.max(1, Math.round(FRONT_SEEN_CONFIRM_FRAMES.get()))) {
        logPickup(String.format(java.util.Locale.ROOT,
            "FRONT_SEEN_AT_LIFT_%.1f", o_subsystem.getServoLiftAngle()));
        // Seen: stop the lift here, center and drive up to the object.
        enterVisionPickupState(VisionPickupState.FRONT_APPROACHING);
      }
      return;
    }
    frontSeenFrames = 0;

    double lift = o_subsystem.getServoLiftAngle();
    double lowest = LIFT_LOWEST_ANGLE.get();
    if (lift >= lowest - 0.01) {
      // Whole way down and never seen: try the grab from above.
      o_vision.setText("Pickup Warning", "FRONT_NOT_SEEN_SWITCH_TO_ABOVE");
      pickupFrontFailed = true;
      logPickup("FRONT_NOT_SEEN_SWITCH_TO_ABOVE");
      enterVisionPickupState(VisionPickupState.RAISING_FOR_SEARCH);
      return;
    }
    o_subsystem.setServoLift(Math.min(lowest, lift + FRONT_SEARCH_LIFT_STEP.get()));
    o_vision.setText("Pickup Warning", "FRONT_LIFT_DOWN_LOOKING");
  }

  /**
   * Stage 0: measure the object width with the claw closed.
   * Stage 1: back off VERIFY_BACKOFF_MM.
   * Stage 2: measure again and decide whether the object is in the claw.
   */
  private void runGrabVerification(double now, boolean targetVisible) {
    o_vision.setText("Pickup Step", "CHECKING_OBJECT_IN_CLAW");
    o_subsystem.servo_Hook(CLAW_CLOSED_ANGLE.get());
    o_subsystem.servo_Hook_Hand(pickupGrabArmAngle);
    o_subsystem.setServoLift(LIFT_SEARCH_ANGLE);
    double elapsed = now - verifyStageStart;

    if (verifyStage == 1) {
      double backed = Math.abs(
          o_subsystem.getAverageDriveDistance() - verifyStartMm);
      if (backed < VERIFY_BACKOFF_MM.get() && elapsed < VERIFY_BACKOFF_TIMEOUT_SEC) {
        setPickupDrive(-PICKUP_MIN_FORWARD.get(), 0, now);
        return;
      }
      stopPickupDrive();
      verifyStage = 2;
      verifyStageStart = now;
      verifyWidthSum = 0;
      verifyWidthCount = 0;
      return;
    }

    stopPickupDrive();
    if (elapsed < VERIFY_SETTLE_SEC) {
      return;
    }
    if (targetVisible && pickupFrameUpdated) {
      verifyWidthSum += o_vision.getVisionObjectWidth();
      verifyWidthCount++;
    }
    if (verifyWidthCount < VERIFY_FRAMES
        && elapsed < VERIFY_SETTLE_SEC + VERIFY_SAMPLE_TIMEOUT_SEC) {
      return;
    }
    boolean visible = verifyWidthCount > 0;
    double width = visible ? verifyWidthSum / verifyWidthCount : 0.0;

    if (verifyStage == 0) {
      verifyVisibleBefore = visible;
      verifyWidthBefore = width;
      verifyStage = 1;
      verifyStageStart = now;
      verifyStartMm = o_subsystem.getAverageDriveDistance();
      return;
    }

    boolean held;
    if (verifyVisibleBefore && visible) {
      held = width >= verifyWidthBefore * VERIFY_HELD_WIDTH_RATIO.get();
    } else {
      // Seen only after backing off: it was left behind. Not seen at all
      // (or lost after backing off): cannot prove a miss, accept it.
      held = !(visible && !verifyVisibleBefore);
    }
    logPickup(String.format(java.util.Locale.ROOT,
        "VERIFY_%s_BEFORE_%.0f_AFTER_%.0f", held ? "HELD" : "MISSED",
        verifyVisibleBefore ? verifyWidthBefore : -1, visible ? width : -1));
    if (held) {
      enterVisionPickupState(VisionPickupState.RAISING_ARM);
      return;
    }
    // Missed: open the claw and start over, this time grabbing from above.
    pickupFrontFailed = true;
    o_vision.setText("Pickup Warning", "FRONT_GRAB_MISSED_RETRY_FROM_ABOVE");
    enterVisionPickupState(VisionPickupState.RAISING_FOR_SEARCH);
  }

  /**
   * Front grab after the object was seen: lift holds still, the robot turns
   * to center it and drives until it is FRONT_GRAB_WIDTH_PX wide, then the
   * claw closes. Lost while far: back to lowering the lift to find it.
   */
  private void runFrontApproaching(double now, boolean targetVisible) {
    o_vision.setText("Pickup Step", "FRONT_CENTERING_AND_APPROACHING");
    o_subsystem.servo_Hook(CLAW_OPEN_ANGLE.get());
    o_subsystem.servo_Hook_Hand(ARM_FRONT_GRAB_ANGLE.get());

    if (!targetVisible || !pickupFilterReady) {
      stopPickupDrive();
      visionCameraCloseSince = 0;
      pickupCameraCloseFrames = 0;
      if (frontLastWidth >= FRONT_GRAB_WIDTH_PX.get() * FRONT_LOST_ACCEPT_FACTOR) {
        finishFrontApproach("FRONT_CLOSE_THEN_LOST");
        return;
      }
      o_vision.setText("Pickup Warning", "FRONT_APPROACH_LOST");
      if (now - visionTargetLastSeen >= VISION_TARGET_LOST_TIMEOUT_SEC) {
        enterVisionPickupState(VisionPickupState.FRONT_ALIGNING);
      }
      return;
    }

    double width = pickupFilteredWidth;
    frontLastWidth = width;
    o_vision.setValue("Pickup Front Width", width);

    if (width >= FRONT_GRAB_WIDTH_PX.get()) {
      stopPickupDrive();
      if (visionCameraCloseSince == 0) {
        visionCameraCloseSince = now;
      }
      if (pickupFrameUpdated) {
        pickupCameraCloseFrames++;
      }
      if (pickupCameraCloseFrames >= PICKUP_CAMERA_GRAB_MIN_FRAMES
          && now - visionCameraCloseSince >= VISION_CAMERA_CLOSE_CONFIRM_SEC) {
        finishFrontApproach("FRONT_SIZE_REACHED");
      }
      return;
    }
    visionCameraCloseSince = 0;
    pickupCameraCloseFrames = 0;

    if (isPushingObject()) {
      // Object moves with the robot: it is already at the claw.
      finishFrontApproach("FRONT_PUSHING_OBJECT");
      return;
    }

    double turn = aimTurnInPlace();
    if (turn != 0.0
        || Math.abs(getRawPickupOffset()) > VISION_CENTER_TOLERANCE_PX.get() * 1.5) {
      o_vision.setText("Pickup Warning", "FRONT_CENTERING");
      setPickupDrive(0, turn, now);
      return;
    }
    o_vision.setText("Pickup Warning", "FRONT_DRIVING_TO_OBJECT");
    setPickupDrive(PICKUP_MIN_FORWARD.get(), 0, now);
  }

  private void finishFrontApproach(String reason) {
    stopPickupDrive();
    pickupGrabLiftAngle = Math.min(LIFT_LOWEST_ANGLE.get(),
        o_subsystem.getServoLiftAngle() + FRONT_LIFT_EXTRA_DOWN_DEG.get());
    pickupGrabArmAngle = pickupFrontArmAngle;
    pickupGrabIsFront = true;
    o_vision.setText("Pickup Descent Stop", reason);
    o_vision.setValue("Pickup Grab Lift Angle", pickupGrabLiftAngle);
    logPickup("DESCENT_STOP_" + reason);
    enterVisionPickupState(VisionPickupState.GRABBING);
  }

  private double getLiftGrabAngle() {
    return LIFT_GRAB_ANGLE_FLOOR.get()
        + (LIFT_GRAB_ANGLE_PODIUM.get() - LIFT_GRAB_ANGLE_FLOOR.get()) * pickupHeightFactor;
  }

  private double getSecondViewBottomTarget() {
    double target = SECOND_VIEW_BOTTOM_Y_FLOOR
        + (SECOND_VIEW_BOTTOM_Y_PODIUM - SECOND_VIEW_BOTTOM_Y_FLOOR)
            * pickupHeightFactor;
    return Math.min(target, getPickupBottomEdgeY());
  }

  private void finishCameraDescent(String reason) {
    pickupGrabLiftAngle = o_subsystem.getServoLiftAngle();
    pickupGrabArmAngle = ARM_SCAN_DOWN_ANGLE.get();
    pickupGrabIsFront = false;
    o_vision.setText("Pickup Descent Stop", reason);
    o_vision.setValue("Pickup Grab Lift Angle", pickupGrabLiftAngle);
    logPickup("DESCENT_STOP_" + reason);
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
      double limit = pickupLostDirY > 0 ? ARM_SCAN_DOWN_ANGLE.get() : ARM_SEARCH_ANGLE;
      double arm = o_subsystem.getServoArmAngle();
      if (Math.abs(limit - arm) > 0.5) {
        pickupTrackedArmTarget = clampVision(
            arm + pickupLostDirY * PICKUP_LOST_ARM_STEP,
            ARM_SEARCH_ANGLE, ARM_SCAN_DOWN_ANGLE.get());
        o_subsystem.servo_Hook_Hand(pickupTrackedArmTarget);
        acting = true;
      }
      where += pickupLostDirY > 0 ? "DOWN" : "UP";
    }

    if (pickupLostDirX != 0) {
      setPickupDrive(0, pickupLostDirX * PICKUP_MIN_TURN.get(), now);
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
    double target = normalizeYaw(targetDeg);
    double speed = 0.3;

    double error = target - currentAngle;
    if (error > 180)  error -= 360;
    if (error < -180) error += 360;

    if (Math.abs(error) > 0.15) {
      if (Math.abs(error) < 10) speed = 0.15;
      // Pure in-place rotation: all three motors spin at the exact same speed.
      o_subsystem.rotateInPlace(Math.copySign(speed, error));
    } else {
      o_subsystem.TimerStop();
      angleRobot = target;
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

    double front = o_subsystem.getForwardSharp();
    if (front < dist) {
      o_subsystem.TimerStop();
      stateAutomatic++;
      return;
    }
    // Last 10 cm at minSpeed: the old slowdown factor dropped the power to
    // ~0.1 just before the target and the robot stopped short.
    double base = front - dist <= 10.0 ? minSpeed : 0.3;
    double lSpeed = base;
    double rSpeed = base;
    double error = headingError();
    if (error > 0)       rSpeed -= Math.min(0.15, error / 90.0);
    else if (error < 0)  lSpeed -= Math.min(0.15, -error / 90.0);
    o_subsystem.setDrivePower(lSpeed, rSpeed);
  }

  /**
   * Strafes with the gyro holding angleRobot. strafe > 0 = right,
   * strafe < 0 = left.
   */
  private void strafeHoldingHeading(double strafe) {
    double error = headingError();
    double turn = Math.abs(error) <= 0.5 ? 0.0
        : clampVision(error * SIDE_HEADING_KP,
            -SIDE_HEADING_MAX_TURN, SIDE_HEADING_MAX_TURN);
    o_subsystem.driveOmni(0, strafe, turn);
  }

  private double sideStrafeSpeed(double remainingCm) {
    return remainingCm <= SIDE_SLOW_ZONE_CM
        ? SIDE_STRAFE_SLOW_SPEED.get() : SIDE_STRAFE_SPEED.get();
  }

  /** Strictly left until the LEFT ultrasonic reads dist cm or less. */
  public void goLeftSonic(double dist) {
    double sonic = o_subsystem.getDistanceSonicLeft();
    if (Double.isFinite(sonic) && sonic <= dist) {
      o_subsystem.TimerStop();
      stateAutomatic++;
      return;
    }
    // No echo yet = the wall is still far away: keep going.
    double remaining = Double.isFinite(sonic) ? sonic - dist : Double.MAX_VALUE;
    strafeHoldingHeading(-sideStrafeSpeed(remaining));
  }

  /** Strictly right until the RIGHT ultrasonic reads dist cm or less. */
  public void goRightSonic(double dist) {
    double sonic = o_subsystem.getDistanceSonicRight();
    if (Double.isFinite(sonic) && sonic <= dist) {
      o_subsystem.TimerStop();
      stateAutomatic++;
      return;
    }
    double remaining = Double.isFinite(sonic) ? sonic - dist : Double.MAX_VALUE;
    strafeHoldingHeading(sideStrafeSpeed(remaining));
  }

  /** Strictly left, away from the right wall, until RIGHT reads >= dist. */
  public void goLeftWithRightSonic(double dist) {
    double sonic = o_subsystem.getDistanceSonicRight();
    if (!Double.isFinite(sonic) || sonic >= dist) {
      o_subsystem.TimerStop();
      stateAutomatic++;
      return;
    }
    strafeHoldingHeading(-sideStrafeSpeed(dist - sonic));
  }

  /** Strictly right, away from the left wall, until LEFT reads >= dist. */
  public void goRigtWithLeftSonic(double dist) {
    double sonic = o_subsystem.getDistanceSonicLeft();
    if (!Double.isFinite(sonic) || sonic >= dist) {
      o_subsystem.TimerStop();
      stateAutomatic++;
      return;
    }
    strafeHoldingHeading(sideStrafeSpeed(dist - sonic));
  }

  /**
   * Forward until the front Sharps read dist cm. There is no front
   * ultrasonic any more (they are on the sides), so this uses the Sharps.
   */
  public void goForwardSonic(double dist) {
    goForwardSharp(dist);
  }

  /**
   * There is no sensor at the back any more: this step is skipped so the
   * route does not hang. Replace it with goBackDistance(...) if needed.
   */
  public void goBackSonic(double dist) {
    o_subsystem.stopDrive();
    o_vision.setText("Auto Warning", "goBackSonic skipped: no rear sensor");
    stateAutomatic++;
  }

  public void alignToVision(double offsetX) {
    double speed = 0.2;
    double tolerance = 10;
    if (Math.abs(offsetX) > tolerance) {
      o_subsystem.rotateInPlace(Math.copySign(speed, offsetX));
    } else {
      o_subsystem.TimerStop();
      stateAutomatic++;
      return;
    }
  }

  // ─── Захват ───────────────────────────────────────────────────────────────

  public void open_Hand() {
    o_subsystem.servo_Hook(CLAW_OPEN_ANGLE.get());
  }

  public void closed_Hand_Kub() {
    o_subsystem.servo_Hook(CLAW_CLOSED_ANGLE.get());
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
    double sonicLeft    = o_subsystem.getDistanceSonicLeft();
    double sonicRight   = o_subsystem.getDistanceSonicRight();
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
        if (sonicLeft < 28 && sonicRight < 28) turnDegrees = 180.0;
        targetAngle = normalizeYaw(currentYaw + turnDegrees);
        angleRobot = targetAngle;
        driveState = "turning";
      }
    }

    if (driveState.equals("turning")) {
      double error = ((targetAngle - currentYaw + 180) % 360) - 180;
      if (error < -180) error += 360;
      if (Math.abs(error) > 1.8) {
        double rotateSpeed = error * 0.0042;
        if (Math.abs(rotateSpeed) < 0.15) rotateSpeed = Math.copySign(0.15, error);
        rotateSpeed = Math.max(-0.45, Math.min(0.45, rotateSpeed));
        o_subsystem.rotateInPlace(rotateSpeed);
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
