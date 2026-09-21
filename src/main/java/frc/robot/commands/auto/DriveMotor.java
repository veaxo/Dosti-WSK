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
  
  public final double maxSpeed =                     0.4;
  public final double minSpeed =                     0.16;

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
  boolean found = false;
  String basket_1 = null;
  String basket_2 = null;
  String seve_scan;

  private double sortTimer = 0;
  private int    sortState = 0;

  private static final int SERVO_CENTER = 152;
  private static final int SERVO_LEFT   = 107;
  private static final int SERVO_RIGHT  = 197;

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
    // closed_Hand_Kub();
    // o_subsystem.setServoLift(150);
    o_subsystem.servo_Hook_Hand(90);
    o_vision.resetCameraScan();
    o_subsystem.resetYaw();
    o_subsystem.resetEncoder();
    o_subsystem.setButtonLed("Running", false);
    o_subsystem.setButtonLed("Stopped", true);
    o_subsystem.setButtonLed("Start",   true);
    o_subsystem.setButtonLed("Stop",    true);
    start = true;
    stateAutomatic = 0;
    angleRobot = normalizeYaw(o_subsystem.getYaw()); 
  }

  @Override
  public void execute() {
    if (o_subsystem.getButtonState("Start")) {
      start = true;
      // open_Hand();
      flzekSharp = true;         
      flzekSonic = false;
      flzekSonicLeft = true;      
      flzekSonicRight = true;
      // o_subsystem.setServoLift(0);
      o_subsystem.setButtonLed("Running", true);
      o_subsystem.setButtonLed("Stopped", false); 
    }

    if (o_subsystem.getButtonState("Stop")) {
      start = false;
      stateAutomatic = -1;
      // o_subsystem.setServoLift(500);
      o_subsystem.setButtonLed("Running", false);
      o_subsystem.setButtonLed("Stopped", true);
      o_subsystem.resetEncoder();
      o_subsystem.resetYaw();
      o_subsystem.TimerStop();
    }

    if (start) {
      if (o_subsystem.getButtonState("Stop")) {
        start = false;
        stateAutomatic = -1;
        o_subsystem.setButtonLed("Running", false);
        o_subsystem.setButtonLed("Stopped", true);
        o_subsystem.resetEncoder();
        o_subsystem.resetYaw();
        o_subsystem.TimerStop();
      }

      switch (stateAutomatic) {
        case 0:
          // goForwardDistance(50);
          // o_vision.setMode(VisionSubsystem.MODE_MARKER);
          // o_vision.enableScanning(true);
          break;
        case 1:
          // rotateTheRobot(90);
          break;
        case 2:
          goForwardDistance(138);
          break;
        case 3:
          rotateTheRobot(180);
          break;
        case 4:
          goForwardDistance(40);
          break;
        case 5:
          rotateTheRobot(360);
          break;
        case 6:
          goForwardDistance(139);
          break;
        case 7:
          rotateTheRobot(90);
          break;
        case 8:
          goForwardDistance(170);
          break;
        case 9:
          rotateTheRobot(180);;
          break;
        case 10:
          goForwardDistance(150);
          break;
        case 11:
          rotateTheRobot(270);
          break;
        case 12:
          goForwardDistance(200);
          break;
        case 13:
          rotateTheRobot(360);
          break;
        case 14:
          goForwardDistance(60);
          break;
        case 15:
          rotateTheRobot(270);
          break;
        case 16:
          goForwardDistance(103);
          break;
        case 17:
         rotateTheRobot(180);
          break;
        case 18:
          goForwardDistance(90);
          break;
        case 19:
          // goForwardSharp(60);
          break;
        case 20:
          // A zero target can never be reached by a valid distance sensor and
          // causes division by zero in the slowdown calculation.
          stateAutomatic++;
          break;
        case 21:
          goLeftSonic(24);
          break;
        case 22:
          goRightSonic(70);
          break;
        case 23:
          goForwardSharp(47);
          break;
        case 24:
          open_Hand();
          Timer.delay(1);
          stateAutomatic++;
          break;
        case 25:
        goBackSharp(10);
        break;
        case 26:
        rotateTheRobot(90);
        break;
        case 27:
        goForwardDistance(110);
        break;
        case 28:
        goRightSonic(7);
        break;
        case 29:
        goLeftSonic(17);
        break;
        case 30:
        goForwardDistance(40);
        break;
        case 31:
        closed_Hand_Kub();
        Timer.delay(2);
        stateAutomatic++;
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
        goRightSonic(34);
        break;
        case 36:
        goLeftWithRightSonic(34);
        break;
        case 37:
        goForwardSharp(50);
        break;
        case 38:
        open_Hand();
        Timer.delay(1);
        stateAutomatic++;
        break;

        // case 21:
        //   open_Hand();
        //   Timer.delay(1);
        //   stateAutomatic++;
        //   break;
        // case 10:
        //   goRightSonic(70);
        //   break;
        // case 11:
        //     goForwardSharp(20);
        //   break;
        // case 12:
        //   rotateTheRobot(270);
        //   break;
        // case 13:
        //   goRightSonic(dist);
        //   break;
        // case 14:
        //   goLeftWithRightSonic(dist);
        //   break;
        // case 15:
        //   goBackSharp(dist);
        //   break;
        // case 16:
        //   goForwardDistance(dist);
        //   break;
        // case 17:
        //   closed_Hand_Kub();
        //   Timer.delay(1);
        //   stateAutomatic++;
        //   break;
        // case 18:
        //   rotateTheRobot(0);
        //   break;
        // case 19:
        //   goBackSharp(10);
        //   break;
        // case 20:
        //   goLeftSonic(dist);
        //   break;
        // case 21:
        //   goRigtWithLeftSonic();
        //   break;
        // case 22:
        //   goForwardDistance(dist);
        //   break;
        // case 23:
        //   goLeftSonic(dist);
        //   break;
        // case 24:
        //   goRigtWithLeftSonic();
        //   break;
        // case 25:
        //   goForwardDistance(dist);
        //   break;
        // case 26:
        //   closed_Hand_Kub();
        //   Timer.delay(1);
        //   stateAutomatic++;
        //   break;
        default:
          // o_subsystem.setMotorLeft(0);
          // o_subsystem.setMotorRight(0);
          // o_subsystem.setMotorBack(0);
          o_subsystem.TimerStop();  
          o_subsystem.resetEncoder();
          o_subsystem.resetYaw();
          o_subsystem.setButtonLed("Running", false);
          o_subsystem.setButtonLed("Stopped", true);
          break;
      }
    } 
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
    double currentAngle = normalizeYaw(o_subsystem.getYaw());
    double speed = 0.3;

    double error = targetDeg - currentAngle;
    if (error > 180)  error -= 360;
    if (error < -180) error += 360;

    if (Math.abs(error) > 0.15) {
      if (Math.abs(error) < 10) speed = 0.15;
      if (error > 0) {
        o_subsystem.setMotorLeft(speed / 2);
        o_subsystem.setMotorRight(speed / 2);
      } else {
        o_subsystem.setMotorLeft(-speed);
        o_subsystem.setMotorRight(-speed);
      }
    } else {
      o_subsystem.setMotorLeft(0);
      o_subsystem.setMotorRight(0);
      o_subsystem.TimerStop();
      angleRobot = targetDeg;
      stateAutomatic++;
    }
  }

  public void goForwardDistance(double dist) {
    double tickLeftF = o_subsystem.getEncoderLeft();
    double tickRightF = o_subsystem.getEncoderRight();
    double MID = (tickLeftF + (tickRightF * -1)) / 2;

    double lSpeed = maxSpeed;
    double rSpeed = maxSpeed;
    double distance_to_tick = dist * 11.1;

    if (MID >= distance_to_tick) {
      o_subsystem.TimerStop();
      stateAutomatic++;
    } else {
      if (MID >= (dist - 10) * 11.1) {
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
      o_subsystem.setMotorLeft(lSpeed);
      o_subsystem.setMotorRight(-rSpeed);
    }
  }

  public void goBackDistance(double dist) {
    double tickLeftF = o_subsystem.getEncoderLeft();
    double tickRightF = o_subsystem.getEncoderRight();
    double MID = (tickRightF + (tickLeftF * -1)) / 2;

    double lSpeed = maxSpeed;
    double rSpeed = maxSpeed;
    double distance_to_tick = dist * 9.5;

    if (MID >= distance_to_tick) {
      o_subsystem.TimerStop();
      stateAutomatic++;
    } else {
      if (MID >= (dist - 10) * 9.5) {
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
      o_subsystem.setMotorLeft(-lSpeed);
      o_subsystem.setMotorRight(rSpeed);
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
    o_subsystem.setMotorLeft(lSpeed * slowdown_factor);
    o_subsystem.setMotorRight(-rSpeed * slowdown_factor);
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
    o_subsystem.setMotorLeft(-lSpeed * slowdown_factor);
    o_subsystem.setMotorRight(rSpeed * slowdown_factor);
    if (o_subsystem.getBackSharp() < dist) {
      o_subsystem.TimerStop();
      stateAutomatic++;
    }
  }

  public void goLeftSonic(double dist) {
    if (dist <= 0) {
      o_subsystem.stopAllMotors();
      stateAutomatic++;
      return;
    }

    double sonic = o_subsystem.getDistanceSonicLeft();
    double lSpeed = maxSpeed / 2;
    double rSpeed = maxSpeed / 2;
    double currentAngle = o_subsystem.getYaw();
    if (currentAngle < 0) currentAngle += 360;
    double angleDifference = angleRobot - currentAngle;
    if (angleDifference > 180)  angleDifference -= 360;
    if (angleDifference < -180) angleDifference += 360;
    double slowdown_factor = Math.max(0, Math.min(1.0, (sonic - dist + 10) / dist));
    o_subsystem.setMotorLeft(-lSpeed * slowdown_factor);
    o_subsystem.setMotorRight(-rSpeed * slowdown_factor);
    if (sonic < dist) {
      o_subsystem.TimerStop();
      stateAutomatic++;
    }
  }

  public void goRightSonic(double dist) {
    if (dist <= 0) {
      o_subsystem.stopAllMotors();
      stateAutomatic++;
      return;
    }

    double sonic = o_subsystem.getDistanceSonicRight();
    double lSpeed = maxSpeed / 2;
    double rSpeed = maxSpeed / 2;
    double bSpeed = maxSpeed;
    double currentAngle = o_subsystem.getYaw();
    if (currentAngle < 0) currentAngle += 360;
    double angleDifference = angleRobot - currentAngle;
    if (angleDifference > 180)  angleDifference -= 360;
    if (angleDifference < -180) angleDifference += 360;
    bSpeed -= (angleDifference / 90);
    double slowdown_factor = Math.max(minSpeed, Math.max(0, Math.min(1.0, (sonic - dist + 10) / dist)));
    o_subsystem.setMotorLeft(lSpeed * slowdown_factor);
    o_subsystem.setMotorRight(rSpeed * slowdown_factor);
    o_subsystem.setMotorBack(-bSpeed * slowdown_factor);
    if (sonic < dist) {
      o_subsystem.TimerStop();
      stateAutomatic++;
    }
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
    o_subsystem.setMotorLeft(lSpeed);
    o_subsystem.setMotorRight(-rSpeed);
    if (o_subsystem.getBackSharp() >= dist) {
      o_subsystem.TimerStop();
      stateAutomatic++;
    }
  }

  public void goLeftWithRightSonic(double dist) {
    double sonic = o_subsystem.getDistanceSonicRight();
    double lSpeed = maxSpeed / 2;
    double rSpeed = maxSpeed / 2;
    double bSpeed = maxSpeed;
    double currentAngle = o_subsystem.getYaw();
    if (currentAngle < 0) currentAngle += 360;
    double angleDifference = angleRobot - currentAngle;
    if (angleDifference > 180)  angleDifference -= 360;
    if (angleDifference < -180) angleDifference += 360;
    bSpeed += (angleDifference / 90);
    if (sonic > dist - 10) {
      lSpeed = maxSpeed / 2;
      rSpeed = maxSpeed / 2;
      bSpeed = maxSpeed;
    }
    o_subsystem.setMotorLeft(-lSpeed);
    o_subsystem.setMotorRight(-rSpeed);
    o_subsystem.setMotorBack(bSpeed);
    if (sonic >= dist) {
      o_subsystem.TimerStop();
      stateAutomatic++;
    }
  }

  public void goRigtWithLeftSonic(double dist) {
    double sonic = o_subsystem.getDistanceSonicLeft();
    double lSpeed = maxSpeed / 2;
    double rSpeed = maxSpeed / 2;
    double bSpeed = maxSpeed;
    double currentAngle = o_subsystem.getYaw();
    if (currentAngle < 0) currentAngle += 360;
    double angleDifference = angleRobot - currentAngle;
    if (angleDifference > 180)  angleDifference -= 360;
    if (angleDifference < -180) angleDifference += 360;
    bSpeed -= (angleDifference / 90);
    if (sonic > dist - 10) {
      lSpeed = maxSpeed * 0.25;
      rSpeed = maxSpeed * 0.25;
      bSpeed = maxSpeed * 0.5;
    }
    o_subsystem.setMotorLeft(lSpeed);
    o_subsystem.setMotorRight(rSpeed);
    o_subsystem.setMotorBack(-bSpeed);
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
    double backSpeed = 0;
    if (Math.abs(offsetX) > tolerance) {
      if (offsetX > 0) {
        leftSpeed  =  speed / 2;
        rightSpeed =  speed / 2;
        backSpeed  = -speed;
      } else {
        leftSpeed  = -speed / 2;
        rightSpeed = -speed / 2;
        backSpeed  =  speed;
      }
    } else {
      o_subsystem.TimerStop();
      stateAutomatic++;
      return;
    }
    o_subsystem.setMotorLeft(leftSpeed);
    o_subsystem.setMotorRight(rightSpeed);
    o_subsystem.setMotorBack(backSpeed);
  }

  public void moveLift(int state) {
    if (state == 1) {
      o_subsystem.setMotorGear(0.35);
      if (o_subsystem.getButtonState("up")) {
        o_subsystem.setMotorGear(0);
        stateAutomatic++;
      }
    }
    if (state == 2) {
      o_subsystem.setMotorGear(0.35);
      if (o_subsystem.getButtonState("up")) {
        o_subsystem.setMotorGear(-0.2);
        Timer.delay(4.3);
        o_subsystem.setMotorGear(0);
        stateAutomatic++;
      }
    }
    if (state == 3) {
      o_subsystem.setMotorGear(-0.2);
      if (o_subsystem.getButtonState("down")) {
        o_subsystem.setMotorGear(0);
        stateAutomatic++;
      }
    }
  }

  // ─── Захват ───────────────────────────────────────────────────────────────

  public void open_Hand() {
    o_subsystem.servo_Hook(300);
  }

  public void closed_Hand_Kub() {
    o_subsystem.servo_Hook(100);
  }

  private String driveState = "forward";

  private static final double BBOX_CLOSE_RATIO = 0.95;
  private static final int    SERVO_START      = 300;
  private static final int    SERVO_MAX_PUSH   = 35;
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
        o_subsystem.setMotorLeft(lSpeed);
        o_subsystem.setMotorRight(-rSpeed);
      } else {
        o_subsystem.setMotorLeft(0);
        o_subsystem.setMotorRight(0);
        double turnDegrees = 90.0;
        if (sonicLeft < 28 && sonicRight < 28) turnDegrees = 180.0;
        if (sonicLeft + 10 >= sonicRight) targetAngle = normalizeYaw(currentYaw - turnDegrees);
        else                              targetAngle = normalizeYaw(currentYaw + turnDegrees);
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
        o_subsystem.setMotorLeft(rotateSpeed);
        o_subsystem.setMotorRight(rotateSpeed);
        o_subsystem.setMotorBack(rotateSpeed);
      } else {
        o_subsystem.setMotorLeft(0);
        o_subsystem.setMotorRight(0);
        o_subsystem.setMotorBack(0);
        driveState = "forward";
      }
    }
  }

  @Override
  public void end(final boolean interrupted) {
    o_subsystem.stopAllMotors();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
