package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.HeadingController;
import frc.robot.util.RobotDiagnostics;
import frc.robot.util.ThreeWheelDrive;

import com.studica.frc.TitanQuad;
import com.studica.frc.TitanQuadEncoder;
import com.studica.frc.Servo;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;  
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Timer;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

public class ExampleSubsystem extends SubsystemBase {

  // Physical servo ranges on this robot. Keep these limits at the lowest
  // hardware layer so autonomous and Teleop cannot command a hard stop.
  private static final double LIFT_SERVO_MIN_ANGLE = 15.0;
  private static final double LIFT_SERVO_MAX_ANGLE = 300.0;
  private static final double ARM_SERVO_MIN_ANGLE = 30.0;
  private static final double ARM_SERVO_MAX_ANGLE = 165.0;
  private static final double HOOK_SERVO_MIN_ANGLE = 20.0;
  private static final double HOOK_SERVO_MAX_ANGLE = 300.0;

  private DigitalInput buttonStart;
  private DigitalInput buttonStop;
  private DigitalInput downLimit;   // нижний концевик
  private DigitalInput upLimit;      // верхний концевик

  private DigitalOutput ledStart;
  private DigitalOutput ledStop;
  private DigitalOutput ledRunning;
  private DigitalOutput ledStopped;
  
  private TitanQuad motorLeft;
  private TitanQuad motorRight;
  private TitanQuad motorBack;

  private TitanQuadEncoder motorLE;
  private TitanQuadEncoder motorRE;
  private TitanQuadEncoder motorBE;
  private boolean firstEncoderReset = true;
  private static final double TELEMETRY_PERIOD_SEC = 0.10;
  private double nextTelemetryAt;

  private double leftDriveCommand;
  private double rightDriveCommand;
  private double backDriveCommand;
  private final HeadingController headingController = new HeadingController();
  private double lastSonicFrontCm = Double.POSITIVE_INFINITY;
  private double lastSonicRearCm = Double.POSITIVE_INFINITY;
  private double lastSonicFrontUpdateSec = Double.NEGATIVE_INFINITY;
  private double lastSonicRearUpdateSec = Double.NEGATIVE_INFINITY;

  private Servo servoLift;
  private Servo servo_Hook_Hand;
  private Servo servo_Hook;
  private double servoLiftCommand = 185;
  private double servoHandCommand = 30;
  private double servoHookCommand = 15;

  private Ultrasonic sonicFront;
  private Ultrasonic sonicRear;

  private AnalogInput sharpForward;
  private AnalogInput sharpBack;



  private ShuffleboardTab sanzhar = Shuffleboard.getTab("Sanzhar");

  private NetworkTableEntry sbSharpForward = sanzhar.add("Sharp Forward", 0).getEntry();
  private NetworkTableEntry sbSharpBack = sanzhar.add("Sharp Back", 0).getEntry();

  private NetworkTableEntry sbSonicFront = sanzhar.add("Sonic FRONT cm", 0).getEntry();
  private NetworkTableEntry sbSonicRear = sanzhar.add("Sonic REAR cm", 0).getEntry();
  private NetworkTableEntry sbEncoderLeft = sanzhar.add("Encoder LEFT mm", 0).getEntry();
  private NetworkTableEntry sbEncoderRight = sanzhar.add("Encoder RIGHT mm", 0).getEntry();
  private NetworkTableEntry sbEncoderBack = sanzhar.add("Encoder BACK mm", 0).getEntry();
  private NetworkTableEntry sbEncoderAverage = sanzhar.add("Encoder AVERAGE mm", 0).getEntry();
  private NetworkTableEntry sbEncoderStrafe = sanzhar.add("Encoder STRAFE mm", 0).getEntry();
  private NetworkTableEntry sbDriveLeft = sanzhar.add("Drive LEFT", 0).getEntry();
  private NetworkTableEntry sbDriveRight = sanzhar.add("Drive RIGHT", 0).getEntry();
  private NetworkTableEntry sbDriveBack = sanzhar.add("Drive BACK", 0).getEntry();
  private NetworkTableEntry sbServoLift = sanzhar.add("Servo Lift Angle", 185).getEntry();
  private NetworkTableEntry sbServoArm = sanzhar.add("Servo Arm Angle", 30).getEntry();
  private NetworkTableEntry sbServoClaw = sanzhar.add("Servo Claw Angle", 15).getEntry();
  private NetworkTableEntry sbYaw = sanzhar.add("Yaw", 0).getEntry();
  private NetworkTableEntry sbGyroReady = sanzhar.add("Gyro Ready", false).getEntry();
  private NetworkTableEntry sbGyroState = sanzhar.add("Gyro State", "STARTING").getEntry();
  private NetworkTableEntry sbYawRate = sanzhar.add("Yaw Rate deg per sec", 0).getEntry();
  private NetworkTableEntry sbHeadingTarget = sanzhar.add("Heading Target deg", 0).getEntry();
  private NetworkTableEntry sbHeadingError = sanzhar.add("Heading Error deg", 0).getEntry();
  private NetworkTableEntry sbHeadingCorrection = sanzhar.add("Heading Correction", 0).getEntry();
  private NetworkTableEntry sbHeadingMode = sanzhar.add("Heading Mode", "STOPPED").getEntry();

  private NetworkTableEntry sbButStart = sanzhar.add("BUTTON START", false).getEntry();
  private NetworkTableEntry sbButStop = sanzhar.add("BUTTON STOP", false).getEntry();
  // private NetworkTableEntry sbButReset = sanzhar.add("BUTTON RESET", false).getEntry();
  private NetworkTableEntry sbButUp = sanzhar.add("BUTTON UP", false).getEntry();
  private NetworkTableEntry sbButDown = sanzhar.add("BUTTON DOWN", false).getEntry();

  private NetworkTableEntry sbSeveScan = sanzhar.add("Saved Scan", "None").getEntry();
  private NetworkTableEntry sbDetected_1 = sanzhar.add("Detected 1", "None").getEntry();
  private NetworkTableEntry sbDetected_2 = sanzhar.add("Detected 2", "None").getEntry();
  private NetworkTableEntry sbDetected_3 = sanzhar.add("Detected 3", "None").getEntry();
  private NetworkTableEntry sbDetected_4 = sanzhar.add("Detected 4", "None").getEntry();

  AHRS gyro;

  public ExampleSubsystem() 
  {
    RobotDiagnostics.stage("Titan motors: left M" + Constants.LEFT_MOTOR_CHANNEL
        + ", right M" + Constants.RIGHT_MOTOR_CHANNEL + ", rear M" + Constants.BACK_MOTOR_CHANNEL);
    motorLeft = new TitanQuad(Constants.TITAN_ID, Constants.LEFT_MOTOR_CHANNEL);
    motorRight = new TitanQuad(Constants.TITAN_ID, Constants.RIGHT_MOTOR_CHANNEL);
    motorBack = new TitanQuad(Constants.TITAN_ID, Constants.BACK_MOTOR_CHANNEL);


    RobotDiagnostics.stage("Creating servos 3, 4, 5");
    servoLift = new Servo(3);
    servo_Hook_Hand = new Servo(4);
    servo_Hook = new Servo(5);

    RobotDiagnostics.stage("Creating ultrasonic and Sharp sensors");
    sonicFront = new Ultrasonic(9, 8);
    sonicRear = new Ultrasonic(11, 10);
    sonicFront.setAutomaticMode(true);

    sharpForward = new AnalogInput(0);
    sharpBack = new AnalogInput(1);

    RobotDiagnostics.stage("Creating Titan encoders");
    motorLE = new TitanQuadEncoder(
        motorLeft,
        Constants.LEFT_MOTOR_CHANNEL,
        Constants.DRIVE_DISTANCE_PER_TICK_MM);
    motorRE = new TitanQuadEncoder(
        motorRight,
        Constants.RIGHT_MOTOR_CHANNEL,
        Constants.DRIVE_DISTANCE_PER_TICK_MM);
    motorBE = new TitanQuadEncoder(
        motorBack,
        Constants.BACK_MOTOR_CHANNEL,
        Constants.DRIVE_DISTANCE_PER_TICK_MM);

    RobotDiagnostics.stage("Creating navX on SPI kMXP");
    gyro = new AHRS(SPI.Port.kMXP);

    RobotDiagnostics.stage("Creating buttons and LEDs");
    buttonStart = new DigitalInput(0);
    ledStart = new DigitalOutput(1);

    buttonStop = new DigitalInput(2);
    ledStop = new DigitalOutput(3);
    
    upLimit = new DigitalInput(6);    
    downLimit = new DigitalInput(7); 

    ledRunning = new DigitalOutput(5);
    ledStopped = new DigitalOutput(4);
  }

  public double getYaw()
  {
      return gyro.getYaw();
  }

  public void resetYaw()
  {
      headingController.reset();
      gyro.reset();
  }

  public boolean isGyroReady() {
    return gyro.isConnected() && !gyro.isCalibrating()
        && Double.isFinite(getYaw()) && Double.isFinite(gyro.getRate());
  }

  public void TimerStop() {
    stopDrive();
    resetDriveEncoders();
  }

  public void stopAllMotors() {
    stopDrive();
  }

  public void stopDrive() {
    setDrivePower(0, 0);
    sbHeadingMode.setString("STOPPED");
  }

  public double getEncoderLeft()
  {
    double distance = motorLE.getEncoderDistance();
    return Constants.LEFT_ENCODER_INVERTED ? -distance : distance;
  }

  public double getEncoderRight()
  {
    double distance = motorRE.getEncoderDistance();
    return Constants.RIGHT_ENCODER_INVERTED ? -distance : distance;
  }

  public double getAverageDriveDistance() {
    return ThreeWheelDrive.forwardDistance(getEncoderLeft(), getEncoderRight());
  }

  public double getEncoderBack() {
    double distance = motorBE.getEncoderDistance();
    return Constants.BACK_ENCODER_INVERTED ? -distance : distance;
  }

  public double getStrafeDriveDistance() {
    return ThreeWheelDrive.strafeDistance(
        getEncoderLeft(), getEncoderRight(), getEncoderBack());
  }

  public void resetEncoder()
  {
    resetDriveEncoders();
  }

  public void resetDriveEncoders() {
    if (firstEncoderReset) RobotDiagnostics.stage(
        "Resetting LEFT Titan encoder M" + Constants.LEFT_MOTOR_CHANNEL);
    motorLE.reset();
    if (firstEncoderReset) RobotDiagnostics.stage(
        "Resetting RIGHT Titan encoder M" + Constants.RIGHT_MOTOR_CHANNEL);
    motorRE.reset();
    if (firstEncoderReset) RobotDiagnostics.stage(
        "Resetting REAR Titan encoder M" + Constants.BACK_MOTOR_CHANNEL);
    motorBE.reset();
    if (firstEncoderReset) {
      firstEncoderReset = false;
      RobotDiagnostics.stage("Titan encoder reset completed");
    }
  }

  public void setMotorLeft(double speed)
  {
    setDrivePower(speed, rightDriveCommand);
  }

  public void setMotorRight(double speed)
  {
    setDrivePower(leftDriveCommand, speed);
  }

  /**
   * Compatibility for autonomous and VisionPickup forward/turn commands.
   * Equal values move forward; left > right rotates clockwise using all three
   * wheels. The rear wheel is stationary during straight forward travel.
   */
  public void setDrivePower(double left, double right) {
    double logicalLeft = clamp(left);
    double logicalRight = clamp(right);
    holonomicDrive(0.0,
        (logicalLeft + logicalRight) / (2.0 * ThreeWheelDrive.FORWARD_PROJECTION),
        (logicalLeft - logicalRight) / 2.0);
  }

  /** Robot-relative drive: rightward strafe, forward travel, clockwise turn. */
  public void holonomicDrive(double strafe, double forward, double clockwise) {
    // Direct commands, including VisionPickup, never use gyro correction.
    headingController.reset();
    sbHeadingMode.setString("DIRECT");
    applyHolonomicDrive(strafe, forward, clockwise);
  }

  /** Holds an explicit autonomous course; speed keeps the legacy wheel-power scale. */
  public void driveStraightWithHeading(double speed, double targetYaw) {
    driveWithHeading(0.0, speed / ThreeWheelDrive.FORWARD_PROJECTION, targetYaw);
  }

  public void driveWithHeading(double strafe, double forward, double targetYaw) {
    if (!Double.isFinite(strafe) || !Double.isFinite(forward)
        || !Double.isFinite(targetYaw)
        || Math.hypot(strafe, forward) < 1e-6) {
      stopDrive();
      return;
    }
    if (!prepareGyroDrive()) return;
    double correction = headingController.calculate(
        targetYaw, getYaw(), gyro.getRate(), Timer.getFPGATimestamp());
    applyHolonomicDrive(strafe, forward, correction);
    sbHeadingMode.setString("AUTO_HOLD");
  }

  /** Teleop: manual turns take precedence; otherwise latch and hold the current course. */
  public void holonomicDriveWithHeadingHold(double strafe, double forward, double clockwise) {
    if (!Double.isFinite(strafe) || !Double.isFinite(forward)
        || !Double.isFinite(clockwise)) {
      stopDrive();
      return;
    }
    if (Math.hypot(strafe, forward) < 1e-6 && Math.abs(clockwise) < 1e-6) {
      stopDrive();
      return;
    }
    if (!prepareGyroDrive()) return;
    if (Math.abs(clockwise) >= 1e-6) {
      headingController.manualTurn();
      applyHolonomicDrive(strafe, forward, clockwise);
      sbHeadingMode.setString("MANUAL_TURN");
      return;
    }
    double correction = headingController.hold(
        getYaw(), gyro.getRate(), Timer.getFPGATimestamp());
    applyHolonomicDrive(strafe, forward, correction);
    sbHeadingMode.setString(headingController.isHolding()
        ? "TELEOP_HOLD" : "WAITING_TURN_SETTLE");
  }

  private boolean prepareGyroDrive() {
    if (isGyroReady()) return true;
    stopDrive();
    sbHeadingMode.setString("WAITING_FOR_GYRO");
    return false;
  }

  private void applyHolonomicDrive(double strafe, double forward, double clockwise) {
    double[] powers = ThreeWheelDrive.calculate(strafe, forward, clockwise);
    leftDriveCommand = powers[0];
    rightDriveCommand = powers[1];
    backDriveCommand = powers[2];

    double leftOutput = Constants.LEFT_MOTOR_INVERTED
        ? -leftDriveCommand : leftDriveCommand;
    double rightOutput = Constants.RIGHT_MOTOR_INVERTED
        ? -rightDriveCommand : rightDriveCommand;
    double backOutput = Constants.BACK_MOTOR_INVERTED
        ? -backDriveCommand : backDriveCommand;

    motorLeft.set(leftOutput);
    motorRight.set(rightOutput);
    motorBack.set(backOutput);
  }

  public void tankDrive(double left, double right) {
    setDrivePower(left, right);
  }

  private double clamp(double value) {
    return Math.max(-1.0, Math.min(1.0, value));
  }

  public boolean isLiftUpperLimitPressed() {
    return !upLimit.get();
  }

  public boolean isLiftLowerLimitPressed() {
    return !downLimit.get();
  }

  public void setServoLift(double angle)
  {
    servoLiftCommand = clampLiftServoAngle(angle);
    servoLift.setAngle(servoLiftCommand);
  }

  public void servo_Hook_Hand(double angle)
  {
    servoHandCommand = clampArmServoAngle(angle);
    servo_Hook_Hand.setAngle(servoHandCommand);
  }

  public double getServoArmAngle() {
    return servoHandCommand;
  }

  public void servo_Hook(double angle)
  {
    servoHookCommand = clampHookServoAngle(angle);
    servo_Hook.setAngle(servoHookCommand);
  }

  public boolean moveServoLiftToward(double target, double maxStep) {
    double safeTarget = clampLiftServoAngle(target);
    double next = moveToward(servoLiftCommand, safeTarget, maxStep);
    setServoLift(next);
    return Math.abs(servoLiftCommand - safeTarget) < 0.001;
  }

  public boolean moveServoArmToward(double target, double maxStep) {
    double safeTarget = clampArmServoAngle(target);
    double next = moveToward(servoHandCommand, safeTarget, maxStep);
    servo_Hook_Hand(next);
    return Math.abs(servoHandCommand - safeTarget) < 0.001;
  }

  public boolean moveServoClawToward(double target, double maxStep) {
    double safeTarget = clampHookServoAngle(target);
    double next = moveToward(servoHookCommand, safeTarget, maxStep);
    servo_Hook(next);
    return Math.abs(servoHookCommand - safeTarget) < 0.001;
  }

  private double moveToward(double current, double target, double maxStep) {
    double step = Math.max(0.1, Math.abs(maxStep));
    if (current < target) return Math.min(current + step, target);
    if (current > target) return Math.max(current - step, target);
    return target;
  }

  private double clampLiftServoAngle(double angle) {
    return Math.max(
        LIFT_SERVO_MIN_ANGLE,
        Math.min(LIFT_SERVO_MAX_ANGLE, angle));
  }

  private double clampArmServoAngle(double angle) {
    return Math.max(
        ARM_SERVO_MIN_ANGLE,
        Math.min(ARM_SERVO_MAX_ANGLE, angle));
  }

  private double clampHookServoAngle(double angle) {
    return Math.max(
        HOOK_SERVO_MIN_ANGLE,
        Math.min(HOOK_SERVO_MAX_ANGLE, angle));
  }

  public boolean getButtonState(String state)
  {
    if(state.equals("Start"))
      return !buttonStart.get();
    if(state.equals("Stop"))
      return !buttonStop.get();
    if(state.equals("up"))
      return isLiftUpperLimitPressed();
    if(state.equals("down"))
      return isLiftLowerLimitPressed();
    else
      return false;
      
  }

  public void setButtonLed(String led, boolean state)
  {
    if(led.equals("Start"))
      ledStart.set(state);
    if(led.equals("Stop"))
      ledStop.set(state);
    // if(led.equals("Reset"))
    //   ledReset.set(state);
    if(led.equals("Running"))
      ledRunning.set(state);
    if(led.equals("Stopped"))
      ledStopped.set(state);
  }

  public double getDistanceSonicFront()
  {
    if (sonicFront.isRangeValid()) {
      lastSonicFrontCm = sonicFront.getRangeMM() / 10.0;
      lastSonicFrontUpdateSec = Timer.getFPGATimestamp();
    }
    return lastSonicFrontCm;
  }

  public boolean isFrontSonicRangeValid() {
    return Double.isFinite(lastSonicFrontCm)
        && Timer.getFPGATimestamp() - lastSonicFrontUpdateSec <= 0.25;
  }

  public double getDistanceSonicRear()
  {
    if (sonicRear.isRangeValid()) {
      lastSonicRearCm = sonicRear.getRangeMM() / 10.0;
      lastSonicRearUpdateSec = Timer.getFPGATimestamp();
    }
    return lastSonicRearCm;
  }

  // Compatibility aliases for the existing autonomous route.
  public double getDistanceSonicLeft() {
    return getDistanceSonicFront();
  }

  public double getDistanceSonicRight() {
    return getDistanceSonicRear();
  }

  public double getForwardSharp()
 {
    return (Math.pow(sharpForward.getAverageVoltage(), -1.2045)) * 27.726;
 }

 public double getBackSharp()
 {
    return (Math.pow(sharpBack.getAverageVoltage(), -1.2045)) * 27.726;
 }

 public void setSeveScan(String value) {
  sbSeveScan.setString(value != null ? value : "None");
  }

  public void setDetected_1(String value) {
    sbDetected_1.setString(value != null ? value : "None");
  }

  public void setDetected_2(String value) {
    sbDetected_2.setString(value != null ? value : "None");
  }

  public void setDetected_3(String value) {
    sbDetected_3.setString(value != null ? value : "None");
  }

  public void setDetected_4(String value) {
    sbDetected_4.setString(value != null ? value : "None");
  }
  


  @Override
  public void periodic() 
  {
    // Limit dashboard work only; drive commands and their sensor reads still
    // run on every scheduler cycle. Never throttle VisionPickup processing.
    double now = Timer.getFPGATimestamp();
    if (now < nextTelemetryAt) return;
    nextTelemetryAt = now + TELEMETRY_PERIOD_SEC;

    sbSharpForward.setDouble(getForwardSharp());
    sbSharpBack.setDouble(getBackSharp());

    double sonicFrontCm = getDistanceSonicFront();
    double sonicRearCm = getDistanceSonicRear();
    sbSonicFront.setDouble(Double.isFinite(sonicFrontCm) ? sonicFrontCm : -1);
    sbSonicRear.setDouble(Double.isFinite(sonicRearCm) ? sonicRearCm : -1);

    // Each Titan/JNI encoder read is needed only once for this snapshot.
    double leftDistance = getEncoderLeft();
    double rightDistance = getEncoderRight();
    double backDistance = getEncoderBack();
    sbEncoderLeft.setDouble(leftDistance);
    sbEncoderRight.setDouble(rightDistance);
    sbEncoderBack.setDouble(backDistance);
    sbEncoderAverage.setDouble(ThreeWheelDrive.forwardDistance(leftDistance, rightDistance));
    sbEncoderStrafe.setDouble(ThreeWheelDrive.strafeDistance(
        leftDistance, rightDistance, backDistance));
    sbDriveLeft.setDouble(leftDriveCommand);
    sbDriveRight.setDouble(rightDriveCommand);
    sbDriveBack.setDouble(backDriveCommand);
    sbServoLift.setDouble(servoLiftCommand);
    sbServoArm.setDouble(servoHandCommand);
    sbServoClaw.setDouble(servoHookCommand);
    sbYaw.setDouble(getYaw());
    sbGyroReady.setBoolean(isGyroReady());
    sbGyroState.setString(!gyro.isConnected() ? "DISCONNECTED"
        : gyro.isCalibrating() ? "CALIBRATING" : "READY");
    sbYawRate.setDouble(gyro.getRate());
    sbHeadingTarget.setDouble(headingController.getTarget());
    sbHeadingError.setDouble(headingController.getError());
    sbHeadingCorrection.setDouble(headingController.getCorrection());

    sbButStart.setBoolean(getButtonState("Start"));
    sbButStop.setBoolean(getButtonState("Stop"));
    sbButUp.setBoolean(getButtonState("up"));
    sbButDown.setBoolean(getButtonState("down"));
    // sbButReset.setBoolean(getButtonState("Reset"));

  }
}
