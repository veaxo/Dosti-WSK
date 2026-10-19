package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.drive.ThreeWheelDrive;

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
  private TitanQuad motorRear;

  private TitanQuadEncoder motorLE;
  private TitanQuadEncoder motorRE;
  private TitanQuadEncoder motorRearE;

  private boolean rotateActive;
  private double rotateSign;
  private double rotateRefLeft;
  private double rotateRefRight;
  private double rotateRefRear;
  private double leftDriveCommand;
  private double rightDriveCommand;
  private double rearDriveCommand;
  private double lastSonicLeftCm = Double.POSITIVE_INFINITY;
  private double lastSonicRightCm = Double.POSITIVE_INFINITY;

  private Servo servoLift;
  private Servo servo_Hook_Hand;
  private Servo servo_Hook;
  private double servoLiftCommand = 185;
  private double servoHandCommand = 30;
  private double servoHookCommand = 15;

  private Ultrasonic sonicLeft;
  private Ultrasonic sonicRight;

  private AnalogInput sharpFrontLeft;
  private AnalogInput sharpFrontRight;



  private ShuffleboardTab sanzhar = Shuffleboard.getTab("Sanzhar");

  private NetworkTableEntry sbSharpFrontLeft = sanzhar.add("Sharp FRONT LEFT cm", 0).getEntry();
  private NetworkTableEntry sbSharpFrontRight = sanzhar.add("Sharp FRONT RIGHT cm", 0).getEntry();

  private NetworkTableEntry sbSonicLeft = sanzhar.add("Sonic LEFT cm", 0).getEntry();
  private NetworkTableEntry sbSonicRight = sanzhar.add("Sonic RIGHT cm", 0).getEntry();
  private NetworkTableEntry sbEncoderLeft = sanzhar.add("Encoder LEFT mm", 0).getEntry();
  private NetworkTableEntry sbEncoderRight = sanzhar.add("Encoder RIGHT mm", 0).getEntry();
  private NetworkTableEntry sbEncoderRear = sanzhar.add("Encoder REAR mm", 0).getEntry();
  private NetworkTableEntry sbEncoderAverage = sanzhar.add("Encoder AVERAGE mm", 0).getEntry();
  private NetworkTableEntry sbDriveLeft = sanzhar.add("Drive LEFT", 0).getEntry();
  private NetworkTableEntry sbDriveRight = sanzhar.add("Drive RIGHT", 0).getEntry();
  private NetworkTableEntry sbDriveRear = sanzhar.add("Drive REAR", 0).getEntry();
  private NetworkTableEntry sbServoLift = sanzhar.add("Servo Lift Angle", 185).getEntry();
  private NetworkTableEntry sbServoArm = sanzhar.add("Servo Arm Angle", 30).getEntry();
  private NetworkTableEntry sbServoClaw = sanzhar.add("Servo Claw Angle", 15).getEntry();
  private NetworkTableEntry sbYaw = sanzhar.add("Yaw", 0).getEntry();

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
    motorLeft = new TitanQuad(Constants.TITAN_ID, Constants.LEFT_MOTOR_CHANNEL);
    motorRight = new TitanQuad(Constants.TITAN_ID, Constants.RIGHT_MOTOR_CHANNEL);
    motorRear = new TitanQuad(Constants.TITAN_ID, Constants.REAR_MOTOR_CHANNEL);


    servoLift = new Servo(3);
    servo_Hook_Hand = new Servo(4);
    servo_Hook = new Servo(5);

    // Pins are set in Constants (SONIC_* / SHARP_*).
    // sonicLeft = new Ultrasonic(
    //     Constants.SONIC_LEFT_PING_DIO, Constants.SONIC_LEFT_ECHO_DIO);
    // sonicRight = new Ultrasonic(
    //     Constants.SONIC_RIGHT_PING_DIO, Constants.SONIC_RIGHT_ECHO_DIO);
    // sonicLeft.setAutomaticMode(true);

    sonicLeft = new Ultrasonic(9,8);
    sonicRight = new Ultrasonic(11,10);
    sonicLeft.setAutomaticMode(true);
    // sonicRight.setAutomaticMode(true);
    
    sharpFrontLeft = new AnalogInput(Constants.SHARP_FRONT_LEFT_AIN);
    sharpFrontRight = new AnalogInput(Constants.SHARP_FRONT_RIGHT_AIN);

    motorLE = new TitanQuadEncoder(
        motorLeft,
        Constants.LEFT_MOTOR_CHANNEL,
        Constants.DRIVE_DISTANCE_PER_TICK_MM);
    motorRE = new TitanQuadEncoder(
        motorRight,
        Constants.RIGHT_MOTOR_CHANNEL,
        Constants.DRIVE_DISTANCE_PER_TICK_MM);
    motorRearE = new TitanQuadEncoder(
        motorRear,
        Constants.REAR_MOTOR_CHANNEL,
        Constants.DRIVE_DISTANCE_PER_TICK_MM);

    gyro = new AHRS(SPI.Port.kMXP);

    buttonStart = new DigitalInput(0);
    ledStart = new DigitalOutput(1);

    buttonStop = new DigitalInput(2);
    ledStop = new DigitalOutput(3);
    
    upLimit = new DigitalInput(6);    
    downLimit = new DigitalInput(7); 

    ledRunning = new DigitalOutput(4);
    ledStopped = new DigitalOutput(5);
  }

  public double getYaw()
  {
      return gyro.getYaw();
  }

  public void resetYaw()
  {
      gyro.reset();
  }

  public void TimerStop() {
    stopDrive();
    resetDriveEncoders();
  }

  public void stopAllMotors() {
    stopDrive();
  }

  public void stopDrive() {
    driveOmni(0, 0, 0);
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
    return (getEncoderLeft() + getEncoderRight()) / 2.0;
  }

  public double getEncoderRear()
  {
    double distance = motorRearE.getEncoderDistance();
    return Constants.REAR_ENCODER_INVERTED ? -distance : distance;
  }

  public void resetEncoder()
  {
    resetDriveEncoders();
  }

  public void resetDriveEncoders() {
    motorLE.reset();
    motorRE.reset();
    motorRearE.reset();
  }

  public void setMotorLeft(double speed)
  {
    double safeSpeed = clamp(speed);
    if (Math.abs(safeSpeed) < 1e-4) {
      stopDrive();
    } else {
      rotateInPlace(safeSpeed);
    }
  }

  public void setMotorRight(double speed)
  {
    double safeSpeed = clamp(speed);
    if (Math.abs(safeSpeed) < 1e-4) {
      stopDrive();
    } else {
      rotateInPlace(-safeSpeed);
    }
  }

  /**
   * Pure in-place rotation: spins all three motors at the exact same speed
   * so the robot turns on the spot (positive = right/clockwise,
   * negative = left/counter-clockwise).
   */
  /**
   * Turns on the spot: positive = clockwise (right). All three motors get
   * the same power in the same direction, and the encoders keep the three
   * wheels at the same speed (a slower wheel gets slightly more power).
   */
  public void rotateInPlace(double turn) {
    double safeTurn = clamp(turn);
    if (Math.abs(safeTurn) < 1e-6 || !Double.isFinite(safeTurn)) {
      rotateActive = false;
      applyWheelCommands(0, 0, 0);
      return;
    }
    double sign = Math.signum(safeTurn);
    double left = Math.abs(getEncoderLeft());
    double right = Math.abs(getEncoderRight());
    double rear = Math.abs(getEncoderRear());
    if (!rotateActive || sign != rotateSign) {
      rotateActive = true;
      rotateSign = sign;
      rotateRefLeft = left;
      rotateRefRight = right;
      rotateRefRear = rear;
    }
    double travelLeft = Math.abs(left - rotateRefLeft);
    double travelRight = Math.abs(right - rotateRefRight);
    double travelRear = Math.abs(rear - rotateRefRear);
    double average = (travelLeft + travelRight + travelRear) / 3.0;
    double power = Math.abs(safeTurn);
    double[] logical = ThreeWheelDrive.rotateInPlace(sign);
    applyWheelCommands(
        logical[0] * clamp(power + rotateSync(average - travelLeft)),
        logical[1] * clamp(power + rotateSync(average - travelRight)),
        logical[2] * clamp(power + rotateSync(average - travelRear)));
  }

  private double rotateSync(double lagMm) {
    return Math.max(-Constants.ROTATE_SYNC_MAX,
        Math.min(Constants.ROTATE_SYNC_MAX, lagMm * Constants.ROTATE_SYNC_KP));
  }

  /**
   * Sets logical left/right wheel power. Positive values always mean forward;
   * if left and right command a turn in place (opposite signs or only one side
   * driven), all three motors spin at the same speed to rotate on the spot.
   */
  public void setDrivePower(double left, double right) {
    double safeLeft = clamp(left);
    double safeRight = clamp(right);
    boolean oppositeSigns = safeLeft * safeRight < 0.0;
    boolean singleSideTurn =
        (Math.abs(safeLeft) < 1e-4) != (Math.abs(safeRight) < 1e-4);
    if (oppositeSigns || singleSideTurn) {
      double turnSpeed = Math.max(Math.abs(safeLeft), Math.abs(safeRight));
      rotateInPlace(Math.copySign(turnSpeed, safeLeft - safeRight));
      return;
    }
    driveOmni((safeLeft + safeRight) / 2.0, 0,
        (safeLeft - safeRight) / 2.0);
  }

  /** Drives the triangular omni base in robot coordinates. */
  public void driveOmni(double forward, double strafe, double turn) {
    if (Math.abs(forward) < 1e-6 && Math.abs(strafe) < 1e-6) {
      rotateInPlace(turn);
      return;
    }
    rotateActive = false;
    double[] powers = ThreeWheelDrive.mix(forward, strafe, turn);
    applyWheelCommands(powers[0], powers[1], powers[2]);
  }

  private void applyWheelCommands(double left, double right, double rear) {
    leftDriveCommand = left;
    rightDriveCommand = right;
    rearDriveCommand = rear;

    double leftOutput = Constants.LEFT_MOTOR_INVERTED
        ? -leftDriveCommand : leftDriveCommand;
    double rightOutput = Constants.RIGHT_MOTOR_INVERTED
        ? -rightDriveCommand : rightDriveCommand;
    double rearOutput = Constants.REAR_MOTOR_INVERTED
        ? -rearDriveCommand : rearDriveCommand;

    motorLeft.set(leftOutput);
    motorRight.set(rightOutput);
    motorRear.set(rearOutput);
  }

  public void tankDrive(double left, double right) {
    setDrivePower(left, right);
  }

  /** Tank sticks plus sideways motion; positive strafe moves right. */
  public void tankDriveWithStrafe(double left, double right, double strafe) {
    double safeLeft = clamp(left);
    double safeRight = clamp(right);
    double safeStrafe = clamp(strafe);
    if (Math.abs(safeStrafe) < 1e-4) {
      boolean oppositeSigns = safeLeft * safeRight < 0.0;
      boolean singleSideTurn =
          (Math.abs(safeLeft) < 1e-4) != (Math.abs(safeRight) < 1e-4);
      if (oppositeSigns || singleSideTurn) {
        double turnSpeed = Math.max(Math.abs(safeLeft), Math.abs(safeRight));
        rotateInPlace(Math.copySign(turnSpeed, safeLeft - safeRight));
        return;
      }
    }
    driveOmni((safeLeft + safeRight) / 2.0, safeStrafe,
        (safeLeft - safeRight) / 2.0);
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

  public double getServoLiftAngle() {
    return servoLiftCommand;
  }

  public double getServoClawAngle() {
    return servoHookCommand;
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

  /** Left side ultrasonic, cm (last valid reading). */
  public double getDistanceSonicLeft() {
    if (sonicLeft.isRangeValid()) {
      lastSonicLeftCm = sonicLeft.getRangeMM() / 10.0;
    }
    return lastSonicLeftCm;
  }

  /** Right side ultrasonic, cm (last valid reading). */
  public double getDistanceSonicRight() {
    if (sonicRight.isRangeValid()) {
      lastSonicRightCm = sonicRight.getRangeMM() / 10.0;
    }
    return lastSonicRightCm;
  }

  private double sharpCm(AnalogInput sharp) {
    return Math.pow(sharp.getAverageVoltage(), -1.2045) * 27.726;
  }

  /** Front-left Sharp IR distance, cm. */
  public double getSharpFrontLeft() {
    return sharpCm(sharpFrontLeft);
  }

  /** Front-right Sharp IR distance, cm. */
  public double getSharpFrontRight() {
    return sharpCm(sharpFrontRight);
  }

  /** Distance straight ahead: the closer of the two front Sharps, cm. */
  public double getForwardSharp() {
    return Math.min(getSharpFrontLeft(), getSharpFrontRight());
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
    sbSharpFrontLeft.setDouble(getSharpFrontLeft());
    sbSharpFrontRight.setDouble(getSharpFrontRight());

    double sonicLeftCm = getDistanceSonicLeft();
    double sonicRightCm = getDistanceSonicRight();
    sbSonicLeft.setDouble(Double.isFinite(sonicLeftCm) ? sonicLeftCm : -1);
    sbSonicRight.setDouble(Double.isFinite(sonicRightCm) ? sonicRightCm : -1);

    sbEncoderLeft.setDouble(getEncoderLeft());
    sbEncoderRight.setDouble(getEncoderRight());
    sbEncoderRear.setDouble(getEncoderRear());
    sbEncoderAverage.setDouble(getAverageDriveDistance());
    sbDriveLeft.setDouble(leftDriveCommand);
    sbDriveRight.setDouble(rightDriveCommand);
    sbDriveRear.setDouble(rearDriveCommand);
    sbServoLift.setDouble(servoLiftCommand);
    sbServoArm.setDouble(servoHandCommand);
    sbServoClaw.setDouble(servoHookCommand);
    sbYaw.setDouble(getYaw());

    sbButStart.setBoolean(getButtonState("Start"));
    sbButStop.setBoolean(getButtonState("Stop"));
    sbButUp.setBoolean(getButtonState("up"));
    sbButDown.setBoolean(getButtonState("down"));
    // sbButReset.setBoolean(getButtonState("Reset"));

  }
}
