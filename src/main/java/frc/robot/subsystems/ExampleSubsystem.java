package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

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
  private TitanQuad motorGear;

  private TitanQuadEncoder motorLE;
  private TitanQuadEncoder motorRE;
  private TitanQuadEncoder motorGE;

  private double leftDriveCommand;
  private double rightDriveCommand;
  private double liftCommand;
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
  private NetworkTableEntry sbEncoderAverage = sanzhar.add("Encoder AVERAGE mm", 0).getEntry();
  private NetworkTableEntry sbDriveLeft = sanzhar.add("Drive LEFT", 0).getEntry();
  private NetworkTableEntry sbDriveRight = sanzhar.add("Drive RIGHT", 0).getEntry();
  private NetworkTableEntry sbLiftCommand = sanzhar.add("Lift Power", 0).getEntry();
  private NetworkTableEntry sbLiftEncoder = sanzhar.add("Lift Encoder", 0).getEntry();
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
    motorGear = new TitanQuad(Constants.TITAN_ID, Constants.LIFT_MOTOR_CHANNEL);


    servoLift = new Servo(3);
    servo_Hook_Hand = new Servo(4);
    servo_Hook = new Servo(5);

    sonicFront = new Ultrasonic(9, 8);
    sonicRear = new Ultrasonic(11, 10);
    sonicFront.setAutomaticMode(true);

    sharpForward = new AnalogInput(0);
    sharpBack = new AnalogInput(1);

    motorLE = new TitanQuadEncoder(
        motorLeft,
        Constants.LEFT_MOTOR_CHANNEL,
        Constants.DRIVE_DISTANCE_PER_TICK_MM);
    motorRE = new TitanQuadEncoder(
        motorRight,
        Constants.RIGHT_MOTOR_CHANNEL,
        Constants.DRIVE_DISTANCE_PER_TICK_MM);
    motorGE = new TitanQuadEncoder(
        motorGear,
        Constants.LIFT_MOTOR_CHANNEL,
        Constants.LIFT_DISTANCE_PER_TICK_MM);

    gyro = new AHRS(SPI.Port.kMXP);

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
      gyro.reset();
  }

  public void TimerStop() {
    stopDrive();
    resetDriveEncoders();
  }

  public void stopAllMotors() {
    stopDrive();
    setMotorGear(0);
  }

  public void stopDrive() {
    setDrivePower(0, 0);
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

  public double getEncoderGear()
  {
    return motorGE.getEncoderDistance();
  }

  public void resetEncoder()
  {
    resetDriveEncoders();
    motorGE.reset();
  }

  public void resetDriveEncoders() {
    motorLE.reset();
    motorRE.reset();
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
   * Sets logical left/right wheel power. Positive values always mean forward;
   * physical motor inversion is handled here and nowhere else.
   */
  public void setDrivePower(double left, double right) {
    leftDriveCommand = clamp(left);
    rightDriveCommand = clamp(right);

    double leftOutput = Constants.LEFT_MOTOR_INVERTED
        ? -leftDriveCommand : leftDriveCommand;
    double rightOutput = Constants.RIGHT_MOTOR_INVERTED
        ? -rightDriveCommand : rightDriveCommand;

    motorLeft.set(leftOutput);
    motorRight.set(rightOutput);
  }

  public void tankDrive(double left, double right) {
    setDrivePower(left, right);
  }

  private double clamp(double value) {
    return Math.max(-1.0, Math.min(1.0, value));
  }

  public void setMotorGear(double speed){
    double safeSpeed = clamp(speed);

    if ((safeSpeed > 0 && isLiftUpperLimitPressed())
        || (safeSpeed < 0 && isLiftLowerLimitPressed())) {
      safeSpeed = 0;
    }

    liftCommand = safeSpeed;
    motorGear.set(safeSpeed);
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
    sbSharpForward.setDouble(getForwardSharp());
    sbSharpBack.setDouble(getBackSharp());

    double sonicFrontCm = getDistanceSonicFront();
    double sonicRearCm = getDistanceSonicRear();
    sbSonicFront.setDouble(Double.isFinite(sonicFrontCm) ? sonicFrontCm : -1);
    sbSonicRear.setDouble(Double.isFinite(sonicRearCm) ? sonicRearCm : -1);

    sbEncoderLeft.setDouble(getEncoderLeft());
    sbEncoderRight.setDouble(getEncoderRight());
    sbEncoderAverage.setDouble(getAverageDriveDistance());
    sbDriveLeft.setDouble(leftDriveCommand);
    sbDriveRight.setDouble(rightDriveCommand);
    sbLiftCommand.setDouble(liftCommand);
    sbLiftEncoder.setDouble(getEncoderGear());
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
