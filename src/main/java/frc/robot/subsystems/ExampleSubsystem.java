package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.studica.frc.TitanQuad;
import com.studica.frc.TitanQuadEncoder;
import com.studica.frc.Servo;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;  
import edu.wpi.first.wpilibj.DigitalOutput;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

import frc.robot.Robot;





public class ExampleSubsystem extends SubsystemBase {

  private DigitalInput buttonStart;
  private DigitalInput buttonStop;
  private DigitalInput buttonReset;
  private DigitalInput downLimit;   // нижний концевик
  private DigitalInput upLimit;      // верхний концевик

  private DigitalOutput ledStart;
  private DigitalOutput ledStop;
  private DigitalOutput ledRunning;
  private DigitalOutput ledStopped;
  private DigitalOutput ledReset;
  
  private TitanQuad motorLeft;
  private TitanQuad motorRight;
  private TitanQuad motorBack;
  private TitanQuad motorGear;

  private TitanQuadEncoder motorLE;
  private TitanQuadEncoder motorRE;
  private TitanQuadEncoder motorBE;
  private TitanQuadEncoder motorGE;

  private Servo servoLift;
  private Servo servo_Hook_Hand;
  private Servo servo_Hook;

  private Ultrasonic sonicLeft;
  private Ultrasonic sonicRight;

  private AnalogInput sharpForward;
  private AnalogInput sharpBack;



  private ShuffleboardTab sanzhar = Shuffleboard.getTab("Sanzhar");
 
  private NetworkTableEntry sbSharpForward = sanzhar.add("Sharp Forward", 0).getEntry();
  private NetworkTableEntry sbSharpBack = sanzhar.add("Sharp Back", 0).getEntry();

  private NetworkTableEntry sbSonicLeft = sanzhar.add("Sonic LEFT", 0).getEntry();
  private NetworkTableEntry sbSonicRight = sanzhar.add("Sonic RIGHT", 0).getEntry();
  private NetworkTableEntry dit = sanzhar.add("dit", 0).getEntry();

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
    motorLeft = new TitanQuad(42, 0);
    motorRight = new TitanQuad(42, 1);
    motorBack = new TitanQuad(42, 3);
    motorGear = new TitanQuad(42, 2);


    servoLift = new Servo(3);
    servo_Hook_Hand = new Servo(4);
    servo_Hook = new Servo(5);

    sonicLeft = new Ultrasonic(9, 8);
    sonicRight = new Ultrasonic(11, 10);

    sharpForward = new AnalogInput(0);
    sharpBack = new AnalogInput(1);

    motorLE = new TitanQuadEncoder(motorLeft, 0, Constants.distancePerTick);
    motorRE = new TitanQuadEncoder(motorRight, 1, Constants.distancePerTick);
    motorBE = new TitanQuadEncoder(motorBack, 3, Constants.distancePerTick);
    motorGE = new TitanQuadEncoder(motorGear, 2, Constants.distanceGearTick);

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
    stopAllMotors();
    resetEncoder();
  }

  public void stopAllMotors() {
    setMotorLeft(0);
    setMotorRight(0);
    setMotorBack(0);
    setMotorGear(0);
  }

  public double getEncoderLeft()
  {
    return motorLE.getEncoderDistance();
  }

  public double getEncoderRight()
  {
    return motorRE.getEncoderDistance();
  }

  public double getEncoderBack()
  {
    return motorBE.getEncoderDistance();
  }

  public double getEncoderGear()
  {
    return motorGE.getEncoderDistance();
  }

  public void resetEncoder()
  {
    motorLE.reset();
    motorRE.reset();
    motorBE.reset();
    motorGE.reset();
  }

  public void setMotorLeft(double speed)
  {
    motorLeft.set(speed);
  }

  public void setMotorRight(double speed)
  {
    motorRight.set(speed);
  }

  public void setMotorBack(double speed)
  {
    motorBack.set(speed);
  }

  public void setMotorGear(double speed){
    motorGear.set(speed);
  }

  public void setServoLift(double angle)
  {
    servoLift.setAngle(angle);
  }

  public void servo_Hook_Hand(double angle)
  {
    servo_Hook_Hand.setAngle(angle);
  }
  public void servo_Hook(double angle)
  {
    servo_Hook.setAngle(angle);
  }

 


  public void holonomicDrive(double x, double y, double z)
  {
      double rightSpeed = ((x / 3) - (y / Math.sqrt(3)) + z) * Math.sqrt(3);
      double leftSpeed = ((x / 3) + (y / Math.sqrt(3)) + z) * Math.sqrt(3);
      double backSpeed = (-2 * x / 3) + z;
      double max = Math.abs(rightSpeed);
      if (Math.abs(leftSpeed) > max) max = Math.abs(leftSpeed);
      if (Math.abs(backSpeed) > max) max = Math.abs(backSpeed);
      if (max > 1)
      {
          rightSpeed /= max;
          leftSpeed /= max;
          backSpeed /= max;
      }
      motorLeft.set(leftSpeed);
      motorRight.set(rightSpeed);
      motorBack.set(backSpeed);
  }

  public boolean getButtonState(String state)
  {
    if(state.equals("Start"))
      return !buttonStart.get();
    if(state.equals("Stop"))
      return !buttonStop.get();
    if(state.equals("up"))
      return !upLimit.get();
    if(state.equals("down"))
      return !downLimit.get();
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

  public double getDistanceSonicLeft()
  {
    sonicLeft.ping();
    Timer.delay(0.015);
    return sonicLeft.getRangeMM() / 10;
  }

  public double getDistanceSonicRight()
  {
    sonicRight.ping();
    Timer.delay(0.015);
    return sonicRight.getRangeMM() / 10;
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

    sbSonicLeft.setDouble(getDistanceSonicLeft());
    sbSonicRight.setDouble(getDistanceSonicRight());

    sbButStart.setBoolean(getButtonState("Start"));
    sbButStop.setBoolean(getButtonState("Stop"));
    sbButUp.setBoolean(getButtonState("up"));
    sbButDown.setBoolean(getButtonState("down"));
    // sbButReset.setBoolean(getButtonState("Reset"));

  }
}
