package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.gamepad.OI;

public class Teleop extends CommandBase
{
  private final ExampleSubsystem o_subsystem;
  private final OI oi;

  public int Transmission = 0;
  public int stateTeleop = 0;

  static int degreesCrane = 0;
  static int degreesLift = 0;
  static int degreesHook = 0;

  int currentGear = 1;

  int liftAngle = 185;
  int handAngle = 30;
  int hookAngle = 15;

  boolean buttonY = false;
  boolean buttonA = false;
  boolean buttonB = false;

  boolean leftBumper = false;
  boolean rightBumper = false;


  boolean back = false;
  boolean start = false;
  boolean previousBack = false;
  boolean previousStart = false;

  double inputLeftY = 0;
  double inputLeftX = 0;
  double inputRightX = 0;

  double prevLeftY = 0;
  double prevLeftX = 0;
  double prevRightX = 0;

  private static final double DRIVE_SLEW_PER_CYCLE = 0.05;


  public Teleop(ExampleSubsystem subsystem, OI operatorInterface)
  {
    o_subsystem = subsystem;
    oi = operatorInterface;
    addRequirements(o_subsystem);
  }

  @Override
  public void initialize() 
  {
    prevLeftY = 0;
    prevLeftX = 0;
    prevRightX = 0;
    o_subsystem.stopDrive();
    o_subsystem.servo_Hook_Hand(handAngle);
    o_subsystem.servo_Hook(hookAngle);
    o_subsystem.setServoLift(liftAngle);
    o_subsystem.setButtonLed("Stopped",true);
    o_subsystem.setButtonLed("Running",true);
  }

  @Override
  public void execute()
  {
    buttonY = oi.getDriveYButton();

    leftBumper = oi.getDriveLeftBumper();
    rightBumper = oi.getDriveRightBumper();

    start = oi.getDriveStartButton();
    back = oi.getDriveBackButton();

    buttonA = oi.getDriveAButton();
    buttonB = oi.getDriveBButton();

    if(buttonA)
    {

      if(leftBumper)
      {
        hookAngle++;
        if(hookAngle > 290)
        hookAngle = 290;
      }

      if(rightBumper)
      {
        hookAngle--;
        if(hookAngle < 15)
        hookAngle = 15;
      }

    }

    if(buttonB)
    {

      if(leftBumper)
      {
        handAngle++;
        if(handAngle > 165)
        handAngle = 165;
      }

      if(rightBumper)
      {
        handAngle--;
        if(handAngle < 30)
        handAngle = 30;
      }

    }

    if(buttonY)
    {

      if(leftBumper)
      {
        liftAngle++;
        if(liftAngle > 185)
          liftAngle = 185;
      }

      if(rightBumper)
      {
        liftAngle--;
        if(liftAngle < 15)
          liftAngle = 15;
      }

    }

    o_subsystem.setServoLift(liftAngle);
    o_subsystem.servo_Hook(hookAngle);
    o_subsystem.servo_Hook_Hand(handAngle);

    if(start && !previousStart) {
      if(currentGear >= 1)
        currentGear++;
      if(currentGear > 4)
        currentGear = 4;

    }

    else if(back && !previousBack) {
      if(currentGear <= 4)
        currentGear--;
      if(currentGear < 1)
        currentGear = 1;

    }

    previousStart = start;
    previousBack = back;
    
    inputLeftY = - oi.getLeftDriveY();
    inputLeftX = oi.getLeftDriveX();
    inputRightX = oi.getRightDriveX();

    inputLeftY = slew(prevLeftY, inputLeftY);
    inputLeftX = slew(prevLeftX, inputLeftX);
    inputRightX = slew(prevRightX, inputRightX);
    prevLeftY = inputLeftY;
    prevLeftX = inputLeftX;
    prevRightX = inputRightX;

    double gearScale;
    if (currentGear == 1)      gearScale = 0.20;
    else if (currentGear == 2) gearScale = 0.40;
    else if (currentGear == 3) gearScale = 0.60;
    else                       gearScale = 0.95;

    // Left stick translates in any direction; right stick X rotates.
    o_subsystem.holonomicDriveWithHeadingHold(
        inputLeftX * gearScale,
        inputLeftY * gearScale,
        inputRightX * gearScale * 0.5);
    
  }

  private double slew(double current, double target)
  {
    double delta = target - current;
    if (delta > DRIVE_SLEW_PER_CYCLE)
      return current + DRIVE_SLEW_PER_CYCLE;
    if (delta < -DRIVE_SLEW_PER_CYCLE)
      return current - DRIVE_SLEW_PER_CYCLE;
    return target;
  }

  @Override
  public void end (boolean interrupted)
  {
    o_subsystem.stopAllMotors();
  }

  @Override
  public boolean isFinished()
  {
    return false;
  }
}
