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

  int liftAngle = 0;  
  int handAngle = 300;
  int hookAngle = 300;

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
  double inputRightY = 0;
  double inputRightX = 0;

  double deltaLeftY = 0;
  double deltaLeftX = 0;
  double deltaRightY = 0;
  double deltaRightX = 0;
  double prevLeftY = 0;
  double prevLeftX = 0;
  double prevRightY = 0;
  double prevRightX = 0;

  double leftMotor = 0;
  double rightMotor = 0;
  double backMotor = 0;
  double max = 0;

  private static final double RAMP_UP     = 0.05;

  private static final double RAMP_DOWN   = 0.05;

  private static final double DELTA_LIMIT = 0.075;


  public Teleop(ExampleSubsystem subsystem, OI operatorInterface)
  {
    o_subsystem = subsystem;
    oi = operatorInterface;
    addRequirements(o_subsystem);
  }

  @Override
  public void initialize() 
  {
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
        if(hookAngle > 300)
        hookAngle = 300;
      }

      if(rightBumper)
      {
        hookAngle--;
        if(hookAngle < 100)
        hookAngle = 100;
      }

    }

    if(buttonB)
    {

      if(leftBumper)
      {
        handAngle++;
        if(handAngle > 300)
        handAngle = 300;
      }

      if(rightBumper)
      {
        handAngle--;
        if(handAngle < 150)
        handAngle = 150;
      }

    }

    if(buttonY)
    {

      if(leftBumper)
      {
        liftAngle++;
        if(liftAngle > 300)
          liftAngle = 300;
      }

      if(rightBumper)
      {
        liftAngle--;
        if(liftAngle < 0)
          liftAngle = 0;
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
    
    inputLeftX = oi.getLeftDriveX();
    inputLeftY = - oi.getLeftDriveY();
    inputRightX = oi.getRightDriveX();

    
    deltaLeftX = inputLeftX - prevLeftX;
    deltaLeftY = inputLeftY - prevLeftY;
    deltaRightX = inputRightX - prevRightX;
    if(deltaLeftX >= DELTA_LIMIT)
        inputLeftX += RAMP_UP;
    else if (deltaLeftX <= -DELTA_LIMIT)
        inputLeftX -= RAMP_DOWN;
    if(deltaLeftY >= DELTA_LIMIT)
        inputLeftY += RAMP_UP;
    else if (deltaLeftY <= -DELTA_LIMIT)
        inputLeftY -= RAMP_DOWN;
    if(deltaRightX >= DELTA_LIMIT)
        inputRightX += RAMP_UP;
    else if (deltaRightX <= -DELTA_LIMIT)
        inputRightX -= RAMP_DOWN;
    prevLeftY = inputLeftY;
    prevLeftX = inputLeftX;
    prevRightX = inputRightX;
    
    if(currentGear == 1) {
      o_subsystem.holonomicDrive(inputLeftX * 0.2, inputLeftY * 0.2, inputRightX* 0.1);
    }
    else if(currentGear == 2) {
      o_subsystem.holonomicDrive(inputLeftX * 0.4, inputLeftY * 0.4, inputRightX* 0.2);
    }
    else if(currentGear == 3) {
      o_subsystem.holonomicDrive(inputLeftX * 0.6, inputLeftY * 0.6, inputRightX* 0.4);
    }
    else if(currentGear == 4) {
      o_subsystem.holonomicDrive(inputLeftX * 0.95, inputLeftY * 0.95, inputRightX* 0.725);
    }
    
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
