
package frc.robot;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.Teleop;
import frc.robot.commands.auto.DriveMotor;
import frc.robot.gamepad.OI;
import frc.robot.subsystems.ExampleSubsystem;

import frc.robot.subsystems.VisionSubsystem;
  
public class RobotContainer
{

  public static ExampleSubsystem o_subsystem;
  public static VisionSubsystem visionSubsystem;
  public static OI oi;

  public static SendableChooser<String> autoChooser;
  public static Map<String, CommandBase> autoMode = new HashMap<>();

  public RobotContainer()
  {
    o_subsystem = new ExampleSubsystem();
    visionSubsystem = new VisionSubsystem();
    oi = new OI();

    o_subsystem.setDefaultCommand(new Teleop());
  }

  public Command getAutonomousCommand()
  {
    String mode = RobotContainer.autoChooser.getSelected();
    SmartDashboard.putString("Chosen Auto Mode", mode);
    return autoMode.getOrDefault(mode, new DriveMotor());
  }
}
