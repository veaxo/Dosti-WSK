
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

  private final ExampleSubsystem o_subsystem;
  private final VisionSubsystem visionSubsystem;
  private final OI oi;

  private final SendableChooser<String> autoChooser = new SendableChooser<>();
  private final Map<String, CommandBase> autoMode = new HashMap<>();

  public RobotContainer()
  {
    o_subsystem = new ExampleSubsystem();
    visionSubsystem = new VisionSubsystem(
        () -> o_subsystem.getButtonState("Start"));
    oi = new OI();

    o_subsystem.setDefaultCommand(new Teleop(o_subsystem, oi));

    autoChooser.setDefaultOption("Drive Motor", "Drive Motor");
    autoMode.put("Drive Motor", new DriveMotor(o_subsystem, visionSubsystem));
    SmartDashboard.putData(autoChooser);
  }

  public Command getAutonomousCommand()
  {
    String mode = autoChooser.getSelected();
    SmartDashboard.putString("Chosen Auto Mode", mode);
    CommandBase selected = autoMode.get(mode);
    return selected != null
        ? selected
        : new DriveMotor(o_subsystem, visionSubsystem);
  }
}
