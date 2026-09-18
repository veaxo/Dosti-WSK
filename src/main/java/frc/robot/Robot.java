package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.auto.DriveMotor;
import com.studica.frc.MockDS;

public class Robot extends TimedRobot {

    private Command m_autonomousCommand;
    private RobotContainer m_robotContainer;

    @Override
    public void robotInit() {
        MockDS ds = new MockDS();
        ds.enable();

        m_robotContainer = new RobotContainer();

        if (RobotContainer.autoChooser == null) {
            RobotContainer.autoChooser = new SendableChooser<>();
        }
        RobotContainer.autoChooser.setDefaultOption("Drive Motor", "Drive Motor");
        RobotContainer.autoMode.put("Drive Motor", new DriveMotor());
        SmartDashboard.putData(RobotContainer.autoChooser);
    }

    @Override
    public void robotPeriodic() {
        edu.wpi.first.wpilibj2.command.CommandScheduler.getInstance().run();
        CommandScheduler.getInstance().run();
    }

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        if (m_autonomousCommand != null) m_autonomousCommand.schedule();
    }

    @Override public void autonomousPeriodic() {}
    @Override public void teleopInit() {
        if (m_autonomousCommand != null) m_autonomousCommand.cancel();
    }
    @Override public void teleopPeriodic() {}
    @Override public void disabledInit() {}
    @Override public void disabledPeriodic() {}
    @Override public void testInit() { CommandScheduler.getInstance().cancelAll(); }
    @Override public void testPeriodic() {}
}
