package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import com.studica.frc.MockDS;
import frc.robot.util.RobotDiagnostics;

public class Robot extends TimedRobot {

    private Command m_autonomousCommand;
    private RobotContainer m_robotContainer;
    private MockDS m_mockDS;
    private boolean firstPeriodic = true;

    @Override
    public void startCompetition() {
        try {
            super.startCompetition();
        } catch (RuntimeException | Error error) {
            RobotDiagnostics.failure("Robot main loop", error);
            if (m_robotContainer != null) {
                try {
                    m_robotContainer.stopDriveOnFailure();
                } catch (RuntimeException | Error stopError) {
                    RobotDiagnostics.failure("Stopping drive after failure", stopError);
                }
            }
            throw error;
        }
    }

    @Override
    public void robotInit() {
        RobotDiagnostics.stage("Constructing RobotContainer");
        m_robotContainer = new RobotContainer();

        // WPILib 2020 otherwise polls all registered hardware a second time
        // through LiveWindow, even outside Test mode. Keep our Shuffleboard
        // telemetry, and leave explicit Test-mode LiveWindow behavior intact.
        LiveWindow.disableAllTelemetry();
        RobotDiagnostics.stage("LiveWindow background telemetry disabled");

        // VMX/Studica uses MockDS as its internal Driver Station when the
        // robot is operated without a standard FRC Driver Station.
        RobotDiagnostics.stage("Starting MockDS");
        m_mockDS = new MockDS();
        m_mockDS.enable();
        RobotDiagnostics.stage("robotInit completed");
    }

    @Override
    public void robotPeriodic() {
        if (firstPeriodic) RobotDiagnostics.stage("First scheduler cycle");
        CommandScheduler.getInstance().run();
        if (firstPeriodic) {
            firstPeriodic = false;
            RobotDiagnostics.stage("Scheduler running");
        }
    }

    @Override
    public void autonomousInit() {
        RobotDiagnostics.stage("Starting autonomous command");
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        RobotDiagnostics.stage("Autonomous command selected; scheduling");
        if (m_autonomousCommand != null) m_autonomousCommand.schedule();
        RobotDiagnostics.stage("autonomousInit completed");
    }

    @Override public void autonomousPeriodic() {}
    @Override public void teleopInit() {
        RobotDiagnostics.stage("Starting teleop");
        if (m_autonomousCommand != null) m_autonomousCommand.cancel();
    }
    @Override public void teleopPeriodic() {}
    @Override public void disabledInit() {}
    @Override public void disabledPeriodic() {}
    @Override public void testInit() { CommandScheduler.getInstance().cancelAll(); }
    @Override public void testPeriodic() {}
}
