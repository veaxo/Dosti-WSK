package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.util.RobotDiagnostics;

public final class Main {
  private Main() {
  }

  public static void main(String... args) {
    RobotDiagnostics.install();
    try {
      RobotDiagnostics.stage("Starting WPILib HAL");
      RobotBase.startRobot(() -> {
        try {
          RobotDiagnostics.stage("Constructing TimedRobot");
          return new Robot();
        } catch (RuntimeException | Error error) {
          RobotDiagnostics.failure("Robot construction", error);
          throw error;
        }
      });
    } catch (RuntimeException | Error error) {
      RobotDiagnostics.failure("WPILib startup", error);
      throw error;
    }
  }
}
