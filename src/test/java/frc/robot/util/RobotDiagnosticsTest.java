package frc.robot.util;

import static org.junit.Assert.assertTrue;

import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;

public class RobotDiagnosticsTest {
  @Rule
  public TemporaryFolder temporary = new TemporaryFolder();

  @Test
  public void persistsStartupStageAndOriginalFailureCause() throws Exception {
    Path log = temporary.newFile("robot_startup.log").toPath();
    String previous = System.getProperty("robot.diagnostics.path");
    try {
      System.setProperty("robot.diagnostics.path", log.toString());
      RobotDiagnostics.stage("Creating Titan encoders");
      RobotDiagnostics.failure("Robot main loop",
          new IllegalStateException("Startup failed", new RuntimeException("Original cause")));
      String text = new String(Files.readAllBytes(log), StandardCharsets.UTF_8);
      assertTrue(text.contains("Creating Titan encoders"));
      assertTrue(text.contains("Robot main loop"));
      assertTrue(text.contains("IllegalStateException: Startup failed"));
      assertTrue(text.contains("Caused by: java.lang.RuntimeException: Original cause"));
    } finally {
      if (previous == null) System.clearProperty("robot.diagnostics.path");
      else System.setProperty("robot.diagnostics.path", previous);
    }
  }
}
