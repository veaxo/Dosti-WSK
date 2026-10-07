package frc.robot.util;

import java.io.PrintWriter;
import java.io.StringWriter;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.StandardOpenOption;
import java.time.Instant;

/** Persistent diagnostics for VMX startup errors hidden by MockDS/RobotBase. */
public final class RobotDiagnostics {
  private static volatile String lastStage = "Before robot startup";

  private RobotDiagnostics() {}

  public static void install() {
    Thread.UncaughtExceptionHandler previous = Thread.getDefaultUncaughtExceptionHandler();
    Thread.setDefaultUncaughtExceptionHandler((thread, error) -> {
      failure("Thread " + thread.getName(), error);
      if (previous != null) previous.uncaughtException(thread, error);
    });
    stage("New robot program run");
  }

  public static void stage(String stage) {
    lastStage = stage;
    append(Instant.now() + " STAGE " + stage + System.lineSeparator());
  }

  public static void failure(String location, Throwable error) {
    StringWriter trace = new StringWriter();
    error.printStackTrace(new PrintWriter(trace));
    String message = Instant.now() + " FAILURE " + location
        + "; last stage: " + lastStage + System.lineSeparator() + trace;
    System.err.print(message);
    append(message);
  }

  private static synchronized void append(String message) {
    Path primary = Paths.get(System.getProperty(
        "robot.diagnostics.path", "/home/lvuser/robot_startup.log"));
    if (!write(primary, message)) {
      Path fallback = Paths.get(System.getProperty("java.io.tmpdir"), "robot_startup.log");
      write(fallback, message);
    }
  }

  private static boolean write(Path path, String message) {
    try {
      Files.write(path, message.getBytes(StandardCharsets.UTF_8),
          StandardOpenOption.CREATE, StandardOpenOption.APPEND);
      return true;
    } catch (Exception ignored) {
      // Diagnostics must never replace the original robot failure.
      return false;
    }
  }
}
