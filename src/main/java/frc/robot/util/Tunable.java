package frc.robot.util;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/**
 * A number shown on the "Pickup Tuning" Shuffleboard tab that can be edited
 * while the robot runs. The code reads the current value every time.
 *
 * Values typed in Shuffleboard are lost when the robot program restarts:
 * once a value works, copy it into the default in the code and deploy.
 */
public final class Tunable {
  private static final ShuffleboardTab TAB = Shuffleboard.getTab("Pickup Tuning");

  private final NetworkTableEntry entry;
  private final double defaultValue;

  public Tunable(String name, double defaultValue) {
    this.defaultValue = defaultValue;
    this.entry = TAB.add(name, defaultValue).getEntry();
  }

  public double get() {
    return entry.getDouble(defaultValue);
  }
}
