package frc.robot;

public final class Constants {  

      private Constants() {}

      public static final int TITAN_ID = 42;
      public static final int LEFT_MOTOR_CHANNEL = 0;
      public static final int RIGHT_MOTOR_CHANNEL = 1;
      public static final int LIFT_MOTOR_CHANNEL = 2;

      // Both drive methods use logical wheel direction: positive means that
      // wheel moves the robot forward. The right side is mounted in reverse.
      public static final boolean LEFT_MOTOR_INVERTED = false;
      public static final boolean RIGHT_MOTOR_INVERTED = true;
      public static final boolean LEFT_ENCODER_INVERTED = false;
      public static final boolean RIGHT_ENCODER_INVERTED = true;

      public static final double WHEEL_RADIUS_MM = 55.0;
      public static final double LIFT_GEAR_RADIUS_MM = 12.5;
      public static final int ENCODER_PULSES_PER_ROTATION = 1440;
      public static final double DRIVE_GEAR_RATIO = 1.0;

      public static final double DRIVE_DISTANCE_PER_TICK_MM =
              (Math.PI * 2.0 * WHEEL_RADIUS_MM)
                      / (ENCODER_PULSES_PER_ROTATION * DRIVE_GEAR_RATIO);

      public static final double LIFT_DISTANCE_PER_TICK_MM =
              (Math.PI * 2.0 * LIFT_GEAR_RADIUS_MM)
                      / ENCODER_PULSES_PER_ROTATION;
  }
  
