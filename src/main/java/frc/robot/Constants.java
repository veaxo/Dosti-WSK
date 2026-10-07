package frc.robot;

public final class Constants {  

      private Constants() {}

      public static final int TITAN_ID = 42;
      public static final int LEFT_MOTOR_CHANNEL = 0;
      public static final int RIGHT_MOTOR_CHANNEL = 1;
      public static final int BACK_MOTOR_CHANNEL = 2;

      // Logical wheel axes: left/right point forward, rear points right.
      // Hardware signs match the original three-wheel omni arrangement.
      public static final boolean LEFT_MOTOR_INVERTED = false;
      public static final boolean RIGHT_MOTOR_INVERTED = true;
      public static final boolean BACK_MOTOR_INVERTED = true;
      public static final boolean LEFT_ENCODER_INVERTED = false;
      public static final boolean RIGHT_ENCODER_INVERTED = true;
      public static final boolean BACK_ENCODER_INVERTED = true;

      // Heading control starting values; tune on the real chassis.
      // Output is clockwise wheel power, error is degrees, rate is deg/sec.
      public static final double HEADING_KP = 0.020;
      public static final double HEADING_KI = 0.001;
      public static final double HEADING_KD = 0.002;
      public static final double HEADING_MAX_CORRECTION = 0.20;
      public static final double HEADING_INTEGRAL_LIMIT = 25.0;
      public static final double HEADING_DEADBAND_DEG = 0.30;
      public static final double HEADING_RATE_FILTER_SEC = 0.08;
      public static final double HEADING_SETTLE_RATE_DEG_PER_SEC = 3.0;
      public static final double HEADING_SETTLE_TIME_SEC = 0.15;

      public static final double WHEEL_RADIUS_MM = 55.0;
      public static final int ENCODER_PULSES_PER_ROTATION = 1440;
      public static final double DRIVE_GEAR_RATIO = 1.0;

      public static final double DRIVE_DISTANCE_PER_TICK_MM =
              (Math.PI * 2.0 * WHEEL_RADIUS_MM)
                      / (ENCODER_PULSES_PER_ROTATION * DRIVE_GEAR_RATIO);

  }
  
