package frc.robot;

public final class Constants {  

      private Constants() {}

      public static final int TITAN_ID = 42;
      public static final int LEFT_MOTOR_CHANNEL = 0;
      public static final int RIGHT_MOTOR_CHANNEL = 1;
      public static final int REAR_MOTOR_CHANNEL = 2;

      // Both drive methods use logical wheel direction: positive means that
      // wheel moves the robot forward (rear wheel: moves the robot right).
      // The three motors sit around the triangle the same way, so turning in
      // place must spin all three motors in the same direction. With the
      // rear motor not inverted it spun against the other two and the robot
      // turned around a point beside it instead of on the spot.
      public static final boolean LEFT_MOTOR_INVERTED = false;
      public static final boolean RIGHT_MOTOR_INVERTED = true;
      public static final boolean REAR_MOTOR_INVERTED = true;
      public static final boolean LEFT_ENCODER_INVERTED = false;
      public static final boolean RIGHT_ENCODER_INVERTED = true;
      public static final boolean REAR_ENCODER_INVERTED = true;

      // Turning in place: wheels that lag behind the others (encoder travel)
      // get a little more power so all three turn at the same speed.
      public static final double ROTATE_SYNC_KP = 0.004;   // power per mm
      public static final double ROTATE_SYNC_MAX = 0.08;

      // ── Distance sensors: change the pin numbers here ──
      // Ultrasonic sensors on the LEFT and RIGHT sides (ping DIO, echo DIO).
      // If left and right come out swapped on the "Sanzhar" tab, swap the
      // LEFT and RIGHT numbers.
      public static final int SONIC_LEFT_PING_DIO = 10;
      public static final int SONIC_LEFT_ECHO_DIO = 11;
      public static final int SONIC_RIGHT_PING_DIO = 9;
      public static final int SONIC_RIGHT_ECHO_DIO = 8;
      // Sharp IR sensors at the FRONT, left and right (analog inputs).
      public static final int SHARP_FRONT_LEFT_AIN = 0;
      public static final int SHARP_FRONT_RIGHT_AIN = 1;

      public static final double WHEEL_RADIUS_MM = 55.0;
      public static final int ENCODER_PULSES_PER_ROTATION = 1440;
      public static final double DRIVE_GEAR_RATIO = 1.0;

      public static final double DRIVE_DISTANCE_PER_TICK_MM =
              (Math.PI * 2.0 * WHEEL_RADIUS_MM)
                      / (ENCODER_PULSES_PER_ROTATION * DRIVE_GEAR_RATIO);
  }
  
