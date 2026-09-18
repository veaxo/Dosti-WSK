package frc.robot;

public final class Constants {  

      public static final int wheelRadius             = 55;
      public static final double gearRadius = 12.5;

      public static final int pulsePerRotation        = 1440;
  
      public static final double gearRatio            = 1/1;

      public static final double encoderPulseRatio    = pulsePerRotation * gearRatio;
      public static final double encoderGeareRatio = pulsePerRotation * gearRatio;
  
      public static final double distancePerTick      = (Math.PI * 2 * wheelRadius) / encoderPulseRatio;
      public static final double distanceGearTick = (Math.PI * 2 * gearRadius) / encoderGeareRatio;
  } 
  