package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class DriveConstants {
  // PID constants for the flywheel / shooter
  public static double p = 200;
  public static double i = 5;
  public static double d = 20;
  public static double f = 0;

  // PID constants for the turret

  // For the odometry tuning (uses angle differences in degrees)
  public static double spinP = 0.0195;
  public static double spinI = 0.0008;
  public static double spinD = 0.004;
  // For the camera tuning (uses x/y positions on the camera)
  // Camera tuning isn't currently used, so values are really off.
  public static double camSpinP = 0.006;
  public static double camSpinI = 0.003;
  public static double camSpinD = 0.00001;
}