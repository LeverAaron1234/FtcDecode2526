package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class DriveConstants {
  // PID constants for the flywheel / shooter
  public static double p = 19;
  public static double i = 1.6;
  public static double d = 4;
  public static double f = 0;

  // PID constants for the turret
  public static double spinP = 0.015;
  public static double spinI = 0.0001;
  public static double spinD = 0.002;
}