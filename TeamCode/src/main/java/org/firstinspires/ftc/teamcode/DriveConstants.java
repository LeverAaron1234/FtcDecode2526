package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class DriveConstants {
  // PID constants for the flywheel / shooter
  public static double p = 19;
  public static double i = 5;
  public static double d = 20;
  public static double f = 0;

  // PID constants for the turret
  public static double spinP = 0.02;
  public static double spinI = 0.0003;
  public static double spinD = 0.001;
}