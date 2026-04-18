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
  public static double spinP = 0.02;
  public static double spinI = 0.0006;
  public static double spinD = 0.002;
  public static double camSpinP = 0.006;
  public static double camSpinI = 0.003;
  public static double camSpinD = 0.00001;
}