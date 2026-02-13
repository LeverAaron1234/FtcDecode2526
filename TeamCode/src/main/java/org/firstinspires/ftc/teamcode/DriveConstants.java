package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class DriveConstants {
  public static double p = 15;
  public static double i = 0.8;
  public static double d = 1;
  public static double f = 0;

  public static double spinP = 0.010;
  public static double spinI = 0.001;
  public static double spinD = 0.000;
}

// Small Sized Flywheel:  P: I: D: F:
// Medium Sized Flywheel: P: 15 I:0.8 D: 1 F: 0
// Large Sized Flywheel:  P: I: D: F: