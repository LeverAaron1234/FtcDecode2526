package org.firstinspires.ftc.teamcode.appendeges;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.DriveConstants;

public class TurretSpinner {
  private CRServo spin;

  private double kP = DriveConstants.spinP;
  private double kI = DriveConstants.spinI;
  private double kIgain = 0.0;
  private double kD = DriveConstants.spinD;
  private double goalX = 0.0;
  private double lastError = 0.0;
  private double angleTolerance = 2;
  private final double MAX_POWER = 0.6;
  private double power = 0;

  private final ElapsedTime timer = new ElapsedTime();

  public void init(HardwareMap hardwareMap) {
    spin = hardwareMap.get(CRServo.class, "spin");
    spin.setDirection(CRServo.Direction.REVERSE);
  }

  public void resetTimer() {
    timer.reset();
  }

  public double update(LLResult result, boolean leftPressed, boolean rightPressed) {
    kP = DriveConstants.spinP;
    kP = DriveConstants.spinI;
    kD = DriveConstants.spinD;
    double deltaTime = timer.seconds();
    timer.reset();

    if (!result.isValid()) {
      spin.setPower(0);
      lastError = 0;
      return 0;
    }

    double error = goalX - result.getTy();
    double PTerm = error * kP;

    kIgain += error*deltaTime;
    double Iterm = kIgain*kI;

    double Dterm = 0;
    if (deltaTime > 0) {
      Dterm = ((error - lastError) / deltaTime) * kD;
    }

    if (Math.abs(error) < angleTolerance) {
      power = 0;
      kIgain = 0;
    } else {
      power = Range.clip(PTerm+Iterm+Dterm, -MAX_POWER, MAX_POWER);
    }

    if ((leftPressed && power < 0) || (rightPressed && power > 0)) {
      power = 0;
    }
    if (power == 0.0) {
      spin.setPower(0.001);
    } else {
      spin.setPower(power);
    }
    lastError = error;

    return power;
  }

  public double update(LLResult result) {
    return update(result, false, false);
  }
}
