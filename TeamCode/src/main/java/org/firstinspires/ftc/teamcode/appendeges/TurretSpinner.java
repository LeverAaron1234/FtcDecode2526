package org.firstinspires.ftc.teamcode.appendeges;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.DriveConstants;

import java.util.ArrayList;
import java.util.List;

public class TurretSpinner {
  private CRServo spin;


  private double kP = DriveConstants.spinP;
  private double kI = DriveConstants.spinI;
  private double kD = DriveConstants.spinD;
  private double kIgain = 0.0;
  private double goalX = 0.0;
  private double lastError = 0.0;
  private double angleTolerance = 0;
  private double deAccellSpeed = 0.001;
  private double lastTx = 0.0;
  private final double MAX_POWER = 0.6;
  private double power = 0;
  private boolean move_left = true;

  private final ElapsedTime timer = new ElapsedTime();

  public void init(HardwareMap hardwareMap) {
    spin = hardwareMap.get(CRServo.class, "spin");
    spin.setDirection(CRServo.Direction.FORWARD);
  }

  public void resetTimer() {
    timer.reset();
  }

  public List<Double> update(LLResult result, boolean leftPressed, boolean rightPressed, boolean lock) {
    kP = DriveConstants.spinP;
    kI = DriveConstants.spinI;
    kD = DriveConstants.spinD;
    double deltaTime = timer.seconds();
    timer.reset();

    // TODO: delete to turn on spinning
    lock = !lock;

    List<Double> returnList = new ArrayList<>();

    if ((leftPressed && power < 0) || (rightPressed && power > 0)) {
      power = 0;
    }

    if (!result.isValid()) {
      if (leftPressed) {move_left = false;}
      if (rightPressed) {move_left = true;}

      if (!lock) {
      spin.setPower(0.3 * ((move_left) ? -1 : 1));
      } else {
        if (lastTx > 0) {
          power += deAccellSpeed;
          if (power > 0) {power = 0;}
        }
        if (lastTx < 0) {
          power -= deAccellSpeed;
          if (power < 0) {power = 0;}
        }
        spin.setPower(power);
      }
      lastError = 0;
      returnList.add(power);
      returnList.add(DriveConstants.spinP);
      returnList.add(DriveConstants.spinI);
      returnList.add(DriveConstants.spinD);
      return returnList;
    }

    double error = goalX - result.getTx();
    lastTx = result.getTx();
    double PTerm = error * DriveConstants.spinP;

    kIgain += error*deltaTime;
    double Iterm = kIgain*DriveConstants.spinI;

    double Dterm = 0;
    if (deltaTime > 0) {
      Dterm = ((error - lastError) / deltaTime) * DriveConstants.spinD;
    }

    if (Math.abs(error) < angleTolerance) {
      power = 0;
      kIgain = 0;
    } else {
      power = Range.clip(PTerm+Iterm+Dterm, -MAX_POWER, MAX_POWER);
    }

    if (power == 0.0) {
      spin.setPower(0.001);
    } else {
      spin.setPower(power);
    }
    lastError = error;

    returnList.add(power);
    returnList.add(DriveConstants.spinP);
    returnList.add(DriveConstants.spinI);
    returnList.add(DriveConstants.spinD);

    return returnList;
  }

  public List<Double> update(LLResult result) {
    return update(result, false, false, false);
  }
}
