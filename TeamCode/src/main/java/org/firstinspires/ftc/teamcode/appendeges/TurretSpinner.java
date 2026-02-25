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
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class TurretSpinner {
  private CRServo spin;


  private double kP = DriveConstants.spinP;
  private double kI = DriveConstants.spinI;
  private double kD = DriveConstants.spinD;
  private double kIgain = 0.0;
  private double goalX = 0.0;
  private double lastError = 0.0;
  private double angleTolerance = 0;
  private double deAccellSpeed = 0.95;
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

  public Map<String, Object> update(LLResult result, boolean leftPressed, boolean rightPressed, boolean lock) {
    kP = DriveConstants.spinP;
    kI = DriveConstants.spinI;
    kD = DriveConstants.spinD;
    double deltaTime = timer.seconds();
    timer.reset();

    // TODO: delete to make the turret spin around when it can't find the target (or click the up arrow on the gamepad)
    lock = !lock;

    Map<String, Object> returnList = new HashMap<>();

    if ((leftPressed && power < 0) || (rightPressed && power > 0)) {
      power = 0;
    }

    if (!result.isValid()) {
      if (leftPressed) {move_left = false;}
      if (rightPressed) {move_left = true;}

      if (!lock) {
      spin.setPower(0.3 * ((move_left) ? -1 : 1));
      } else {
        power = power * deAccellSpeed;
        if (Math.abs(power) < 0.003) {power = 0.0;}
        spin.setPower(power);
      }
      lastError = 0;
      returnList.put("Spin Power",power);
      returnList.put("spin P",DriveConstants.spinP);
      returnList.put("spin I",DriveConstants.spinI);
      returnList.put("spin D",DriveConstants.spinD);
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


    spin.setPower(power);

    lastError = error;

    returnList.put("Spin Power",power);
    returnList.put("spin P",DriveConstants.spinP);
    returnList.put("spin I",DriveConstants.spinI);
    returnList.put("spin D",DriveConstants.spinD);

    return returnList;
  }

  public Map<String, Object> update(LLResult result) {
    return update(result, false, false, false);
  }
}
