package org.firstinspires.ftc.teamcode.SwerveDrive;

import android.util.Pair;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class SwerveDrive {

  // Setup

  /// Motors
  private DcMotor leftFrontMotor = null;
  private DcMotor leftBackMotor = null;
  private DcMotor rightFrontMotor = null;
  private DcMotor rightBackMotor = null;

  /// Servos
  private Servo leftFrontServo = null;
  private Servo leftBackServo = null;
  private Servo rightFrontServo = null;
  private Servo rightBackServo = null;

  /// Servo Encoders
  private AnalogInput leftFrontServoEncoder = null;
  private AnalogInput leftBackServoEncoder = null;
  private AnalogInput rightFrontServoEncoder = null;
  private AnalogInput rightBackServoEncoder = null;

  /// Starting Position
  private Pose2d beginPose;


  // initialization
  public SwerveDrive(@NonNull HardwareMap hardwareMap, Pose2d beginPose) {

    // TODO: Change names to the configuration names in the driver hub
    this.leftFrontMotor = hardwareMap.get(DcMotor.class, "LF_M");
    this.leftBackMotor = hardwareMap.get(DcMotor.class, "LB_M");
    this.rightFrontMotor = hardwareMap.get(DcMotor.class, "RF_M");
    this.rightBackMotor = hardwareMap.get(DcMotor.class, "RB_M");

    // TODO: Change these names to match as well
    this.leftFrontServo = hardwareMap.get(Servo.class, "LF_S");
    this.leftFrontServoEncoder = hardwareMap.get(AnalogInput.class, "LF_S-encoder");
    this.leftBackServo = hardwareMap.get(Servo.class, "LB_S");
    this.leftBackServoEncoder = hardwareMap.get(AnalogInput.class, "LB_S-encoder");
    this.rightFrontServo = hardwareMap.get(Servo.class, "RF_S");
    this.rightFrontServoEncoder = hardwareMap.get(AnalogInput.class, "RF_S-encoder");
    this.rightBackServo = hardwareMap.get(Servo.class, "RB_S");
    this.rightBackServoEncoder = hardwareMap.get(AnalogInput.class, "RB_S-encoder");

    if (beginPose != null) {
      this.beginPose = beginPose;
    } else {
      this.beginPose = new Pose2d(0,0,0);
    }
  }

  public Pair<Double[], Double[]> targetMovement(Vector2d translation, double rotation) {
    Double[] targetPower = new Double[4];
    Double[] targetAngle = new Double[4];
    // RB -> LB -> FL -> FR

    /// Target Positions

    // Clamp too large translational vector
    if (magnitude(translation) > 1) {
      translation = normalize(translation);
    }

    for (int i = 0; i < 4; i++) {

      // perpendicular vectors
      Vector2d perp = new Vector2d(rotation * Math.sin(Math.PI/2 * i + Math.PI/4),rotation * Math.cos(Math.PI/2 * i + Math.PI/4));

      // Add the vectors
      Vector2d finalVector = translation.plus(perp);
      if (magnitude(finalVector) > 1) {
        finalVector = normalize(finalVector);
      }

      targetPower[i] = magnitude(finalVector);
      targetAngle[i] = finalVector.angleCast().toDouble();
    }

    return new Pair<>(targetPower,targetAngle);
  }



  public void changeOrigin(Pose2d newBeginPos) {
    this.beginPose = newBeginPos;
  }

  double magnitude(@NonNull Vector2d v) {
    return Math.hypot(v.x,v.y);
  }

  Vector2d normalize(@NonNull Vector2d v) {
    return v.div(magnitude(v));
  }

  public void move(Vector2d translation, double rotation) {
    /// target.first -> targetPower[]
    /// target.second -> targetAngle[]
    Pair<Double[],Double[]> target = targetMovement(translation,rotation);
    // wheel order
    // RightBack -> LeftBack -> LeftFront -> RightFront
    Servo[] servos = {this.rightBackServo,this.leftBackServo,this.leftFrontServo,this.rightFrontServo};
    AnalogInput[] servoEncoders = {this.rightBackServoEncoder,this.leftBackServoEncoder,this.leftFrontServoEncoder,this.rightFrontServoEncoder};
    DcMotor[] motors = {this.rightBackMotor,this.leftBackMotor,this.leftFrontMotor,this.rightFrontMotor};

    boolean[] reverseWheels = {false,false,false,false};

    for (int i = 0; i < 4; i++) {
      Servo currentServo = servos[i];

      double currentTargetAngle = target.second[i]; // Target (-pi - pi)
      double secondaryTargetAngle = (currentTargetAngle + Math.PI) % (2*Math.PI) - Math.PI;
      double currentServoAngle = getServoPosition(servoEncoders[i]); // Current (0.0 - 1.0)
      currentServoAngle = currentServoAngle * (2*Math.PI) - Math.PI; // Current (but now ranged -pi - pi)

      // If the wheels are able to spin all the way around, then we only need to move the wheel 90 degrees max
      // and reverse the wheel power
      if (Math.abs(currentServoAngle - currentTargetAngle) <= Math.abs(currentServoAngle - secondaryTargetAngle)) {
        reverseWheels[i] = false;
      } else {
        reverseWheels[i] = true;
      }

      if (reverseWheels[i]) {
        currentServo.setPosition(secondaryTargetAngle);
      } else {
        currentServo.setPosition(currentTargetAngle);
      }

      DcMotor currentMotor = motors[i];
      if (reverseWheels[i]) {
        currentMotor.setPower(-target.first[i]);
      } else {
        currentMotor.setPower(target.first[i]);
      }

    }

  }

  public double getServoPosition(@NonNull AnalogInput servoEncoder) {
    return servoEncoder.getVoltage() / servoEncoder.getMaxVoltage();
  }





}

