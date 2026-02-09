package org.firstinspires.ftc.teamcode;



import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.concurrent.TimeUnit;

@TeleOp(name="Turret", group="Linear Opmode")
public class Turret extends LinearOpMode {

  private final ElapsedTime runtime = new ElapsedTime();

  private DcMotor frontLeftDrive = null;
  private DcMotor backLeftDrive = null;
  private DcMotor frontRightDrive = null;
  private DcMotor backRightDrive = null;

  private DcMotorEx wheeel = null;
  private DcMotorEx intake = null;
  private Limelight3A camq = null;
  private DcMotorEx pew = null;
  private Servo angle = null;
  private Servo spin = null;


  private TelemetryPacket packet = null;


  @Override
  public void runOpMode() {
    // Defining motors
    frontLeftDrive = hardwareMap.get(DcMotor.class, "leftFront");
    backLeftDrive = hardwareMap.get(DcMotor.class, "leftBack");
    frontRightDrive = hardwareMap.get(DcMotor.class, "rightFront");
    backRightDrive = hardwareMap.get(DcMotor.class, "rightBack");

    wheeel = hardwareMap.get(DcMotorEx.class, "launcher");
    intake = hardwareMap.get(DcMotorEx.class, "intake");
    pew = hardwareMap.get(DcMotorEx.class, "pew");

    camq = hardwareMap.get(Limelight3A.class, "limelight");

    angle = hardwareMap.get(Servo.class, "angle");
    spin = hardwareMap.get(Servo.class, "spin");


    packet = new TelemetryPacket(true);

    // Motor directions
    frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
    backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
    frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
    backRightDrive.setDirection(DcMotor.Direction.REVERSE);
    // Set brake on stop
    frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    pew.setDirection(DcMotorSimple.Direction.FORWARD);
    wheeel.setDirection(DcMotorEx.Direction.FORWARD);
    intake.setDirection(DcMotorEx.Direction.FORWARD);


    // Servo setup
    spin.setPosition(0); // 0-720
    angle.setPosition(0);

    // Camera Stuff
    camq.pipelineSwitch(1); // {0: "goal", 1: "obelisk"}
    camq.start();
    double tagx = 0.0;
    double tagy = 0.0;
    double tagArea = -1.0;
    int tagid = -1;
    LLResult result = camq.getLatestResult();

    /*=======================================WAIT FOR START=======================================*/
    waitForStart();
    runtime.reset();


    //  ----------Define Variables----------

    // Mechanum movement
    double drive;
    double strafe;
    double turn;

    // Power values
    double frontLeftPower;
    double frontRightPower;
    double backLeftPower;
    double backRightPower;

    double wheeelSpeed;
    double anglePos;

    double p = DriveConstants.p;
    double i = DriveConstants.i;
    double d = DriveConstants.d;
    double f = DriveConstants.f;

    double dt = 0.0;


    // Speeds & Angles:
    // Far:    S: 0.70 A: 0.80
    // Medium: S: 0.54 A: 0.58
    // Close:  S: 0.53 A: 0.38
    wheeelSpeed = 0;
    anglePos = 0.10;
    angle.setPosition(anglePos);

    /*===================================WHILE OPMODE IS RUNNING==================================*/
    while (opModeIsActive()) {
      dt = runtime.now(TimeUnit.SECONDS);

      p = DriveConstants.p;
      i = DriveConstants.i;
      d = DriveConstants.d;
      f = DriveConstants.f;

      wheeel.setVelocityPIDFCoefficients(p,i,d,f);

      result = camq.getLatestResult();

      if (result.isValid()) {
        tagx = result.getTx();
        tagy = result.getTy();
        tagArea = result.getTa();
        tagid = result.getFiducialResults().get(0).getFiducialId();
      } else {
        tagx = 0.0;
        tagy = 0.0;
        tagArea = -1.0;
        tagid = -1;
      }

      // Drive variables
      drive = -gamepad1.left_stick_x;
      strafe = gamepad1.left_stick_y;
      turn = gamepad1.right_stick_x;








      // Drive equations
      frontLeftPower = Range.clip((drive + strafe - turn), -1, 1);
      frontRightPower = Range.clip((drive - strafe - turn), -1, 1);
      backLeftPower = Range.clip((drive - strafe + turn), -1, 1);
      backRightPower = Range.clip((drive + strafe + turn), -1, 1);



      frontLeftDrive.setPower(frontLeftPower);
      backLeftDrive.setPower(backLeftPower);
      frontRightDrive.setPower(frontRightPower);
      backRightDrive.setPower(backRightPower);

      wheeelSpeed = Range.clip(wheeelSpeed,-1,1);
      anglePos = Range.clip(anglePos, 0.20,1);

      wheeel.setVelocity(wheeelSpeed*2800);
      // Set to tick ct by *2800
      // Set to RPM by *6000

      angle.setPosition(anglePos);


      dt = runtime.now(TimeUnit.MILLISECONDS) - dt;


      telemetry.addLine("==Status==");
      telemetry.addData("Runtime", runtime.seconds());
      telemetry.addData("DeltaTime", dt/1000);
      telemetry.addLine("==Drive==");
      telemetry.addData("Target", "left (%.2f), right (%.2f)", frontLeftPower, frontRightPower, backLeftPower, backRightPower);
      telemetry.addData("Actual", "left (%.2f), right (%.2f)", frontLeftDrive.getCurrentPosition(), frontRightDrive.getCurrentPosition(), backLeftDrive.getCurrentPosition(), backLeftDrive.getCurrentPosition());
      telemetry.addLine("==Launcher==");
      telemetry.addData("Speed", "target (%.2f), actual (%.2f)", wheeelSpeed*6000, wheeel.getVelocity()*60/28);
      telemetry.addLine((result.isValid()) ? "Tag Detected" : "No Tag Detected");
      if (result.isValid()) {
        telemetry.addData("Tag Pos","tagx: (%.2f), tagy: (%.2f)", tagx, tagy);
        telemetry.addData("Tag Info", "tagArea (%.2f), tagID (%.2f)", tagArea, tagid);
      }
      telemetry.addLine("==Other==");
      telemetry.addData("Pew Pwr", pew.getPower());
      telemetry.addData("Angle Pos", angle.getPosition());

    }

  }

}
