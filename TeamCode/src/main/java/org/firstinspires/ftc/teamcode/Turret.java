package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
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

import org.firstinspires.ftc.teamcode.appendeges.TurretSpinner;

import java.util.concurrent.TimeUnit;

@TeleOp(name="Turret", group="Linear Opmode")
public class Turret extends LinearOpMode {

  private final ElapsedTime runtime = new ElapsedTime();

  private DcMotor frontLeftDrive = null;
  private DcMotor backLeftDrive = null;
  private DcMotor frontRightDrive = null;
  private DcMotor backRightDrive = null;

  private DcMotorEx wheeel = null;
  private DcMotor intake = null;
  private DcMotor intake2 = null;
  private Limelight3A camq = null;
  private DcMotorEx pew = null;
  private Servo angle = null;
  private TurretSpinner spin = new TurretSpinner();


  private TelemetryPacket packet = null;


  @Override
  public void runOpMode() {
    // Defining motors
    frontLeftDrive = hardwareMap.get(DcMotor.class, "leftFront");
    backLeftDrive = hardwareMap.get(DcMotor.class, "leftBack");
    frontRightDrive = hardwareMap.get(DcMotor.class, "rightFront");
    backRightDrive = hardwareMap.get(DcMotor.class, "rightBack");

    wheeel = hardwareMap.get(DcMotorEx.class, "launcher");
    intake = hardwareMap.get(DcMotor.class, "intake");
    intake2 = hardwareMap.get(DcMotor.class, "intake2");
    pew = hardwareMap.get(DcMotorEx.class, "pew");

    camq = hardwareMap.get(Limelight3A.class, "limelight");

    angle = hardwareMap.get(Servo.class, "angle");

    spin.init(hardwareMap);


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
    wheeel.setDirection(DcMotorEx.Direction.REVERSE);
    intake.setDirection(DcMotor.Direction.FORWARD);  // Main Intake
    intake2.setDirection(DcMotor.Direction.FORWARD); // Helping Intake


    // Servo setup
    angle.setPosition(0);

    // Camera Stuff
    camq.pipelineSwitch(0); // {0: "goal", 1: "obelisk"}
    camq.start();
    double tagx = 0.0;
    double tagy = 0.0;
    double tagArea = -1.0;
    int tagid = -1;
    LLResult result = camq.getLatestResult();

    /*=======================================WAIT FOR START=======================================*/
    waitForStart();
    runtime.reset();
    spin.resetTimer();


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
    double spinPwr;
    wheeelSpeed = 0;
    anglePos = 0.10;
    spinPwr = 0.0;

    boolean changed = false;
    boolean changed2 = false;
    boolean changed3 = false;
    boolean changed4 = false;
    boolean changed5 = false;
    boolean changed6 = false;
    boolean slow = false;

    double p = DriveConstants.p;
    double i = DriveConstants.i;
    double d = DriveConstants.d;
    double f = DriveConstants.f;

    double dt = 0.0;


    // Speeds & Angles:
    // Far:    S: 0.70 A: 0.80
    // Medium: S: 0.54 A: 0.58
    // Close:  S: 0.53 A: 0.38
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



      // DPAD_DOWN button -> Fine tuning mode
      if ((gamepad1.dpad_down) && !changed) {
        slow = !slow;
        changed = true;
      } else if(!gamepad1.dpad_down) changed = false;

      // During slow mode, everything is slowed
      if (slow) {
        drive = 0.0;
        strafe = 0.0;
        turn *= 0.0;
      }




      // left trigger -> Run intake and the helper motor
      // to get the ball into the launcher
      if ((gamepad1.left_trigger >= 0.2) && !changed2) {
        intake.setPower(1);
        intake2.setPower(1);
        changed2 = true;
      } else if (!(gamepad1.left_trigger >= 0.2)) {
        intake.setPower(0);
        intake2.setPower(0);
        changed2 = false;
      }

      // Stop the launcher and lower the hood
      if (gamepad1.x) {
        anglePos = 0.0;
        wheeelSpeed = 0.0;
      }

      // Debug angle / wheel speed
      if (gamepad1.dpad_left) {
        if (slow) {
          wheeelSpeed -= 0.01;
        } else {
          anglePos -= 0.01;
        }
      }

      if (gamepad1.dpad_right) {
        if (slow) {
          wheeelSpeed += 0.01;
        } else {
          anglePos += 0.01;
        }
      }

      // Angles and Speeds
      // Far
      if (gamepad1.a && !changed3) {
        anglePos = 0.80;
        wheeelSpeed = 0.70;
        changed3 = true;
      } else if (!gamepad1.a) {
        changed3 = false;
      }

      // Medium
      if (gamepad1.b && !changed4) {
        anglePos = 0.58;
        wheeelSpeed = 0.54;
        changed4 = true;
      } else if (!gamepad1.b) {
        changed4 = false;
      }

      //Close
      if (gamepad1.y && !changed5) {
        anglePos = 0.38;
        wheeelSpeed = 0.53;
        changed5 = true;
      } else if (!gamepad1.y) {
        changed5 = false;
      }


      // Right trigger -> push ball into launcher
      // stops the intake servo so balls don't get stuck under
      if (gamepad1.right_trigger >= 0.2 && !changed6) {
        pew.setPower(1);
        changed6 = true;
      } else if (!(gamepad1.right_trigger >= 0.2)) {
        pew.setPower(0);
        changed6 = false;
      }

      // Run artifacts backwards
      if (gamepad1.left_bumper) {
        pew.setPower(-1);
        intake.setPower(-1);
        intake2.setPower(-1);
      }



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

      packet.put("SpinPwr",spin.update(result));
      angle.setPosition(anglePos);


      dt = runtime.now(TimeUnit.MILLISECONDS) - dt;



      /*telemetry.addLine("==Status==");
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
      telemetry.addData("Angle Pos", angle.getPosition());*/


      packet.addTimestamp();
      packet.put("Shooter Target Velocity", wheeelSpeed*6000);
      packet.put("Shooter Actual Velocity", wheeel.getVelocity()*60/28);
      packet.put("Camera result", (result.isValid()) ? "Has result" : "No result");
      packet.put("TagX", tagx);
      packet.put("TagY", tagy);
      packet.put("TagArea", tagArea);
      packet.put("TagID", tagid);


      FtcDashboard dashboard = FtcDashboard.getInstance();
      dashboard.sendTelemetryPacket(packet);



      telemetry.update();
    }

  }

}
