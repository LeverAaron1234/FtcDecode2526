package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.appendeges.TurretSpinner;

import java.util.concurrent.TimeUnit;

@TeleOp(name="BlueTurret", group="Linear Opmode")
public class BlueTurret extends LinearOpMode {

  private final ElapsedTime runtime = new ElapsedTime();

  private DcMotor frontLeftDrive = null;
  private DcMotor backLeftDrive = null;
  private DcMotor frontRightDrive = null;
  private DcMotor backRightDrive = null;

  private DcMotorEx wheeel = null;
  private DcMotor intake = null;
  private Limelight3A camq = null;
  private DcMotorEx pew = null;
  private Servo angle = null;
  private Servo stopper = null;
  private TurretSpinner spin = new TurretSpinner();

  public TouchSensor leftLimit = null;
  public TouchSensor rightLimit = null;

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
    pew = hardwareMap.get(DcMotorEx.class, "pew");

    camq = hardwareMap.get(Limelight3A.class, "limelight");

    angle = hardwareMap.get(Servo.class, "angle");
    stopper = hardwareMap.get(Servo.class, "stopper");


    spin.init(hardwareMap);

    leftLimit = hardwareMap.get(TouchSensor.class, "leftLimit");
    rightLimit = hardwareMap.get(TouchSensor.class, "rightLimit");

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


    // Servo setup
    angle.setPosition(0);
    stopper.setPosition(0); // equivalent to pew

    // Camera Stuff
    int pipe = 3;
    camq.pipelineSwitch(pipe); // {0: "goal", 1: "obelisk", 2: "RedGoal", 3: "BlueGoal"} // TODO: For comp, set to goal
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
    anglePos = 0.0;
    spinPwr = 0.0;

    boolean changed = false;
    boolean changed2 = false;
    boolean changed3 = false;
    boolean changed4 = false;
    boolean changed5 = false;
    boolean changed6 = false;
    boolean changed7 = false;
    boolean changed8 = false;

    boolean slow = false;
    boolean turretLock = false;

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
        drive *= 0.0;
        strafe *= 0.0;
        turn *= 0.2;
      }




      // left trigger -> Run intake and the helper motor
      // to get the ball into the launcher
      if ((gamepad1.left_trigger >= 0.2) && !changed2) {
        intake.setPower(1);
        pew.setPower(1);
        changed2 = true;
      } else if (!(gamepad1.left_trigger >= 0.2) && changed2) {
        pew.setPower(0);
        intake.setPower(0);
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
        anglePos = 0.63;
        wheeelSpeed = 0.71;
        changed3 = true;
      } else if (!gamepad1.a) {
        changed3 = false;
      }

      // Medium
      if (gamepad1.b && !changed4) {
        anglePos = 0.0;
        wheeelSpeed = 0.52;
        changed4 = true;
      } else if (!gamepad1.b) {
        changed4 = false;
      }

      //Close
      if (gamepad1.y && !changed5) {
        anglePos = 0.0;
        wheeelSpeed = 0.46;
        changed5 = true;
      } else if (!gamepad1.y) {
        changed5 = false;
      }


      // Right trigger -> push ball into launcher
      // stops the intake servo so balls don't get stuck under
      if (gamepad1.right_trigger >= 0.2 && !changed6) {
        pew.setPower(1);
        intake.setPower(1);
        stopper.setPosition(0.5);
        changed6 = true;
      } else if (!(gamepad1.right_trigger >= 0.2) && changed6) {
        pew.setPower(0);
        intake.setPower(0);
        stopper.setPosition(0);
        changed6 = false;
      }

      // Run artifacts backwards
      if (gamepad1.left_bumper && !changed8) {
        pew.setPower(-0.4);
        intake.setPower(-0.4);
        changed8 = true;
      } else if (!(gamepad1.left_bumper) && changed8) {
        pew.setPower(0);
        intake.setPower(0);
        changed8 = false;
      }


      if (gamepad1.dpad_up && ! changed7) {
        turretLock = !turretLock;
        changed7 = true;
      } else if (!gamepad1.dpad_up) {
        changed7 = false;
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

      wheeelSpeed = Range.clip(wheeelSpeed,0,1);
      anglePos = Range.clip(anglePos, 0,0.9);

      wheeel.setVelocity(wheeelSpeed*2800);
      // wheel speed is in percent of 1
      // Get tick ct by *2800
      // Get RPM by *6000
      // Convert tick ct to rpm by * 60/28

      packet.putAll(spin.update(result, leftLimit.isPressed(), rightLimit.isPressed(), turretLock));
      angle.setPosition(anglePos);

      dt = runtime.now(TimeUnit.MILLISECONDS) - dt;



      telemetry.addLine("==Status==");
      telemetry.addData("Runtime", runtime.seconds());
      telemetry.addData("DeltaTime", dt/1000);

      telemetry.addLine("==Drive==");
      telemetry.addData("Target", "left (%.2f) (%.2f), right (%.2f) (%.2f)", frontLeftPower, frontRightPower, backLeftPower, backRightPower);
      telemetry.addData("Actual", "left (%d) (%d), right (%d) (%d)", frontLeftDrive.getCurrentPosition(), frontRightDrive.getCurrentPosition(), backLeftDrive.getCurrentPosition(), backLeftDrive.getCurrentPosition());

      telemetry.addLine("==Launcher==");
      telemetry.addData("Speed", "target (%.2f), actual (%.2f)", wheeelSpeed*6000, wheeel.getVelocity()*60/28);
      telemetry.addLine((result.isValid()) ? "Tag Detected" : "No Tag Detected");
      if (result.isValid()) {
        telemetry.addData("Tag Pos","tagx: (%.2f), tagy: (%.2f)", tagx, tagy);
        telemetry.addData("Tag Info", "tagArea (%.2f), tagID (%d)", tagArea, tagid);
      }

      telemetry.addLine("==Other==");
      telemetry.addData("Pew Pwr", pew.getPower());
      telemetry.addData("Angle Pos", angle.getPosition());


      packet.addTimestamp();
      packet.put("Shooter Target Velocity", wheeelSpeed*6000);
      packet.put("Shooter Actual Velocity", wheeel.getVelocity()*60/28);
      packet.put("Camera result", (result.isValid()) ? "Has result" : "No result");
      packet.put("TagX", tagx);
      packet.put("TagY", tagy);
      packet.put("TagArea", tagArea);
      packet.put("TagID", tagid);
      packet.put("Camera is connected", camq.isConnected());
      packet.put("Camera is running", camq.isRunning());



      FtcDashboard dashboard = FtcDashboard.getInstance();
      dashboard.sendTelemetryPacket(packet);



      telemetry.update();
    }

    camq.close();

  }

}
