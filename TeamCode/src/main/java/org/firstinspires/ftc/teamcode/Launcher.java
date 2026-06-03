/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * claw stuff:
 * 0.45 - 1
 */


// Importing things
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.Prism.Color;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.Prism.PrismAnimations;

import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import java.util.concurrent.TimeUnit;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;


// Setup
@TeleOp(name="Launcher", group="Linear Opmode")
public class Launcher extends LinearOpMode {

  // Declare OpMode objects
  private final ElapsedTime runtime = new ElapsedTime();

  private DcMotor frontLeftDrive = null;
  private DcMotor backLeftDrive = null;
  private DcMotor frontRightDrive = null;
  private DcMotor backRightDrive = null;
  private DcMotorEx wheeel = null;
  private DcMotorEx intake = null;
  private Servo pew = null;
  private CRServo helper = null;
  private Servo angle = null;
  private TelemetryPacket packet = null;


  private final int READ_PERIOD = 1;




  /*
  Controls:

   /=\<[B1/T2][B2/T2]>/=\
  /===\______________/===\
  |    ^           [Y]   |
  |  < * >       [X] [B] |
  |    v           [A]   |
  \     _(*)____(*)_     /
   \___/ [LS]  [RS] \___/

LS:
x -> Strafe
y -> Drive Forward
RS:
x -> Turn
A -> FASTER WHEEEL
B -> slower wheeel
Y -> slower drive
  */
  @Override

  public void runOpMode() {


    // Defining motors
    frontLeftDrive = hardwareMap.get(DcMotor.class, "leftFront");
    backLeftDrive = hardwareMap.get(DcMotor.class, "leftBack");
    frontRightDrive = hardwareMap.get(DcMotor.class, "rightFront");
    backRightDrive = hardwareMap.get(DcMotor.class, "rightBack");
    wheeel = hardwareMap.get(DcMotorEx.class, "launcher");
    intake = hardwareMap.get(DcMotorEx.class, "intake");
    pew = hardwareMap.get(Servo.class, "pew");
    helper = hardwareMap.get(CRServo.class, "helper"); angle = hardwareMap.get(Servo.class, "angle");
    packet = new TelemetryPacket(true);
    GoBildaPrismDriver prism = hardwareMap.get(GoBildaPrismDriver.class, "prism");
    DigitalChannel beambreak = hardwareMap.get(DigitalChannel.class, "beambreak");







//        inOutLeft = hardwareMap.get(DcMotor.class, "inOutLeft");
//        inOutRight = hardwareMap.get(DcMotor.class, "inOutRight");
//        teeth = hardwareMap.get(Servo.class, "teeth");
//        spin = hardwareMap.get(Servo.class, "spin");
//        wrist = hardwareMap.get(Servo.class, "wrist");
//        limL = hardwareMap.get(RevTouchSensor.class, "limL");
//        limR = hardwareMap.get(RevTouchSensor.class,"limR");




    // Motor directions
    frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
    backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
    frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
    backRightDrive.setDirection(DcMotor.Direction.REVERSE);


//        inOutLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//        inOutRight.setDirection(DcMotorSimple.Direction.FORWARD);
    wheeel.setDirection(DcMotorEx.Direction.FORWARD);
    intake.setDirection(DcMotorEx.Direction.FORWARD);
    pew.setPosition(0);
    helper.setPower(0);
    angle.setPosition(0);

    // Camera Stuff
    double tagx = 0.0;
    double tagy = 0.0;
    double tagArea = -1.0;
    int tagid = -1;
    prism.setStripLength(36);

      Thread Prism = new Thread(() -> {
        PrismAnimations.Solid solid = new PrismAnimations.Solid();
        PrismAnimations.Snakes snakes = new PrismAnimations.Snakes();
        snakes.setColors();// May be unfinished
        prism.clearAllAnimations();
        solid.setPrimaryColor(0, 0, 0);
        solid.setBrightness(0);
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, solid);
        boolean laststate = false;
        solid.setPrimaryColor(0, 0, 255);
        solid.setBrightness(100);
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, solid);
        sleep(5000);

        while (opModeIsActive()) {
          if (beambreak.getState() != laststate) {
            if (!beambreak.getState()) {
              solid.setPrimaryColor(0, 255, 0);
              solid.setBrightness(100);
            } else {
              solid.setPrimaryColor(0, 0, 0);
              solid.setBrightness(0);
            }
            laststate = beambreak.getState();
            prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, solid);
          }
        }
      });



    waitForStart();
    runtime.reset();
    Prism.start();
//        spin.setPosition(0.5);
//
//        wrist.setPosition(0);

    // Reset encoders

//        inOutLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        inOutRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

//
//        inOutLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        inOutRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    // Run with encoder


    // Declare random variables
    boolean changed = false;
    boolean changed2 = false;
    boolean changed3 = false;
    boolean changed4 = false;
    boolean changed5 = false;
    boolean changed6 = false;
    boolean slow = false;
    boolean shootable = false;

    double drive;
    double strafe;
    double turn;
    double frontLeftPower;
    double frontRightPower;
    double backLeftPower;
    double backRightPower;
    double wheeelSpeed;
    double anglePos;
    double wheeelOffset;
    double dt = 0.0;

    long cycleStart = -1;


    double p = DriveConstants.p;
    double i = DriveConstants.i;
    double d = DriveConstants.d;
    double f = DriveConstants.f;

    wheeelSpeed = 0;
    anglePos = 0.28;
    angle.setPosition(anglePos);

    //Presets for the robot angle and speed based on the position
    // FAR    0.71 ANGLE | 0.83 SPEED
    // MEDIUM 0.28 ANGLE | 0.69 SPEED
    // CLOSE  0.10 ANGLE | 0.60 SPEED


    while (opModeIsActive()) {
      dt=runtime.now(TimeUnit.MILLISECONDS);

      p = DriveConstants.p;
      i = DriveConstants.i;
      d = DriveConstants.d;
      f = DriveConstants.f;

      wheeel.setVelocityPIDFCoefficients(p,i,d,f);




      // Drive variables
      drive = -gamepad1.left_stick_x;
      strafe = gamepad1.left_stick_y;
      turn = gamepad1.right_stick_x;
      wheeelOffset = 0;

      // Slides

      // DPAD_DOWN button -> Fine tuning mode
      if (gamepad1.dpad_down && !changed) {
        slow = !slow;
        changed = true;
      } else if(!gamepad1.dpad_down) changed = false;

      // During slow mode, everything is slowed
      if (slow) {
        drive = 0.0;
        strafe = 0.0;
        turn *= 0.2;
      }

      // left trigger -> Run intake and the helper motor
      // to get the ball into the launcher
      if ((gamepad1.left_trigger >= 0.2) && !changed2) {
        intake.setPower(1);
        if (!(gamepad1.right_trigger >= 0.2)) {helper.setPower(1);}
        changed2 = true;
      } else if (!(gamepad1.left_trigger >= 0.2)) {
        intake.setPower(0);
        if (!(gamepad1.right_trigger >= 0.2)) {helper.setPower(0.001);}
        changed2 = false;
      }



      if (gamepad1.x) {
        anglePos = 0.0;
        wheeelSpeed = 0.0;
      }


      // Far
      if (gamepad1.a && !changed4) {
        anglePos = 0.80;
        wheeelSpeed = 0.70;
        changed4 = true;
      } else if (!gamepad1.a) {
        changed4 = false;
      }

      // Medium
      if (gamepad1.b && !changed5) {
        anglePos = 0.58;
        wheeelSpeed = 0.54;
        changed5 = true;
      } else if (!gamepad1.b) {
        changed5 = false;
      }

      //Close
      if (gamepad1.y && !changed6) {
        anglePos = 0.38;
        wheeelSpeed = 0.53;
        changed6 = true;
      } else if (!gamepad1.y) {
        changed6 = false;
      }

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

      // Right trigger -> push ball into launcher
      // stops the helper servo so balls don't get stuck under
      if (gamepad1.right_trigger >= 0.2 && !changed3) {
        helper.setPower(0.001);
        if (pew.getPosition() == 1) {
          pew.setPosition(0);
        } else {
          pew.setPosition(0.99);
        }
        changed3 = true;
      } else if (!(gamepad1.right_trigger >= 0.2)) {
        pew.setPosition(1);
        changed3 = false;
      }

      /* !!MACHINE GUN MODE!! */
      if (gamepad1.right_bumper) {
        if (cycleStart == -1) {
          cycleStart = runtime.now(TimeUnit.MILLISECONDS);
        }
        double t = runtime.now(TimeUnit.MILLISECONDS) - cycleStart;

        if (t <= 200) {
          pew.setPosition(1);
        } else if (t <= 400) {
          pew.setPosition(0);
        } else if (t <= 1100) {
          intake.setPower(1);
          helper.setPower(1);
        } else {
          intake.setPower(0);
          helper.setPower(0.001);
          pew.setPosition(0);
          cycleStart = -1;
        }

      } else {
        if (cycleStart != -1) {
          helper.setPower(0.001);
          intake.setPower(0);
          pew.setPosition(0);
        }
        cycleStart = -1;
      }



      if (gamepad1.left_bumper) {
        intake.setPower(0);
        helper.setPower(-1);
      }

      /*if (tagid != -1) {
        shootable = -5 <= tagx && tagx <= 5;
      }*/

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

      //wheeel.setPower(wheeelSpeed);
      wheeel.setVelocity(wheeelSpeed*2800);


      angle.setPosition(anglePos);


      dt = runtime.now(TimeUnit.MILLISECONDS) - dt;

      // TELEMETRY
      telemetry.addData("Beambreak", beambreak.getState());
      telemetry.addData("Status", "Run Time: " + runtime);
      telemetry.addData("Motors", "left (%.2f), right (%.2f)", frontLeftPower, frontRightPower, backLeftPower, backRightPower);
      telemetry.addData("FL Encoder", frontLeftDrive.getCurrentPosition());
      telemetry.addData("FR Encoder", frontRightDrive.getCurrentPosition());
      telemetry.addData("BL Encoder", backLeftDrive.getCurrentPosition());
      telemetry.addData("BR Encoder", backRightDrive.getCurrentPosition());
      telemetry.addData("Angle Position", angle.getPosition());
      telemetry.addData("deltaTime", dt/1000);
      //telemetry.addData("PewForward", pewForward);
      //telemetry.addData("PewBack", pewBack);
      //telemetry.addData("IntakeOn", intakeOn);
      //telemetry.addData("PewPos", pew.getPosition());

      //telemetry.addData("Tag in view?", (blocks.length > 0));

      telemetry.addData("Pew", pew.getPosition());


      telemetry.addData("Shooter enabled?", wheeel.isMotorEnabled());
      telemetry.addData("Shooter Power", wheeel.getPower());
      telemetry.addData("Shooter Velocity", wheeel.getVelocity()*60/28);
      telemetry.addData("Shooter Current Use", wheeel.getCurrent(CurrentUnit.AMPS));
      telemetry.addData("Shooter Encoder Reading", wheeel.getCurrentPosition());

      packet.put("Shooter Power", wheeel.getPower());
      packet.put("Shooter Target Velocity", (wheeelSpeed*2800)*60/28);
      packet.put("Shooter Actual Velocity", wheeel.getVelocity()*60/28);
      packet.put("Shooter Encoder Reading", wheeel.getCurrentPosition());


      FtcDashboard dashboard = FtcDashboard.getInstance();
      dashboard.sendTelemetryPacket(packet);


      //telemetry.addData("Wheeel Power", wheeel.getPower());
      //telemetry.addData("Wheeel Encoder", wheeel.getCurrentPosition());




      telemetry.update();
    }
  }

}