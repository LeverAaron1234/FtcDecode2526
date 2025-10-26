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

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;


// Setup
@TeleOp(name="Launcher", group="Linear Opmode")
public class Launcher extends LinearOpMode {

  // Declare OpMode objects
  private final ElapsedTime runtime = new ElapsedTime();

  private DcMotor frontLeftDrive = null;
  private DcMotor backLeftDrive = null;
  private DcMotor frontRightDrive = null;
  private DcMotor backRightDrive = null;
  private DcMotor wheeel = null;
  private HuskyLens camq = null;


  private final int READ_PERIOD = 1;


  //timer
  private final ElapsedTime timer = new ElapsedTime();

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
    frontLeftDrive = hardwareMap.get(DcMotor.class, "motorFL");
    backLeftDrive = hardwareMap.get(DcMotor.class, "motorBL");
    //frontRightDrive = hardwareMap.get(DcMotor.class, "motorFR");
    backRightDrive = hardwareMap.get(DcMotor.class, "motorBR");
    wheeel = hardwareMap.get(DcMotor.class, "wheeel");
    camq = hardwareMap.get(HuskyLens.class, "camq");

//        inOutLeft = hardwareMap.get(DcMotor.class, "inOutLeft");
//        inOutRight = hardwareMap.get(DcMotor.class, "inOutRight");
//        teeth = hardwareMap.get(Servo.class, "teeth");
//        spin = hardwareMap.get(Servo.class, "spin");
//        wrist = hardwareMap.get(Servo.class, "wrist");
//        limL = hardwareMap.get(RevTouchSensor.class, "limL");
//        limR = hardwareMap.get(RevTouchSensor.class,"limR");

    int slow = 1;


    // Motor directions
    frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
    backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
    //frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
    backRightDrive.setDirection(DcMotor.Direction.REVERSE);
//        inOutLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//        inOutRight.setDirection(DcMotorSimple.Direction.FORWARD);
    wheeel.setDirection(DcMotorSimple.Direction.FORWARD);



    /*
     * This sample rate limits the reads solely to allow a user time to observe
     * what is happening on the Driver Station telemetry.  Typical applications
     * would not likely rate limit.
     */

    Deadline rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);

    /*
     * Immediately expire so that the first time through we'll do the read.
     */
    rateLimit.expire();

    /*
     * Basic check to see if the device is alive and communicating.  This is not
     * technically necessary here as the HuskyLens class does this in its
     * doInitialization() method which is called when the device is pulled out of
     * the hardware map.  However, sometimes it's unclear why a device reports as
     * failing on initialization.  In the case of this device, it's because the
     * call to knock() failed.
     */
    if (!camq.knock()) {
      telemetry.addData(">>", "Problem communicating with " + camq.getDeviceName());
    } else {
      telemetry.addData(">>", "Press start to continue");
    }

    camq.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);


    waitForStart();
    runtime.reset();

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

    double drive;
    double strafe;
    double turn;
    double frontLeftPower;
    double frontRightPower;
    double backLeftPower;
    double backRightPower;
    double wheeelSpeed;
    int tagy;
    int tagx;
    int tagw;
    int tagh;
    int tagid;
    wheeelSpeed = 0;



    while (opModeIsActive()) {


      HuskyLens.Block[] blocks = camq.blocks();
      telemetry.addData("Block count", blocks.length);
      for (int i = 0; i < blocks.length; i++) {
        telemetry.addData("Block", blocks[i].toString());
        tagx = blocks[0].x;
        tagy = blocks[0].y;
        tagw = blocks[0].width;
        tagh = blocks[0].height;
        tagid = blocks[0].id;
      }




      // Drive variables
      drive = -gamepad1.left_stick_x;
      strafe = gamepad1.left_stick_y;
      turn = gamepad1.right_stick_x;

      // Slides


      if (gamepad1.y && !changed) {
        if (slow == 3) slow = 1;
        else slow = 3;
        changed = true;
      } else if(!gamepad1.y) changed = false;



      // LAUNCHER speed about 71 - 78% for the back launch zone w/ flywheel
      // w/o flywheel, it takes approx 15% extra power
      if (gamepad1.a) {
        wheeelSpeed += 0.001;
      }
      if (gamepad1.b) {
        wheeelSpeed -= 0.001;
      }



      // Drive equations
      frontLeftPower = Range.clip((drive + strafe - turn) / slow, -1, 1);
      frontRightPower = Range.clip((drive - strafe - turn) / slow, -1, 1);
      backLeftPower = Range.clip((drive - strafe + turn) / slow, -1, 1);
      backRightPower = Range.clip((drive + strafe + turn) / slow, -1, 1);

      frontLeftDrive.setPower(frontLeftPower);
      backLeftDrive.setPower(backLeftPower);
      //frontRightDrive.setPower(frontRightPower);
      backRightDrive.setPower(backRightPower);

      wheeelSpeed = Range.clip(wheeelSpeed,-1,1);

      wheeel.setPower(wheeelSpeed);




      // TELEMETRY
      telemetry.addData("Status", "Run Time: " + runtime);
      telemetry.addData("Motors", "left (%.2f), right (%.2f)", frontLeftPower, frontRightPower, backLeftPower, backRightPower);
      telemetry.addData("FL Encoder", frontLeftDrive.getCurrentPosition());
      //telemetry.addData("FR Encoder", frontRightDrive.getCurrentPosition());
      telemetry.addData("BL Encoder", backLeftDrive.getCurrentPosition());
      telemetry.addData("BR Encoder", backRightDrive.getCurrentPosition());

      telemetry.addData("FL Power", frontLeftPower);
      telemetry.addData("FR Power", frontRightPower);
      telemetry.addData("BL Power", backLeftPower);
      telemetry.addData("BR Power", backRightPower);

      telemetry.addData("Set Power", wheeelSpeed);

      telemetry.addData("Wheeel Power", wheeel.getPower());
      telemetry.addData("Wheeel Encoder", wheeel.getCurrentPosition());




      telemetry.update();
    }
  }

}