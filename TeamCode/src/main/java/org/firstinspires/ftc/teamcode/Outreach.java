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
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.Prism.PrismAnimations;
import org.firstinspires.ftc.teamcode.appendages.Angle;
import org.firstinspires.ftc.teamcode.appendages.Intake;
import org.firstinspires.ftc.teamcode.appendages.Pew;
import org.firstinspires.ftc.teamcode.appendages.Shooter;
import org.firstinspires.ftc.teamcode.appendages.Spin;
import org.firstinspires.ftc.teamcode.appendages.Stopper;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;


// Setup
@TeleOp(name="Outreach", group="Linear Opmode")
public class Outreach extends LinearOpMode {

  private final ElapsedTime runtime = new ElapsedTime();

  @Override
  public void runOpMode() {

    Pose2d beginPose = null;

    boolean startFar = false;
    boolean redTeam = false;
    boolean fieldDrive = true;

    if (startFar) {
      beginPose = new Pose2d(61.95,((redTeam)?-1:1) * -18.62, 0.0);
    } else {
      beginPose = (!redTeam)? new Pose2d(-62.75,-40.25,0.0) : new Pose2d(-55.68,50.88,0.0);
    }


    // I rearranged this,  basically all I did was move the Mechanum instantiation to the beginning and added a sleep.
    // Making sure robot was completely still while pinpoint calibrated.
    MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
    //  Allow for calibration of pinpoint
    //sleep(2000);

    Shooter shooter = new Shooter(hardwareMap);
    Intake intake = new Intake(hardwareMap);
    Pew pew = new Pew(hardwareMap);
    Angle angle = new Angle(hardwareMap);
    Stopper stopper = new Stopper(hardwareMap);

    Limelight3A camq = hardwareMap.get(Limelight3A.class, "limelight");
    Spin spin = new Spin(hardwareMap);
    spin.resetTimer();

    TouchSensor leftLimit = hardwareMap.get(TouchSensor.class, "leftLimit");
    TouchSensor rightLimit = hardwareMap.get(TouchSensor.class, "rightLimit");

    GoBildaPrismDriver prism = hardwareMap.get(GoBildaPrismDriver.class, "prism");
    DigitalChannel beambreak = hardwareMap.get(DigitalChannel.class, "beambreak");

    Actions.runBlocking(pew.set());

    camq.pipelineSwitch(3);// {0: "goal", 1: "obelisk", 2: "RedGoal", 3: "BlueGoal"}
    camq.start();

    FtcDashboard dash = FtcDashboard.getInstance();
    List<Action> runningActions = new ArrayList<>();

    AtomicBoolean turretLock = new AtomicBoolean(false);
    AtomicBoolean team = new AtomicBoolean(redTeam);
    AtomicInteger turretOffset = new AtomicInteger(0);


    // Telemetry output in this thread only.
    Thread turret = new Thread(() -> { // () -> {...} is a lambda expression
      while(opModeIsActive())
      {
        if (Thread.currentThread().isInterrupted() || isStopRequested()) {
          // Update using odometry then add return data to telemetry
          /*if (camq.getLatestResult().isValid()) {
            List vals = spin.camUpdate(camq.getLatestResult(), leftLimit.isPressed(), rightLimit.isPressed(), turretLock.get());
            telemetry.addData("Turret data",
                    "\nspinP (%.2f)" +
                            "\nspinI (%.2f)" +
                            "\nspinD (%.2f)" +
                            "\nspin power (%.2f)",
                    vals.toArray()
            );
            break;

          } else {*/
          List vals = spin.odomUpdate(drive, team.get(), turretOffset.get(), leftLimit.isPressed(), rightLimit.isPressed(), turretLock.get());
          telemetry.addData("Turret data",
                  "\nposX (%.2f)" +
                          "\nposY (%.2f)" +
                          "\ntargetX (%.2f)" +
                          "\ntargetY (%.2f)" +
                          "\nencoder pos (%.2f)" +
                          "\nRobot Heading (%.2f)" +
                          "\nCurrent angle (%.2f)" +
                          "\nTarget angle (%.2f)" +
                          "\nspin power (%.2f)",
                  vals.toArray()
          );
          break;
          //}
        }

        // Update using odometry then add return data to telemetry
        /*if (camq.getLatestResult().isValid()) {
          List vals = spin.camUpdate(camq.getLatestResult(), leftLimit.isPressed(), rightLimit.isPressed(), turretLock.get());
          telemetry.addData("Turret data",
                  "\nspinP (%.2f)" +
                          "\nspinI (%.2f)" +
                          "\nspinD (%.2f)" +
                          "\nspin power (%.2f)",
                  vals.toArray()
          );
        } else {*/
        List vals = spin.odomUpdate(drive, team.get(), turretOffset.get(), leftLimit.isPressed(), rightLimit.isPressed(), turretLock.get());
        telemetry.addData("Turret data",
                "\nposX (%.2f)" +
                        "\nposY (%.2f)" +
                        "\ntargetX (%.2f)" +
                        "\ntargetY (%.2f)" +
                        "\nencoder pos (%.2f)" +
                        "\nRobot Heading (%.2f)" +
                        "\nCurrent angle (%.2f)" +
                        "\nTarget angle (%.2f)" +
                        "\nspin power (%.2f)",
                vals.toArray()
        );
        //}
        telemetry.update();
      }
    });


    Thread Prism = new Thread(() -> {
      prism.enableDefaultBootArtboard(false); // Disable flashy boot animation lights (to conform with rules)
      PrismAnimations.Solid solid = new PrismAnimations.Solid();
      TelemetryPacket packet = new TelemetryPacket();
      prism.clearAllAnimations();
      solid.setPrimaryColor(0, 0, 0);
      solid.setBrightness(0);
      prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, solid);
      boolean laststate = false;
      while (opModeIsActive()) {
        if (isStopRequested()) {break;}
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
        packet.put("Beam value", beambreak.getState());
        dash.sendTelemetryPacket(packet);
      }
      prism.clearAllAnimations();
    });



    Actions.runBlocking(new ParallelAction(
            shooter.stop(),
            pew.set(),
            intake.off(),
            angle.close(),
            stopper.Out()
    ));


    telemetry.update();


    /*=======================================WAIT FOR START=======================================*/
    waitForStart();

    turret.start(); // start the turret thread
    Prism.start();

    runtime.reset();
    spin.resetTimer();


    //  ----------Define Variables----------

    double drivevar;
    double strafe;
    double turn;

    double wheeelSpeed;
    double anglePos;
    wheeelSpeed = 0;
    anglePos = 0.0;


    boolean changed = false;
    boolean changed2 = false;
    boolean changed3 = false;
    boolean changed4 = false;
    boolean changed5 = false;
    boolean changed6 = false;
    boolean changed7 = false;
    boolean changed8 = false;
    boolean changed9 = false;
    boolean changed10 = false;

    boolean slow = false;

    PoseVelocity2d movement = new PoseVelocity2d(new Vector2d(0,0),0);

    /*===================================WHILE OPMODE IS RUNNING==================================*/
    while (opModeIsActive()) {


      TelemetryPacket packet = new TelemetryPacket();

      /// Movement
      drivevar = gamepad1.left_stick_x * -1;
      strafe = gamepad1.left_stick_y * 1;
      turn = -gamepad1.right_stick_x;

      /// Intake
      if (gamepad1.x && !changed2 && !changed8) {
        runningActions.add(intake.on());
        runningActions.add(pew.launch());
        runningActions.add(stopper.Out());
        changed2 = true;
      } else if (!gamepad1.x && changed2) {
        runningActions.add(pew.set());
        runningActions.add(intake.off());
        changed2 = false;
      }

      // Run artifacts backwards
      if ((gamepad1.dpad_down && gamepad1.x) && !changed8 && !changed2) {
        runningActions.add(pew.back());
        runningActions.add(intake.back());
        changed8 = true;
      } else if (!(gamepad1.dpad_down && gamepad1.x) && changed8) {
        runningActions.add(intake.off());
        runningActions.add(pew.set());
        changed8 = false;
      }

      /// Firing
      if (gamepad1.b && !changed6) {
        runningActions.add(pew.launch());
        runningActions.add(intake.on());
        runningActions.add(stopper.In());
        changed6 = true;
      } else if (!gamepad1.b && changed6) {
        runningActions.add(pew.set());
        runningActions.add(intake.off());
        runningActions.add(stopper.Out());
        changed6 = false;
      }

      /// Flywheel
      if (gamepad2.right_trigger >= 0.2) {
        wheeelSpeed = Math.max(wheeelSpeed + 0.01, 1);
      }
      if (gamepad2.left_trigger >= 0.2) {
        wheeelSpeed = Math.min(wheeelSpeed - 0.01, 0);
      }

      /// Angle / Hood
      if (gamepad2.right_bumper) {
        anglePos = Math.max(anglePos + 0.01, 1);
      }

      if (gamepad2.left_bumper) {
        anglePos = Math.min(anglePos - 0.01, 0);
      }


      //////////////////////////////////
      slow = false;
      // During slow mode, everything is slowed
      if (slow) {
        drivevar *= 0.2;
        strafe *= 0.2;
        turn *= 0.2;
      }


      // Stop the launcher and lower the hood
      if (gamepad1.y) {
        anglePos = 0.0;
        wheeelSpeed = 0.0;
      }

      if (gamepad2.dpad_left) {
        turretOffset.set(turretOffset.get()-1);
      }

      if (gamepad2.dpad_right) {
        turretOffset.set(turretOffset.get()+1);
      }


      if (gamepad1.right_stick_button && !changed9) {
        spin.setInitialized(false);
        changed9 = true;
      } else if (!gamepad1.right_stick_button && changed9) {
        changed9 = false;
      }

      /////////////////////////////////////////////////////////

      double heading = drive.localizer.getPose().heading.toDouble();
      double newdrivevar = strafe * Math.cos(heading) - drivevar * Math.sin(heading);
      double newstrafe = strafe * Math.sin(heading) + drivevar * Math.cos(heading);

      movement = new PoseVelocity2d(
              new Vector2d(
                      (fieldDrive)? newstrafe : strafe,
                      (fieldDrive) ? newdrivevar : drivevar
              ),
              turn
      );

      anglePos = Range.clip(anglePos,0.0,1.0);
      runningActions.add(angle.varangle(anglePos));
      runningActions.add(shooter.varshooter(wheeelSpeed));

      if ((DriveConstants.p != shooter.getPID().p) || (DriveConstants.i != shooter.getPID().i) || (DriveConstants.d != shooter.getPID().d)) {
        shooter.resetPID(DriveConstants.p,DriveConstants.i,DriveConstants.d);
      }

      drive.setDrivePowers(movement);
      drive.updatePoseEstimate();

      // update running actions
      List<Action> newActions = new ArrayList<>();
      for (Action action : runningActions) {
        action.preview(packet.fieldOverlay());
        if (action.run(packet)) {
          newActions.add(action);
        }
      }
      runningActions = newActions;

      dash.sendTelemetryPacket(packet);
    }

    drive.localizer.update();


    telemetry.update();


    if (isStopRequested()) {
      Prism.interrupt();
      turret.interrupt();
    }
  }

}