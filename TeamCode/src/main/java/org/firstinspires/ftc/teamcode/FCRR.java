package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.appendages.Angle;
import org.firstinspires.ftc.teamcode.appendages.Intake;
import org.firstinspires.ftc.teamcode.appendages.Pew;
import org.firstinspires.ftc.teamcode.appendages.Shooter;
import org.firstinspires.ftc.teamcode.appendages.Spin;
import org.firstinspires.ftc.teamcode.appendages.Stopper;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;

@TeleOp(group="Linear Opmode")
public class FCRR extends LinearOpMode {

  private final ElapsedTime runtime = new ElapsedTime();

  @Override
  public void runOpMode() {

    Pose2d beginPose = null;
    boolean beginPoseValid = false;

    int selector = 0;
    boolean startFar = true;
    boolean redTeam = false;
    if (!RobotPose.updated) {
    while (opModeInInit()) {
      if (isStopRequested()) {break;}
      if (gamepad2.dpadLeftWasPressed()) {
        selector = (selector-1) % 2;
      } else if (gamepad2.dpadRightWasPressed()) {
        selector = (selector+1) % 2;
      }
      if (gamepad2.a) {
        switch (selector) {
          case 0:
            startFar = !startFar;
            break;
          case 1:
            redTeam = !redTeam;
            break;
          default:
            telemetry.addLine("Selector Error: selector out of bounds: " + selector);
        }
      }

      switch (selector) {
        case 0:
          telemetry.addLine("Starting Pos");
          if (startFar) {
            telemetry.addLine("Far");
          } else {
            telemetry.addLine("Close");
          }
          break;

        case 1:
          telemetry.addLine("Team");
          if (redTeam) {
            telemetry.addLine("Red");
          } else {
            telemetry.addLine("Blue");
          }
          break;

        default:
          telemetry.addLine("Error");

      }

      telemetry.update();
      if (gamepad2.yWasPressed()) {break;}
    }
    } else {
      startFar = RobotPose.startFar;
      redTeam = RobotPose.redTeam;
    }
    if (isStopRequested()) {
      return;
    }

    telemetry.addLine("Ready");
    telemetry.addLine("Starting Pos: " + ((startFar)? "Far" : "Close"));
    telemetry.addLine("Team: " + ((redTeam)? "Red" : "Blue"));
    telemetry.update();

    if (RobotPose.updated) {
      beginPose = RobotPose.lastRobotPose;
      beginPoseValid = true;
    } else {
      if (startFar) {
        beginPose = new Pose2d(61.95,((redTeam)?-1:1) * -18.62, 0.0);
      } else {
        beginPose = (!redTeam)? new Pose2d(-62.75,-40.25,0.0): new Pose2d(-55.68,50.88,0.0);
      }
    }


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

    Actions.runBlocking(pew.set());


    camq.pipelineSwitch(3);// {0: "goal", 1: "obelisk", 2: "RedGoal", 3: "BlueGoal"}
    camq.start();

    MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

    FtcDashboard dash = FtcDashboard.getInstance();
    List<Action> runningActions = new ArrayList<>();

    AtomicBoolean turretLock = new AtomicBoolean(false);
    AtomicBoolean team = new AtomicBoolean(redTeam);
    AtomicInteger turretOffset = new AtomicInteger(0);

    // Autonomous threading so that the camera can control the turret in a loop
    Thread thread = new Thread(() -> { // () -> {...} is a lambda expression
      while(opModeIsActive())
      {
        if (Thread.currentThread().isInterrupted()) {
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
                            "\nCurrent angle (%.2f)" +
                            "\nRobot Heading (%.2f)" +
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
                          "\nCurrent angle (%.2f)" +
                          "\nRobot Heading (%.2f)" +
                          "\nTarget angle (%.2f)" +
                          "\nspin power (%.2f)",
                  vals.toArray()
          );
        //}
        telemetry.update();
      }
    });


    Actions.runBlocking(new ParallelAction(
            shooter.stop(),
            pew.set(),
            intake.off(),
            angle.close()
    ));


    telemetry.update();


    /*=======================================WAIT FOR START=======================================*/
    waitForStart();

    thread.start(); // start the above defined thread

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

      drivevar = -gamepad1.left_stick_x;
      strafe = gamepad1.left_stick_y * ((redTeam)? -1: 1);
      turn = -gamepad1.right_stick_x;


      // DPAD_DOWN button -> Fine tuning mode
      if ((gamepad1.dpad_down) && !changed) {
        slow = !slow;
        changed = true;
      } else if(!gamepad1.dpad_down) changed = false;

      // During slow mode, everything is slowed
      if (slow) {
        drivevar *= 0.0;
        strafe *= 0.0;
        turn *= 0.2;
      }




      // left trigger -> Run intake and the helper motor
      // to get the ball into the launcher
      if ((gamepad1.left_trigger >= 0.2) && !changed2) {
        runningActions.add(intake.on());
        runningActions.add(pew.launch());
        runningActions.add(stopper.Out());
        changed2 = true;
      } else if (!(gamepad1.left_trigger >= 0.2) && changed2) {
        runningActions.add(pew.set());
        runningActions.add(intake.off());
        changed2 = false;
      }

      // Stop the launcher and lower the hood
      if (gamepad1.x) {
        anglePos = 0.0;
        wheeelSpeed = 0.0;
      }

      if (gamepad2.dpad_up) { // debug wheel speed
        wheeelSpeed -= 0.01;
      }

      if (gamepad2.dpad_down) {
        wheeelSpeed += 0.01;
      }

      if (gamepad2.dpad_left) {
        turretOffset.set(turretOffset.get()-1);
      }

      if (gamepad2.dpad_right) {
        turretOffset.set(turretOffset.get()+1);
      }

      if (gamepad2.x) { // debug angle pos
        anglePos -= 0.01;
      }

      if (gamepad2.b) {
        anglePos += 0.01;
      }

      // Angles and Speeds

      // Heatmap
      double goalDist;
      goalDist = Math.sqrt(Math.pow((drive.localizer.getPose().position.x - ((redTeam)? -65.0 : -70.0)),2) + Math.pow((drive.localizer.getPose().position.y - ((redTeam)? 65.0 : -60.0)),2));
      anglePos = 0.00620728 * goalDist - 0.358573; // 0.00620728x-0.358573
      wheeelSpeed = 0.00424584 * goalDist + 0.762331; // 0.00424584x+0.762331


/*      // Far
      if (gamepad1.a && !changed3) {
        anglePos = 0.52; // Min 0 --- Max 1
        wheeelSpeed = 1.342; // Min 0 --- Max 2
        changed3 = true;  // try not to mess with changed3, it makes the button work when pressed
      } else if (!gamepad1.a && changed3) {
        changed3 = false;
      }

      // Medium
      if (gamepad1.b && !changed4) {
        anglePos = 0.22;
        wheeelSpeed = 1.17;
        changed4 = true;
      } else if (!gamepad1.b && changed4) {
        changed4 = false;
      }

      //Close
      if (gamepad1.y && !changed5) {
        anglePos = 0.0;
        wheeelSpeed = 1.0/*5*/;/*
        changed5 = true;
      } else if (!gamepad1.y  && changed5) {
        changed5 = false;
      }*/


      // Right trigger -> push ball into launcher
      // stops the intake servo so balls don't get stuck under
      if (gamepad1.right_trigger >= 0.2 && !changed6) {
        runningActions.add(pew.launch());
        runningActions.add(intake.on());
        runningActions.add(stopper.In());
        changed6 = true;
      } else if (!(gamepad1.right_trigger >= 0.2) && changed6) {
        runningActions.add(pew.set());
        runningActions.add(intake.off());
        runningActions.add(stopper.Out());
        changed6 = false;
      }

      if (gamepad1.right_stick_button && !changed9) {
        spin.setInitialized(false);
        changed9 = true;
      } else if (!gamepad1.right_stick_button && changed9) {
        changed9 = false;
      }

      // updated based on gamepads
      // Run artifacts backwards
      if (gamepad1.left_bumper && !changed8) {
        //pew.setPower(-0.4);
        runningActions.add(pew.back());
        //intake.setPower(-0.4);
        runningActions.add(intake.back());
        changed8 = true;
      } else if (!(gamepad1.left_bumper) && changed8) {
        //intake.setPower(0);
        runningActions.add(intake.off());
        //pew.setPower(0);
        runningActions.add(pew.set());
        changed8 = false;
      }

      if (gamepad1.dpad_up && !changed7) {
        turretLock.getAndSet(!turretLock.get());
        changed7 = true;
      } else if (!gamepad1.dpad_up) {
        changed7 = false;
      }

      double heading = drive.localizer.getPose().heading.toDouble();
      double newdrivevar = strafe * Math.cos(heading) - drivevar * Math.sin(heading);
      double newstrafe = strafe * Math.sin(heading) + drivevar * Math.cos(heading);

      movement = new PoseVelocity2d(
              new Vector2d(
                      newstrafe,
                      newdrivevar
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




  }

}
