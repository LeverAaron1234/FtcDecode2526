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
public class TurretTest extends LinearOpMode {

  private final ElapsedTime runtime = new ElapsedTime();

  @Override
  public void runOpMode() {

    Pose2d beginPose = null;
    boolean beginPoseValid = false;

    if (RobotPose.updated) {
      beginPose = RobotPose.lastRobotPose;
      beginPoseValid = true;
    } else {
      beginPose = new Pose2d(61.95,-18.62,0.0);
    }


    Spin spin = new Spin(hardwareMap);
    spin.resetTimer();




    TouchSensor leftLimit = hardwareMap.get(TouchSensor.class, "leftLimit");
    TouchSensor rightLimit = hardwareMap.get(TouchSensor.class, "rightLimit");


    MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

    FtcDashboard dash = FtcDashboard.getInstance();
    List<Action> runningActions = new ArrayList<>();

    AtomicBoolean turretLock = new AtomicBoolean(false);

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
            List vals = spin.odomUpdate(drive, false, turretOffset.get(), leftLimit.isPressed(), rightLimit.isPressed(), turretLock.get());
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
          List vals = spin.odomUpdate(drive, false, turretOffset.get(), leftLimit.isPressed(), rightLimit.isPressed(), turretLock.get());
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


    /*=======================================WAIT FOR START=======================================*/
    waitForStart();

    thread.start(); // start the above defined thread

    runtime.reset();
    spin.resetTimer();


    //  ----------Define Variables----------

    boolean changed7 = false;
    boolean changed9 = false;


    PoseVelocity2d movement = new PoseVelocity2d(new Vector2d(0,0),0);

    /*===================================WHILE OPMODE IS RUNNING==================================*/
    while (opModeIsActive()) {


      TelemetryPacket packet = new TelemetryPacket();

      // Debug angle / wheel speed
      if (gamepad1.dpad_left) {
        /*if (slow) {
          wheeelSpeed -= 0.01;
        } else {
          anglePos -= 0.01;
        }*/
        turretOffset.set(turretOffset.get() - 1);
      }

      if (gamepad1.dpad_right) {
        /*if (slow) {
          wheeelSpeed += 0.01;
        } else {
          anglePos += 0.01;
        }*/
        turretOffset.set(turretOffset.get() + 1);
      }
      packet.put("turretOffset", turretOffset.get());


      if (gamepad1.right_stick_button && !changed9) {
        spin.setInitialized(false);
        changed9 = true;
      } else if (!gamepad1.right_stick_button && changed9) {
        changed9 = false;
      }

      if (gamepad1.dpad_up && !changed7) {
        turretLock.getAndSet(!turretLock.get());
        changed7 = true;
      } else if (!gamepad1.dpad_up) {
        changed7 = false;
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
