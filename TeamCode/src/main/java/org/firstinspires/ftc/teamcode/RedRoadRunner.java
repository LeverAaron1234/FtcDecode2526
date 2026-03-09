package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
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

import org.firstinspires.ftc.teamcode.appendeges.Angle;
import org.firstinspires.ftc.teamcode.appendeges.Intake;
import org.firstinspires.ftc.teamcode.appendeges.Pew;
import org.firstinspires.ftc.teamcode.appendeges.Shooter;
import org.firstinspires.ftc.teamcode.appendeges.Spin;
import org.firstinspires.ftc.teamcode.appendeges.TurretSpinner;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;

@TeleOp(group="Linear Opmode")
public class RedRoadRunner extends LinearOpMode {

  private final ElapsedTime runtime = new ElapsedTime();

  @Override
  public void runOpMode() {

    Pose2d beginPose = null;
    boolean beginPoseValid = false;

    if (RobotPose.lastRobotPose != null) {
      beginPose = RobotPose.lastRobotPose;
      beginPoseValid = true;
    } else {
      beginPose = new Pose2d(0,0,0);
    }


    Shooter shooter = new Shooter(hardwareMap);
    Intake intake = new Intake(hardwareMap);
    Pew pew = new Pew(hardwareMap);
    Angle angle = new Angle(hardwareMap);

    Limelight3A camq = hardwareMap.get(Limelight3A.class, "limelight");
    Spin spin = new Spin(hardwareMap);
    spin.resetTimer();




    TouchSensor leftLimit = hardwareMap.get(TouchSensor.class, "leftLimit");
    TouchSensor rightLimit = hardwareMap.get(TouchSensor.class, "rightLimit");

    Actions.runBlocking(pew.set());


    camq.pipelineSwitch(2);// {0: "goal", 1: "obelisk", 2: "RedGoal", 3: "BlueGoal"}
    camq.start();

    MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

    FtcDashboard dash = FtcDashboard.getInstance();
    List<Action> runningActions = new ArrayList<>();

    AtomicBoolean turretLock = new AtomicBoolean(false);

    // Autonomous threading so that the camera can control the turret in a loop
    Thread thread = new Thread(() -> { // () -> {...} is a lambda expression
      while(opModeIsActive())
      {
        if (Thread.currentThread().isInterrupted()) {
          spin.update(camq.getLatestResult(), leftLimit.isPressed(), rightLimit.isPressed(), turretLock.get());
          break;
        }

        spin.update(camq.getLatestResult(), leftLimit.isPressed(), rightLimit.isPressed(), turretLock.get());
      }
    });

    Actions.runBlocking(new ParallelAction(
            shooter.stop(),
            pew.set(),
            intake.off(),
            angle.down()
    ));


    telemetry.update();


    /*=======================================WAIT FOR START=======================================*/
    waitForStart();

    thread.start(); // start the above defined thread

    runtime.reset();
    spin.resetTimer();


    //  ----------Define Variables----------

    boolean changed = false;
    boolean changed2 = false;
    boolean changed3 = false;
    boolean changed4 = false;
    boolean changed5 = false;
    boolean changed6 = false;
    boolean changed7 = false;
    boolean changed8 = false;


    PoseVelocity2d movement = new PoseVelocity2d(new Vector2d(0,0),0);

    /*===================================WHILE OPMODE IS RUNNING==================================*/
    while (opModeIsActive()) {


      TelemetryPacket packet = new TelemetryPacket();

      //drive = -gamepad1.left_stick_x;
      //strafe = gamepad1.left_stick_y;
      //turn = gamepad1.right_stick_x;

      movement = new PoseVelocity2d(
              new Vector2d(
                  -gamepad1.left_stick_y,
                  -gamepad1.left_stick_x
              ),
              -gamepad1.right_stick_x
      );



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
