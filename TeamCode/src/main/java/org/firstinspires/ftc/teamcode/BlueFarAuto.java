package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.appendeges.Angle;
import org.firstinspires.ftc.teamcode.appendeges.Helper;
import org.firstinspires.ftc.teamcode.appendeges.Intake;
import org.firstinspires.ftc.teamcode.appendeges.Intake2;
import org.firstinspires.ftc.teamcode.appendeges.LimelightCam;
import org.firstinspires.ftc.teamcode.appendeges.Pew;
import org.firstinspires.ftc.teamcode.appendeges.Shooter;
import org.firstinspires.ftc.teamcode.appendeges.Spin;
import org.firstinspires.ftc.teamcode.appendeges.TurretSpinner;

import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;

@Autonomous
public final class BlueFarAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(64, -29, Math.toRadians(-90));

        Shooter shooter = new Shooter(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Intake2 intake2 = new Intake2(hardwareMap);
        Pew pew = new Pew(hardwareMap);
        Angle angle = new Angle(hardwareMap);

        Limelight3A camq = hardwareMap.get(Limelight3A.class, "limelight");
        Spin spin = new Spin(hardwareMap);
        spin.resetTimer();




        TouchSensor leftLimit = hardwareMap.get(TouchSensor.class, "leftLimit");
        TouchSensor rightLimit = hardwareMap.get(TouchSensor.class, "rightLimit");

        Actions.runBlocking(pew.set());


        camq.pipelineSwitch(3);// {0: "goal", 1: "obelisk", 2: "RedGoal", 3: "BlueGoal"}
        camq.start();

        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

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
                intake2.off(),
                angle.down()
        ));


        telemetry.update();
        waitForStart();

        thread.start(); // start the above defined thread


        telemetry.update();


        Actions.runBlocking(
                new SequentialAction(
                        shooter.full(),
                        angle.far()
                )
        );


        telemetry.update();

        Actions.runBlocking(new SequentialAction(
                drive.actionBuilder(beginPose)
                        .strafeToSplineHeading(new Vector2d(60,- 20), Math.toRadians(-80))
                        .build(),
                shooter.full(),
                new SleepAction(1.5)
        ));

        if (!camq.getLatestResult().isValid()) {
            turretLock.set(true);
        }
        Actions.runBlocking(new SequentialAction(
                intake.on(),
                intake2.on(),
                pew.launch(),
                new SleepAction(2.0)
        ));


        thread.interrupt();

    }

}
