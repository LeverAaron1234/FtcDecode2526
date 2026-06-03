package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
// Very important
// import unresolvable;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.appendages.Angle;
import org.firstinspires.ftc.teamcode.appendages.Intake;
import org.firstinspires.ftc.teamcode.appendages.Pew;
import org.firstinspires.ftc.teamcode.appendages.Shooter;
import org.firstinspires.ftc.teamcode.appendages.Spin;
import org.firstinspires.ftc.teamcode.appendages.Stopper;

import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;

@Autonomous(preselectTeleOp = "RedRoadRunner")
public final class RedFarAuto extends LinearOpMode {

    /*
     * Blue Goal ---- Red Goal
     *            -x
     *         -y    y+
     *            +x
     */

    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d beginPose = new Pose2d(62.4,14.88,0.0);



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


        camq.pipelineSwitch(2);// {0: "goal", 1: "obelisk", 2: "RedGoal", 3: "BlueGoal"}
        camq.start();

        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        FtcDashboard dash = FtcDashboard.getInstance();

        AtomicBoolean turretLock = new AtomicBoolean(false);

        AtomicInteger turretOffset = new AtomicInteger(-12);

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
                    List vals = spin.odomUpdate(drive, true, turretOffset.get(), turretLock.get());
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
                List vals = spin.odomUpdate(drive, true, turretOffset.get(), turretLock.get());
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
                angle.close(),
                stopper.Out()
        ));


        telemetry.update();

        double head = drive.localizer.getPose().heading.toDouble();

        while (opModeInInit()) {
            telemetry.addLine("Gyro");
            telemetry.addData("Started at", head);
            telemetry.addData("Current", drive.localizer.getPose().heading.toDouble());
            telemetry.update();
            drive.updatePoseEstimate();
        }

        /*=======================================WAIT FOR START=======================================*/

        waitForStart();

        thread.start(); // start the above defined thread


        // Me and Dexter edit stuff past here

        /*
        Good Info:
            stopper.In() takes the thing out of the way of the balls
            stopper.Out() puts the thing into the way of the balls
            intake.on()
            intake.off()
            These are self explanatory
            pew.set()
            pew.launch()
            These are not
            (these make a wheel go spin spin and you can shove balls onto them to make them go flying)
         */

        telemetry.update();

        Actions.runBlocking(
                new SequentialAction(
                        shooter.full(),// Ready you spells and weapon for the enemy draws near
                        angle.far(),// Make sure you have a least some long ranged spells
                        new SleepAction(2.0), // wait for the shooter and turret to start up
                        intake.on(), // Fireball!!!
                        stopper.In(),
                        pew.launch(),
                        new SleepAction(2.0), // Wait till it's done
                        // Stop firing, but keep the intake on
                        stopper.Out(),
                        pew.launch(),
                        drive.actionBuilder(beginPose) // Tell the drive its pos
                                // position thyself in front of the first set of artifacts
                                .strafeToSplineHeading(new Vector2d(24,20), Math.toRadians(90))
                                .strafeTo(new Vector2d(24, 50)) // grab the artifacts
                                .strafeToSplineHeading(beginPose.position,beginPose.heading) // go to fire pos
                                .build(),
                        intake.on(), // Fireball!!!
                        stopper.In(),// Start firing for the enemy let down his guard
                        pew.launch(),
                        new SleepAction(2.0), // Wait till it's done
                        // Stop firing, but keep the intake on
                        stopper.Out(),
                        pew.launch(),
                        drive.actionBuilder(beginPose) // Tell the drive its pos
                                // position thyself in front of the second set of artifacts
                                .strafeToSplineHeading(new Vector2d(0,20), Math.toRadians(90))
                                .strafeTo(new Vector2d(0, 50)) // grab the artifacts
                                .strafeToSplineHeading(beginPose.position,beginPose.heading) // go to fire pos
                                .build(),
                        intake.on(), // Fireball!!!
                        stopper.In(),
                        pew.launch(),
                        new SleepAction(2.0), // Wait till it's done
                        // Stop firing, but keep the intake on
                        stopper.Out(),
                        pew.launch(),
                        drive.actionBuilder(beginPose) // Tell the drive its pos
                                // position thyself in front of the third set of artifacts
                                .strafeToSplineHeading(new Vector2d(-24,20), Math.toRadians(90))
                                .strafeTo(new Vector2d(-24, 45)) // grab the artifacts
                                .strafeToSplineHeading(new Vector2d(24,45),Math.toRadians(90)) // go to end pos
                                .build()
                )
        );


        Actions.runBlocking(new SequentialAction(
                drive.actionBuilder(beginPose)
                        .strafeTo(new Vector2d(drive.localizer.getPose().position.x,drive.localizer.getPose().position.y+25))
                        .build()
        ));


        thread.interrupt(); // make sure that the thread isn't running anymore, we don't need it.

        RobotPose.lastRobotPose = drive.localizer.getPose(); // update the robot pose
        RobotPose.updated = true; // tell the updated pose that it was changed, because yes.
    }

}