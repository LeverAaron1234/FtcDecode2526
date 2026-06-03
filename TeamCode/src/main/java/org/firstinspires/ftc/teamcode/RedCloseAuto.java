package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
//import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.appendages.Angle;
//import org.firstinspires.ftc.teamcode.appendages.Helper;
import org.firstinspires.ftc.teamcode.appendages.Intake;
//import org.firstinspires.ftc.teamcode.appendages.LimelightCam;
import org.firstinspires.ftc.teamcode.appendages.Pew;
import org.firstinspires.ftc.teamcode.appendages.Shooter;
import org.firstinspires.ftc.teamcode.appendages.Spin;
import org.firstinspires.ftc.teamcode.appendages.Stopper;

import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;


@Autonomous
public final class RedCloseAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // The starting position for the robot
        Pose2d beginPose = new Pose2d(-55.68,50.88,0.0);

        // Instantiating the classes from the appendages folder
        Shooter shooter = new Shooter(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Pew pew = new Pew(hardwareMap);
        Angle angle = new Angle(hardwareMap);
        Stopper stopper = new Stopper(hardwareMap);

        // More instantiating, this time, the camera and the turret
        //Limelight3A camq = hardwareMap.get(LimelightCam.class, "limelight");
        Spin spin = new Spin(hardwareMap);
        spin.resetTimer();



        // The limit switches on the turret
        TouchSensor leftLimit = hardwareMap.get(TouchSensor.class, "leftLimit");
        TouchSensor rightLimit = hardwareMap.get(TouchSensor.class, "rightLimit");

        // Quick making sure the robot isn't moving
        Actions.runBlocking(pew.set());

        // Make the camera work correctly, by putting it on the correct pipeline.
        // The python dictionary shows what numbers correspond to the different targets
        //camq.pipelineSwitch(3);// {0: "goal", 1: "obelisk", 2: "RedGoal", 3: "BlueGoal"}
        //camq.start(); // start the camera

        // Instantiating the chassis and its motors
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        // The Roadrunner Dashboard
        FtcDashboard dash = FtcDashboard.getInstance();

        // Start of the turret thread

        // So that the turret stops moving around so much
        AtomicBoolean turretLock = new AtomicBoolean(false);

        // So that you can offset the turret if needed.
        AtomicInteger turretOffset = new AtomicInteger(30);

        // Autonomous threading so that the camera can control the turret in a loop
        Thread thread = new Thread(() -> { // Lambda, such a funny word
            while(opModeIsActive()) // Same loop as teleOp
            {
                // Always wrap things, so that when they go wrong, they don't break.
                if (Thread.currentThread().isInterrupted()) {
                    // Update using camera then add return data to telemetry (Not currently used)
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
                    // Update using odometry, then return data to telemetry
                    List vals = spin.odomUpdate(drive, false, turretOffset.get(), turretLock.get());
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

                // Update using camera then add return data to telemetry
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
                // Yes, I did duplicate code. Shhhhhh...
                List vals = spin.odomUpdate(drive, false, turretOffset.get(), turretLock.get());
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


        // Make really sure that nothing moves
        Actions.runBlocking(new ParallelAction(
                shooter.stop(),
                pew.set(),
                intake.off(),
                angle.close(),
                stopper.Out()
        ));


        telemetry.update(); // update telemetry

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

        thread.start(); // start the turret thread


        telemetry.update(); // you wanted comments, you get comments


/*        Actions.runBlocking(camq.update())

        ;
        telemetry.addData("Tag Area", camq.getTagArea());
        telemetry.addData("Tagx", camq.getTagx());
        telemetry.addData("Tagy", camq.getTagy());
        telemetry.addData("TagID", camq.getTagid());
        telemetry.update();
*/

        Actions.runBlocking(
                new ParallelAction(
                        //camq.update(),
                        shooter.low(),
                        intake.on(),
                        pew.set(),
                        //helper.forward(),
                        angle.middle()
                )
        );

        telemetry.update();

        //tagid == 21: GPP
        //tagid == 22: PGP
        //tagid == 23: PPG

        Actions.runBlocking(
                new SequentialAction(
                        drive.actionBuilder(beginPose)
                            .strafeTo(new Vector2d(-40.8, 24))
                            .build(),
                        shooter.full(), // Ready your weapons and magic, for the enemy draws near
                        angle.close(), // Peer down your All-Seeing orbs to track the enemy position
                        new SleepAction(1.9999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999), // wait for the shooter and turret to start up
                        intake.on(), // Begin the casting ritual!!!
                        stopper.In(), // Don't let your magic overcome you, keep the flow steady to inflict maximum damage
                        pew.launch(), // Fireball!!!
                        new SleepAction(2.0), // Have patience, for the weary traveler needs time to rest
                        // Cease firing your spells, but keep your guard up, for they must be ready to slay soon
                        stopper.Out(),
                        pew.launch(),
                        drive.actionBuilder(new Pose2d(-40.8, 24, Math.toRadians(-135))) // Tell the drive its pos
                                // move in front of the first set of artifacts
                                .strafeToSplineHeading(new Vector2d(-12, 21.6), Math.toRadians(90))
                                .strafeTo(new Vector2d(-12, 55.2), new TranslationalVelConstraint(12.5))
                                .strafeToSplineHeading(new Vector2d(-31.2, 24), Math.toRadians(-135))
                                .build(),
                        intake.on(), // Begin the casting ritual!!!
                        stopper.In(), // Don't let your magic overcome you, keep the flow steady to inflict maximum damage
                        pew.launch(), // Fireball!!!
                        new SleepAction(2.0), // The weary traveler requires an additional period of sleep
                        // Stop firing, but keep the intake on
                        stopper.Out(),
                        pew.launch(),
                        drive.actionBuilder(new Pose2d(-40.8, 24, Math.toRadians(-135))) // Tell the drive its pos
                                // position thyself within reach of thy row the second
                                .strafeToSplineHeading(new Vector2d(12,20), Math.toRadians(90))
                                .strafeTo(new Vector2d(12, 45)) // grab the artifacts
                                .strafeToSplineHeading(beginPose.position,beginPose.heading) // go to fire pos
                                .build(),
                        intake.on(), // Begin the casting ritual!!!
                        stopper.In(), // Don't let your magic overcome you, keep the flow steady to inflict maximum damage
                        pew.launch(), // Fireball!!!
                        new SleepAction(2.0), // Wait till it's done
                        // Stop firing, but keep the intake on
                        stopper.Out(),
                        pew.launch(),
                        drive.actionBuilder(beginPose) // Tell the drive its pos
                                // move in front of the third set of artifacts
                                .strafeToSplineHeading(new Vector2d(-12,20), Math.toRadians(90))
                                .strafeTo(new Vector2d(-12, 45)) // grab the artifacts
                                .strafeToSplineHeading(new Vector2d(24,40),Math.toRadians(90)) // go to end pos
                                .build()
                ));
        Actions.runBlocking(new SequentialAction(// First 3 Artifacts
                drive.actionBuilder(beginPose)
                        .strafeTo(new Vector2d(-40.8, 24)) //
                        .build(),
                intake.on(),
                new SleepAction(0.5)
        ));

        for (int i = 0; i < 3; i++) {// Fire the (3) artifacts
            Actions.runBlocking(new SequentialAction(
                    intake.off(),
                    new SleepAction(0.1),
                    pew.launch(),
                    new SleepAction(0.1),
                    pew.set(),
                    new SleepAction(0.1),
                    intake.firein(),
                    new SleepAction(0.75)
            ));
        }

        Actions.runBlocking(new SequentialAction(// Second 3 Artifacts
                intake.on(),
                drive.actionBuilder(new Pose2d(-40.8, 24, Math.toRadians(57)))
                        .strafeToSplineHeading(new Vector2d(-12, 21.6), Math.toRadians(90))
                        .strafeTo(new Vector2d(-12, 55.2), new TranslationalVelConstraint(12.5))
                        .strafeToSplineHeading(new Vector2d(-31.2, 24), Math.toRadians(49.5))
                        .build(),
                new SleepAction(0.5)
        ));

        for (int i = 0; i < 3; i++) {// Fire the (3) artifacts
            Actions.runBlocking(new SequentialAction(
                    intake.off(),
                    new SleepAction(0.1),
                    pew.launch(),
                    new SleepAction(0.1),
                    pew.set(),
                    new SleepAction(0.1),
                    intake.firein(),
                    new SleepAction(0.75)
            ));
        }

        Actions.runBlocking(new SequentialAction(// Third 3 Artifacts
                intake.on(),
                drive.actionBuilder(new Pose2d(-31.2, 24, Math.toRadians(49.5)))
                        .strafeToSplineHeading(new Vector2d(12, 21.6), Math.toRadians(90))
                        .strafeTo(new Vector2d(12, 60))
                        .build()
        ));

        thread.interrupt(); // make sure that the thread isn't running anymore, we don't need it.

        RobotPose.lastRobotPose = drive.localizer.getPose(); // update the robot pose
        RobotPose.updated = true; // tell the updated pose that it was changed, because yes.
    }
}
