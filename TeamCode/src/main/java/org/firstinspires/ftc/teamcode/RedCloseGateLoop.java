package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
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


@Autonomous(preselectTeleOp = "FCRR")
public final class RedCloseGateLoop extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // The starting position for the robot
        Pose2d beginPose = new Pose2d(-62.75,40.25,0.0);
        Pose2d firingPose = new Pose2d(-20,13,Math.toRadians(45));// Can be changed as the enemy move close or raises their shields

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
        AtomicInteger turretOffset = new AtomicInteger(0);

        // Autonomous threading so that the camera can control the turret in a loop
        Thread thread = new Thread(() -> { // Lambda, such a funny word
            while(opModeIsActive()) // Same loop as teleOp
            {
                // Always wrap things, so that when they go wrong, they don't break.
                if (Thread.currentThread().isInterrupted() || isStopRequested()) {
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
                    List vals = spin.odomUpdate(drive, true, turretOffset.get(), leftLimit.isPressed(), rightLimit.isPressed(), turretLock.get());
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
                List vals = spin.odomUpdate(drive, true, turretOffset.get(), leftLimit.isPressed(), rightLimit.isPressed(), turretLock.get());
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
                        intake.on(),
                        pew.set(),
                        angle.varangle(0.065),
                        shooter.varshooter(1.125),
                        drive.actionBuilder(beginPose)
                                .setTangent(0.0)
                                .splineToSplineHeading(firingPose,Math.toRadians(-30))
                                .build()
                )
        );

        telemetry.update();

        //tagid == 21: GPP
        //tagid == 22: PGP
        //tagid == 23: PPG

        Actions.runBlocking(
                new SequentialAction(
                        //shooter.varshooter(1.27), // Ready your weapons and magic, for the enemy draws near
                        //angle.varangle(0.11), // Peer down your All-Seeing orbs to track the enemy position
                        new SleepAction(0.5), // wait for the shooter and turret to start up
                        intake.on(), // Begin the casting ritual!!!
                        stopper.In(), // Don't let your magic overcome you, keep the flow steady to inflict maximum damage
                        pew.launch(), // Fireball!!!
                        new SleepAction(1.0), // Have patience, for the weary traveler needs time to rest
                        // Cease firing your spells, but keep your guard up, for they must be ready to slay soon
                        stopper.Out(),
                        pew.launch(),
                        intake.on()
                ));

        Actions.runBlocking( // grab middle set
                new SequentialAction(
                        drive.actionBuilder(firingPose)
                                .setTangent(Math.toRadians(45))
                                .splineToSplineHeading(new Pose2d(2,50,Math.toRadians(90)), Math.toRadians(90))
                                .setTangent(Math.toRadians(-90))
                                .splineToSplineHeading(new Pose2d(-20,13,Math.toRadians(45)),Math.toRadians(-150))
                                .build(),
                        stopper.In(), // Don't let your magic overcome you, keep the flow steady to inflict maximum damage
                        pew.launch(), // Fireball!!!
                        new SleepAction(1.0), // Have patience, for the weary traveler needs time to rest
                        // Cease firing your spells, but keep your guard up, for they must be ready to slay soon
                        stopper.Out(),
                        pew.launch(),
                        intake.on()
                )
        );

        //while(getRuntime() <= 23){ // take from goal
        //    if (isStopRequested()) {break;}
        Actions.runBlocking(
                new SequentialAction(
                        drive.actionBuilder(new Pose2d(6,15,Math.toRadians(80)))
                                .setTangent(Math.toRadians(60))
                                .splineToSplineHeading(new Pose2d(6,45,Math.toRadians(110)),Math.toRadians(90))
                                .waitSeconds(1.5)
                                .setTangent(Math.toRadians(-90))
                                .splineToSplineHeading(new Pose2d(-20,13,Math.toRadians(45)),Math.toRadians(-150))
                                .build(),
                        stopper.In(), // Don't let your magic overcome you, keep the flow steady to inflict maximum damage
                        pew.launch(), // Fireball!!!
                        new SleepAction(1.0), // Have patience, for the weary traveler needs time to rest
                        // Cease firing your spells, but keep your guard up, for they must be ready to slay soon
                        stopper.Out(),
                        pew.launch(),
                        intake.on()
                )
        );
        Actions.runBlocking( // Take from goal #2
                new SequentialAction(
                        drive.actionBuilder(new Pose2d(6,15,Math.toRadians(80)))
                                .setTangent(Math.toRadians(60))
                                .splineToSplineHeading(new Pose2d(6,45,Math.toRadians(110)),Math.toRadians(90))
                                .waitSeconds(1.5)
                                .setTangent(Math.toRadians(-90))
                                .splineToSplineHeading(new Pose2d(-20,13,Math.toRadians(45)),Math.toRadians(-150))
                                .build(),
                        stopper.In(), // Don't let your magic overcome you, keep the flow steady to inflict maximum damage
                        pew.launch(), // Fireball!!!
                        new SleepAction(1.0), // Have patience, for the weary traveler needs time to rest
                        // Cease firing your spells, but keep your guard up, for they must be ready to slay soon
                        stopper.Out(),
                        pew.launch(),
                        intake.on()
                )
        );
        //}

        Actions.runBlocking( // grab closest set
                new SequentialAction(
                        angle.varangle(0.06),
                        shooter.varshooter(1.11),
                        drive.actionBuilder(new Pose2d(-20,13,Math.toRadians(45)))
                                .setTangent(Math.toRadians(60))
                                .splineToSplineHeading(new Pose2d(-24,40,Math.toRadians(90)),Math.toRadians(90),new TranslationalVelConstraint(20.0))
                                .setTangent(Math.toRadians(-90))
                                .splineToSplineHeading(new Pose2d(-36,8,Math.toRadians(0)),Math.toRadians(-120))
                                //.strafeToSplineHeading(new Vector2d(-12,-24),Math.toRadians(0))
                                .build(),
                        intake.on(),
                        stopper.In(),
                        pew.launch(),
                        new SleepAction(3.0)

                        //,
                        /*stopper.In(), // Don't let your magic overcome you, keep the flow steady to inflict maximum damage
                        pew.launch(), // Fireball!!!
                        new SleepAction(2.0), // Have patience, for the weary traveler needs time to rest
                        // Cease firing your spells, but keep your guard up, for they must be ready to slay soon
                        stopper.Out(),
                        pew.launch(),
                        intake.on()*/
                )
        );


        thread.interrupt(); // make sure that the thread isn't running anymore, we don't need it.

        RobotPose.lastRobotPose = drive.localizer.getPose(); // update the robot pose
        RobotPose.redTeam = true;
        RobotPose.startFar = false;
        RobotPose.updated = true; // tell the updated pose that it was changed, because yes.
    }
}
