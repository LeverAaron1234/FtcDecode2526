package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.appendages.Angle;
import org.firstinspires.ftc.teamcode.appendages.Helper;
import org.firstinspires.ftc.teamcode.appendages.Intake;
import org.firstinspires.ftc.teamcode.appendages.LimelightCam;
import org.firstinspires.ftc.teamcode.appendages.Pew;
import org.firstinspires.ftc.teamcode.appendages.Shooter;
import org.firstinspires.ftc.teamcode.appendages.Spin;
import org.firstinspires.ftc.teamcode.appendages.Stopper;

import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;


@Autonomous
public final class BlueCloseAuto extends LinearOpMode {

    /*
     * Blue Goal ---- Red Goal
     *            -x
     *         -y    y+
     *            +x
     */

    // Robot size is 16.5 * 13.5 (in.)
    // center offset from corner: (8.25, 6.75) (also in.)

    @Override
    public void runOpMode() throws InterruptedException {
        // The starting position for the robot
        Pose2d beginPose = new Pose2d(-62.75,-40.25,0.0);

        // Instantiating the classes from the appendages folder
        Shooter shooter = new Shooter(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Pew pew = new Pew(hardwareMap);
        Angle angle = new Angle(hardwareMap);
        Stopper stopper = new Stopper(hardwareMap);

        // More instantiating, this time, the camera and the turret
        Limelight3A camq = hardwareMap.get(Limelight3A.class, "limelight");
        Spin spin = new Spin(hardwareMap);
        spin.resetTimer();



        // The limit switches on the turret
        TouchSensor leftLimit = hardwareMap.get(TouchSensor.class, "leftLimit");
        TouchSensor rightLimit = hardwareMap.get(TouchSensor.class, "rightLimit");

        // Quick making sure the robot isn't moving
        Actions.runBlocking(pew.set());

        // Make the camera work correctly, by putting it on the correct pipeline.
        // The python dictionary shows what numbers correspond to the different targets
        camq.pipelineSwitch(3);// {0: "goal", 1: "obelisk", 2: "RedGoal", 3: "BlueGoal"}
        camq.start(); // start the camera

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

        SequentialAction FirstSet = new SequentialAction(
                shooter.low(),
                angle.middle(),
                drive.actionBuilder(beginPose) // tell the drive it's start positon
                        .splineTo(new Vector2d(-20,-20),0.0)
                        .build(),
                new SleepAction(1.0), // wait for the shooter and turret to start up
                intake.on(), // Fire!!!
                stopper.In(),
                pew.launch(),
                new SleepAction(2.0), // Wait till it's done
                // Stop firing, but keep the intake on
                stopper.Out(),
                pew.set()
        );
        // turretOffset.set(-15)
        SequentialAction SecondSet = new SequentialAction(
                drive.actionBuilder(new Pose2d(-20,-20,beginPose.heading.toDouble())) // Tell the drive its pos
                        // move in front of the first set of artifacts
                        .splineTo(new Vector2d(-2,-30),Math.toRadians(270))
                        .splineTo(new Vector2d(-2, -65), Math.toRadians(270)) // grab the artifacts
                        .splineTo(new Vector2d(-20,-30),Math.toRadians(0)) // go to fire pos
                        .build(),
                intake.on(), // Fire!!!
                stopper.In(),
                pew.launch(),
                new SleepAction(2.0), // Wait till it's done
                // Stop firing, but keep the intake on
                stopper.Out(),
                pew.set()
        );
        // drive.updatePoseEstimate()
        SequentialAction ThirdSet = new SequentialAction(
                drive.actionBuilder(new Pose2d(-20,-24,Math.toRadians(330))) // Tell the drive its pos
                        // move in front of the second set of artifacts
                        .splineTo(new Vector2d(22,-30), Math.toRadians(270))
                        .splineTo(new Vector2d(22, -65), Math.toRadians(270)) // grab the artifacts
                        .splineTo(new Vector2d(22, -50), Math.toRadians(270)) // grab the artifacts
                        .splineTo(new Vector2d(-20,-30),Math.toRadians(0)) // go to fire pos
                        .build(),
                intake.on(), // Fire!!!
                stopper.In(),
                pew.launch(),
                new SleepAction(2.0), // Wait till it's done
                // Stop firing, but keep the intake on
                stopper.Out(),
                pew.set()
        );
        // drive.updatePoseEstimate()
        SequentialAction LastSet = new SequentialAction(
                drive.actionBuilder(new Pose2d(-20,-24,Math.toRadians(120))) // Tell the drive its pos
                        // move in front of the third set of artifacts
                        .strafeToSplineHeading(new Vector2d(46,-30), Math.toRadians(270))
                        .strafeTo(new Vector2d(46, -75)) // grab the artifacts
                        .strafeToSplineHeading(new Vector2d(24,-45),Math.toRadians(270)) // go to end pos
                        .build()
        );

        waitForStart();

        thread.start(); // start the turret thread

        // The turret is moving now

        telemetry.update(); // you wanted comments, you get comments


        // About here, the robot should start moving.
        // Note to Elijah and Dexter: Here is where you code


        // make the shooter and angle go to firing positions
        Actions.runBlocking(
                FirstSet
        );
        turretOffset.set(-15);
        Actions.runBlocking(
                SecondSet
        );
        drive.updatePoseEstimate();
        Actions.runBlocking(
                ThirdSet
        );
        drive.updatePoseEstimate();
        Actions.runBlocking(
                LastSet
        );


        // Note to Elijah and Dexter: Good job you did it

        thread.interrupt(); // make sure that the thread isn't running anymore, we don't need it.

        RobotPose.lastRobotPose = drive.localizer.getPose(); // update the robot pose
        RobotPose.updated = true; // tell the updated pose that it was changed, because yes.
    }

}
// The end...
