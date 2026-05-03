package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
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

@Autonomous(preselectTeleOp = "BlueRoadRunner")
public final class BlueFarAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // The starting position for the robot
        Pose2d beginPose = new Pose2d(61.95,-18.62,0.0);

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
                angle.down(),
                stopper.Out()
        ));


        telemetry.update(); // update telemetry

        /*=======================================WAIT FOR START=======================================*/

        waitForStart();

        thread.start(); // start the turret thread


        telemetry.update(); // you wanted comments, you get comments


        // make the shooter and angle go to firing positions
        Actions.runBlocking(
                new SequentialAction(
                        shooter.full(),
                        angle.far()
                )
        );


        thread.interrupt(); // make sure that the thread isn't running anymore, we don't need it.

        RobotPose.lastRobotPose = drive.localizer.getPose(); // update the robot pose
        RobotPose.updated = true; // tell the updated pose that it was changed, because yes.
    }

}
// The end...






















// What, you thought something was here?

