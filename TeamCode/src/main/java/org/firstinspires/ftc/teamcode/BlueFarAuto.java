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

        AtomicBoolean turretLock = new AtomicBoolean(false);

        AtomicInteger turretOffset = new AtomicInteger(30);

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


        Actions.runBlocking(new ParallelAction(
                shooter.stop(),
                pew.set(),
                intake.off(),
                angle.down(),
                stopper.Out()
        ));


        telemetry.update();

        /*=======================================WAIT FOR START=======================================*/

        waitForStart();

        thread.start(); // start the above defined thread


        telemetry.update();


        Actions.runBlocking(
                new SequentialAction(
                        shooter.full(),
                        angle.far()
                )
        );


        thread.interrupt();

        RobotPose.lastRobotPose = drive.localizer.getPose();
        RobotPose.updated = true;

    }

}
