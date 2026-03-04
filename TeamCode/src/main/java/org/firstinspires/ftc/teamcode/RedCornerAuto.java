package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.appendeges.Angle;
import org.firstinspires.ftc.teamcode.appendeges.Helper;
import org.firstinspires.ftc.teamcode.appendeges.Intake;
import org.firstinspires.ftc.teamcode.appendeges.LimelightCam;
import org.firstinspires.ftc.teamcode.appendeges.Pew;
import org.firstinspires.ftc.teamcode.appendeges.Shooter;

@Autonomous
public final class RedCornerAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(-63, -24, Math.toRadians(0));

        Shooter shooter = new Shooter(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Pew pew = new Pew(hardwareMap);
        Angle angle = new Angle(hardwareMap);
        Helper helper = new Helper(hardwareMap);
        LimelightCam camq = new LimelightCam(hardwareMap);
        Actions.runBlocking(pew.set());


        camq.setPipeline(LimelightCam.Camera.Obelisk);
        camq.switchPipeline(1); //Obelisk

        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);



        Actions.runBlocking(new SequentialAction(
                shooter.stop(),
                pew.set(),
                helper.off(),
                intake.off(),
                angle.down()
        ));



        telemetry.update();
        waitForStart();



        Actions.runBlocking(camq.update());
        telemetry.addData("Tag Area", camq.getTagArea());
        telemetry.addData("Tagx", camq.getTagx());
        telemetry.addData("Tagy", camq.getTagy());
        telemetry.addData("TagID", camq.getTagid());
        telemetry.update();


        Actions.runBlocking(
                new SequentialAction(
                        camq.update(),
                        shooter.fuller(),
                        intake.off(),
                        pew.set(),
                        helper.off(),
                        angle.farblue()
                )
        );


        telemetry.update();

        Actions.runBlocking(new SequentialAction(
                drive.actionBuilder(beginPose)
                        .strafeToSplineHeading(new Vector2d(-56, -20), Math.toRadians(-20))
                        .build(),
                shooter.full(),
                new SleepAction(3)
        ));

        while (opModeIsActive()) {

            for (int i = 0; i < 3; i++) {
                Actions.runBlocking(new SequentialAction(
                        helper.off(),
                        intake.off(),
                        new SleepAction(0.10),
                        helper.backward(),
                        pew.launch(),
                        new SleepAction(0.15),
                        pew.set(),
                        new SleepAction(0.1),
                        helper.forward(),
                        intake.firein(),
                        new SleepAction(0.6)
                ));
            }

            Actions.runBlocking(new SequentialAction(
                    intake.on(),
                    drive.actionBuilder(new Pose2d(-56, -20, Math.toRadians(-24)))
                            .strafeTo(new Vector2d(-53,-20))
                            .strafeToSplineHeading(new Vector2d(-51,-20),Math.toRadians(-90))
                            .strafeTo(new Vector2d(-51, -70))
                            .build()
            ));

            if (getRuntime() > 25) {
                throw new RuntimeException("Remaining time < 5, pausing to get leave points");
            }

            Actions.runBlocking(new SequentialAction(
                    drive.actionBuilder(new Pose2d(-51,-70,Math.toRadians(-90)))
                            .strafeToSplineHeading(new Vector2d(-56, -20), Math.toRadians(-24))
                            .build()
            ));

        }
    }

}
