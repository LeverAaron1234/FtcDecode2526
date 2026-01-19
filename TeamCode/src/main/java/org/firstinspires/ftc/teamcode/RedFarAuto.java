/*
Red side Far Autonomous
FTC season DECODE
Elijah R. & Dexter G. with source code from Levi R.
7209 Tech Hogs Robotics
1/6/2026

*/



package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
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
public final class RedFarAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(-63, -24, Math.toRadians(0));// Starting Position

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
                        intake.on(),
                        pew.set(),
                        helper.forward(),
                        angle.far()
                )
        );


        telemetry.update();

        //tagid == 21: GPP
        //tagid == 22: PGP
        //tagid == 23: PPG
        /*
        while (camq.getTagid() != 0) {
            Actions.runBlocking(camq.update());
        }
        if (camq.getTagid() == 21) {        // GPP

        } else if (camq.getTagid() == 22) { // PGP

        } else if (camq.getTagid() == 23) { // PPG

        }*/
        Actions.runBlocking(new SequentialAction(// Move to, and sense the oblisk to get the pattern
                /*drive.actionBuilder(beginPose)
                        .strafeTo(new Vector2d(-87, -24))
                        .turnTo(Math.toRadians(343))
                        .build(),
                camq.update(),*/
                drive.actionBuilder(beginPose)
                        .splineToLinearHeading(new Pose2d(-57, -20,Math.toRadians(-23)),0)
                        .build(),
                new SleepAction(2)
        ));

        for (int i=0; i<3; i++){// Fire the (3) pre-loaded balls
            Actions.runBlocking(new SequentialAction(
                    helper.off(),
                    intake.off(),
                    new SleepAction(0.1),
                    pew.launch(),
                    new SleepAction(0.1),
                    pew.set(),
                    new SleepAction(0.1),
                    helper.forward(),
                    intake.on(),
                    new SleepAction(0.75)
            ));
        }
        Actions.runBlocking(new SequentialAction(
                drive.actionBuilder(new Pose2d(-57,-20,Math.toRadians(-23)))
                        .splineTo(new Vector2d(-24,-39),Math.toRadians(-90))
                        .strafeTo(new Vector2d(-24,-64), new TranslationalVelConstraint(15.0))
                        .strafeToSplineHeading(new Vector2d(-52, -18), Math.toRadians(-24))
                        .build(),
                new SleepAction(0.1),
                intake.off()
        ));

        for (int i=0; i<3; i++){
            Actions.runBlocking(new SequentialAction(
                    helper.off(),
                    intake.off(),
                    new SleepAction(0.1),
                    pew.launch(),
                    new SleepAction(0.15),
                    pew.set(),
                    new SleepAction(0.1),
                    helper.forward(),
                    intake.on(),
                    new SleepAction(0.75)
            ));
        }
        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(-57,-20,Math.toRadians(-23)))
                        .strafeTo(new Vector2d(-40,-20))
                        .build()
        );


    }

}