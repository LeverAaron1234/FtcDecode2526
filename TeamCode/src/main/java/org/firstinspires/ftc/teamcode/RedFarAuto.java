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
                        shooter.full(),
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
                        .strafeTo(new Vector2d(-57, -20))
                        .turnTo(Math.toRadians(-23))
                        .build(),
                new SleepAction(2)
        ));

        for (int i=0; i<3; i++){// Fire the (3) pre-loaded balls
            Actions.runBlocking(new SequentialAction(
                    helper.backward(),
                    intake.off(),
                    new SleepAction(0.1),
                    pew.launch(),
                    new SleepAction(0.5),
                    pew.set(),
                    new SleepAction(0.5),
                    helper.forward(),
                    intake.on(),
                    new SleepAction(2)
            ));
        }
        Actions.runBlocking(new SequentialAction(
                drive.actionBuilder(new Pose2d(-49,-20,Math.toRadians(-95)))
                        .strafeTo(new Vector2d(-49,-45), new TranslationalVelConstraint(7.5))
                        .build(),
                new SleepAction(0.1),
                intake.off()
        ));

        for (int i=0; i<3; i++){
            Actions.runBlocking(new SequentialAction(
                    helper.backward(),
                    intake.off(),
                    new SleepAction(0.1),
                    pew.launch(),
                    new SleepAction(0.5),
                    pew.set(),
                    new SleepAction(0.5),
                    helper.forward(),
                    intake.on(),
                    new SleepAction(2)
            ));
        }

        Actions.runBlocking(new SequentialAction(
                drive.actionBuilder(new Pose2d(-60,20,Math.toRadians(-17)))
                        .strafeTo(new Vector2d(-50,20))
                        .turnTo(Math.toRadians(-90))
                        .strafeTo(new Vector2d(-50,50), new TranslationalVelConstraint(10.0))
                        .build(),
                new SleepAction(0.1),
                intake.off(),
                drive.actionBuilder(new Pose2d(-50,50,-90))
                        .strafeTo(new Vector2d(-50,20))
                        .turnTo(-17)
                        .strafeTo(new Vector2d(-60,20))
                        .build()
        ));

        for (int i=0; i<3; i++){
            Actions.runBlocking(new SequentialAction(
                    helper.backward(),
                    intake.off(),
                    new SleepAction(0.1),
                    pew.launch(),
                    new SleepAction(0.5),
                    pew.set(),
                    new SleepAction(0.5),
                    helper.forward(),
                    intake.on(),
                    new SleepAction(2)
            ));
        }




    }

}