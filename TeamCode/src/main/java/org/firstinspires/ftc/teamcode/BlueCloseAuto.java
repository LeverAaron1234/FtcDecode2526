package org.firstinspires.ftc.teamcode;

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
import org.firstinspires.ftc.teamcode.tuning.SquareTest;


@Autonomous
public final class BlueCloseAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(-63, -39, Math.toRadians(270));

        Shooter shooter = new Shooter(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Pew pew = new Pew(hardwareMap);
        Angle angle = new Angle(hardwareMap);
        Actions.runBlocking(pew.set());
        Helper helper = new Helper(hardwareMap);
        LimelightCam camq = new LimelightCam(hardwareMap);


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
                new ParallelAction(
                        camq.update(),
                        shooter.low(),
                        intake.off(),
                        pew.set(),
                        helper.forward(),
                        angle.down()
                )
        );


        telemetry.update();


        Actions.runBlocking(new SequentialAction(
                drive.actionBuilder(beginPose)
                        .strafeToSplineHeading(new Vector2d(-33,-19), Math.toRadians(233)) // Point to goal
                        .strafeTo(new Vector2d(-33,-19))
                        .build(),
                intake.on(),
                new SleepAction(0.5)
        ));

        for (int i=0; i<3; i++){// Fire the (3) artifacts
            Actions.runBlocking(new SequentialAction(
                    helper.off(),
                    intake.off(),
                    new SleepAction(0.1),
                    pew.launch(),
                    new SleepAction(0.1),
                    pew.set(),
                    new SleepAction(0.1),
                    helper.forward(),
                    intake.firein(),
                    new SleepAction(0.75)
            ));
        }

        Actions.runBlocking(new SequentialAction(
                intake.on(),
                drive.actionBuilder(new Pose2d(-33,-16.5,Math.toRadians(233)))
                        .strafeToSplineHeading(new Vector2d(-10, -25), Math.toRadians(270))
                        .build(),
                intake.on(),
                drive.actionBuilder(new Pose2d(-14, -25, Math.toRadians(270)))
                        .strafeTo(new Vector2d(-14, -52), new TranslationalVelConstraint(12.5))
                        .strafeToSplineHeading(new Vector2d(-33, -19), Math.toRadians(233))
                        .strafeTo(new Vector2d(-33,-19))
                        .build(),
                new SleepAction(0.5)
        ));

        for (int i=0; i<3; i++){// Fire the (3) artifacts
            Actions.runBlocking(new SequentialAction(
                    helper.off(),
                    intake.off(),
                    new SleepAction(0.1),
                    pew.launch(),
                    new SleepAction(0.1),
                    pew.set(),
                    new SleepAction(0.1),
                    helper.forward(),
                    intake.firein(),
                    new SleepAction(0.75)
            ));
        }


        Actions.runBlocking(new SequentialAction(
                intake.on(),
                drive.actionBuilder(new Pose2d(-33,-17, Math.toRadians(233)))
                        .strafeToSplineHeading(new Vector2d(10, -25), Math.toRadians(270))
                        .strafeTo(new Vector2d(10,-52), new TranslationalVelConstraint(12.5))
                        .strafeToSplineHeading(new Vector2d(-33,-19), Math.toRadians(233))
                        .strafeTo(new Vector2d(-33,-19))
                        .build()
        ));

        for (int i=0; i<3; i++){// Fire the (3) artifacts
            Actions.runBlocking(new SequentialAction(
                    helper.off(),
                    intake.off(),
                    new SleepAction(0.1),
                    pew.launch(),
                    new SleepAction(0.1),
                    pew.set(),
                    new SleepAction(0.1),
                    helper.forward(),
                    intake.firein(),
                    new SleepAction(0.75)
            ));
        }



}}
