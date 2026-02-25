/*
Red Far Auto
FTC season DECODE
Elijah R. & Dexter G. with source code from Levi R.
7209 Tech Hogs Robotics
started editing 1/6/2026

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
        camq.switchPipeline(1); //1 = Obelisk, 2 = Goals

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

// I just wanted to do this.
/*
nonononononononononononononononononononononononononononono
*/        Actions.runBlocking(
                new SequentialAction(
                        camq.update(),
                        shooter.redfar(),
                        intake.on(),
                        pew.set(),
                        helper.forward(),
                        angle.redfar()
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
        Actions.runBlocking(new SequentialAction(
                drive.actionBuilder(beginPose)
                        .strafeToLinearHeading(new Vector2d(-56, -20),Math.toRadians(-24))//Firing position
                        .build(),
                new SleepAction(1.6)
        ));

        for (int i=0; i<3; i++){// Fires the pre-loaded balls
            Actions.runBlocking(new SequentialAction(
                    helper.off(),
                    intake.off(),
                    new SleepAction(0.2),
                    pew.launch(),
                    new SleepAction(0.2),
                    pew.set(),
                    new SleepAction(0.1),
                    helper.forward(),
                    intake.firein(),
                    new SleepAction(0.65)
            ));
        }
        Actions.runBlocking(new SequentialAction(//goes to pickup the 2nd set of balls
                intake.on(),
                drive.actionBuilder(new Pose2d(-57,-20,Math.toRadians(-23)))
                        .strafeToLinearHeading(new Vector2d(-25,-30),Math.toRadians(-90))
                        .strafeTo(new Vector2d(-25,-65), new TranslationalVelConstraint(25))
                        .strafeToLinearHeading(new Vector2d(-54, -20), Math.toRadians(-24))// Goes back to the firing position
                        .build()
        ));


        for (int i=0; i<3; i++){// Fires the 2nd set of balls
            Actions.runBlocking(new SequentialAction(
                    helper.off(),
                    intake.off(),
                    new SleepAction(0.2),
                    pew.launch(),
                    new SleepAction(0.2),
                    pew.set(),
                    new SleepAction(0.1),
                    helper.forward(),
                    intake.firein(),
                    new SleepAction(0.65)
            ));
        }
        Actions.runBlocking(new SequentialAction(// Goes to pickup the 3rd set of balls
                intake.on(),
                drive.actionBuilder(new Pose2d(-52, -18, Math.toRadians(-25)))
                        .strafeToLinearHeading(new Vector2d(-3,-40), Math.toRadians(-93))
                        .strafeTo(new Vector2d(-3, -70), new TranslationalVelConstraint(30))
                        .strafeToLinearHeading(new Vector2d(-54, -5), Math.toRadians(-24), new TranslationalVelConstraint(25))
                        .build()
        ));

        for (int i=0; i<3; i++){// Fires the last set of balls (3rd)
            Actions.runBlocking(new SequentialAction(
                    helper.off(),
                    intake.off(),
                    new SleepAction(0.2),
                    pew.launch(),
                    new SleepAction(0.2),
                    pew.set(),
                    new SleepAction(0.1),
                    helper.forward(),
                    intake.firein(),
                    new SleepAction(0.65)
            ));
        }
        drive.actionBuilder(new Pose2d(-55, -20, Math.toRadians(-24)))
                .strafeToLinearHeading(new Vector2d(-48, -18), Math.toRadians(-23))
                .build();
}
    }
