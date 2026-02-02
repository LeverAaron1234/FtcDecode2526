package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.appendeges.Angle;
import org.firstinspires.ftc.teamcode.appendeges.Helper;
import org.firstinspires.ftc.teamcode.appendeges.Intake;
import org.firstinspires.ftc.teamcode.appendeges.LimelightCam;
import org.firstinspires.ftc.teamcode.appendeges.Pew;
import org.firstinspires.ftc.teamcode.appendeges.Shooter;

import java.util.concurrent.TimeUnit;

@Autonomous
public final class BlueFarAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(-63, 24, Math.toRadians(0));

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
                        intake.off(),
                        pew.set(),
                        helper.off(),
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
        Actions.runBlocking(new SequentialAction(
                drive.actionBuilder(beginPose)
                        .strafeToSplineHeading(new Vector2d(-60, 20), Math.toRadians(17))
                        .build(),
                shooter.full(),
                new SleepAction(1)
        ));

        for (int i=0; i<3; i++){
            Actions.runBlocking(new SequentialAction(
                    helper.off(),
                    intake.off(),
                    new SleepAction(0.5),
                    helper.backward(),
                    pew.launch(),
                    new SleepAction(0.2),
                    pew.set(),
                    new SleepAction(0.1),
                    helper.forward(),
                    intake.on(),
                    new SleepAction(0.6)
            ));
        }

        Actions.runBlocking(new SequentialAction(
                drive.actionBuilder(new Pose2d(-60,20,Math.toRadians(17)))
                        .strafeToSplineHeading(new Vector2d(-49,20),Math.toRadians(90))
                        .strafeTo(new Vector2d(-49,45), new TranslationalVelConstraint(12.5))
                        .build(),
                new SleepAction(0.1),
                intake.off()
        ));

        Actions.runBlocking(new SequentialAction(
                drive.actionBuilder(new Pose2d(-49,45,Math.toRadians(90)))
                        .strafeTo(new Vector2d(-49,20))
                        .strafeToSplineHeading(new Vector2d(-66, 20),Math.toRadians(17))
                        .build()
        ));

        Actions.runBlocking(new SequentialAction(
                intake.off(),
                helper.off(),
                shooter.full(),
                new SleepAction(1)
        ));

        for (int i=0; i<3; i++){
            Actions.runBlocking(new SequentialAction(
                    helper.off(),
                    intake.off(),
                    new SleepAction(0.5),
                    helper.backward(),
                    pew.launch(),
                    new SleepAction(0.2),
                    pew.set(),
                    new SleepAction(0.1),
                    helper.forward(),
                    intake.on(),
                    new SleepAction(0.6)
            ));
        }


        Actions.runBlocking(new SequentialAction(
                drive.actionBuilder(new Pose2d(-66,18,Math.toRadians(20)))
                        .strafeToSplineHeading(new Vector2d(-26,18), Math.toRadians(90))
                        .strafeTo(new Vector2d(-26, 40), new TranslationalVelConstraint(12.5))
                        .strafeToSplineHeading(new Vector2d(-64,20), Math.toRadians(17))
                        .build()

        ));

        for (int i=0; i<3; i++){
            Actions.runBlocking(new SequentialAction(
                    helper.off(),
                    intake.off(),
                    new SleepAction(0.5),
                    helper.backward(),
                    pew.launch(),
                    new SleepAction(0.2),
                    pew.set(),
                    new SleepAction(0.1),
                    helper.forward(),
                    intake.on(),
                    new SleepAction(0.6)
            ));
        }



    }

}
