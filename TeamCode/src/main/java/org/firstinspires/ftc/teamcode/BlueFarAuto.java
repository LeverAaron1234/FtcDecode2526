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
        Actions.runBlocking(new SequentialAction(
                drive.actionBuilder(beginPose)
                        .strafeTo(new Vector2d(-60, 20))
                        .turnTo(Math.toRadians(17))
                        .build(),
                new SleepAction(2),
                shooter.full()
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
                drive.actionBuilder(new Pose2d(-60,20,Math.toRadians(17)))
                        .strafeTo(new Vector2d(-50,20))
                        .turnTo(Math.toRadians(90))
                        .strafeTo(new Vector2d(-50,50), new TranslationalVelConstraint(10.0))
                        .build(),
                new SleepAction(0.1),
                intake.off(),
                drive.actionBuilder(new Pose2d(-50,50,90))
                        .strafeTo(new Vector2d(-50,20))
                        .turnTo(17)
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
