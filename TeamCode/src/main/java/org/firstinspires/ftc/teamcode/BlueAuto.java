package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.appendeges.Angle;
import org.firstinspires.ftc.teamcode.appendeges.Helper;
import org.firstinspires.ftc.teamcode.appendeges.Intake;
import org.firstinspires.ftc.teamcode.appendeges.Pew;
import org.firstinspires.ftc.teamcode.appendeges.Shooter;

import java.util.concurrent.TimeUnit;

@Autonomous
public final class BlueAuto extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(-63, 24, Math.toRadians(0));

        Shooter shooter = new Shooter(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Pew pew = new Pew(hardwareMap);
        Angle angle = new Angle(hardwareMap);
        Actions.runBlocking(pew.set());
        Helper helper = new Helper(hardwareMap);


        SequentialAction init = new SequentialAction(
                shooter.medium(),
                intake.on(),
                pew.set(),
                helper.forward(),
                angle.middle()
        );
        SequentialAction fire = new SequentialAction(
                helper.backward(),
                intake.off(),
                new SleepAction(0.1),
                pew.launch(),
                new SleepAction(0.5),
                pew.set(),
                new SleepAction(4),
                helper.forward(),
                intake.on()
        );


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

        Actions.runBlocking(
                new ParallelAction(
                        shooter.full(),
                        intake.on(),
                        pew.set(),
                        helper.forward(),
                        angle.far()
                )
        );


        telemetry.update();

        //if (tagid == 21) { // GPP
        Actions.runBlocking(new SequentialAction(
                drive.actionBuilder(beginPose)
                        .strafeTo(new Vector2d(-60, 20))
                        .turnTo(Math.toRadians(17))
                        .build(),
                new SleepAction(2)
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
                angle.middle(),
                shooter.medium()
        ));
        Actions.runBlocking(new SequentialAction(
                drive.actionBuilder(new Pose2d(-60,28,Math.toRadians(15)))
                        .strafeTo(new Vector2d(-50.5,30))
                        .turnTo(Math.toRadians(90))
                        .strafeTo(new Vector2d(-50.5, 50))
                        .build(),
                drive.actionBuilder(new Pose2d(-50.5, 50, Math.toRadians(80)))
                        .strafeTo(new Vector2d(-50.5,30))
                        .strafeTo(new Vector2d(-20,30))
                        .turnTo(Math.toRadians(45))
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
        /*} else if (tagid == 22){ // PGP
            Actions.runBlocking(new SequentialAction(
                    drive.actionBuilder(new Pose2d(-24,29,Math.toRadians(-90)))
                            .strafeTo(new Vector2d(-24,58))
                            .build()
            ));
        } else if (tagid == 23){ // PPG
            Actions.runBlocking(new SequentialAction(
                    drive.actionBuilder(new Pose2d(12,29,Math.toRadians(-90)))
                            .strafeTo(new Vector2d(12,58))
                            .build()
            ));
        }*/



    }


}
