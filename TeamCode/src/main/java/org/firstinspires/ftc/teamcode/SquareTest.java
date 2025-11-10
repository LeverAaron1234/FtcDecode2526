package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

@TeleOp
public final class SquareTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(0, 0, 0);

        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();

        Actions.runBlocking(new SequentialAction(
            drive.actionBuilder(beginPose)
                    .strafeTo(new Vector2d(0,48))
                    .waitSeconds(1)
                    .strafeTo(new Vector2d(0,0))
                    .build(),
            drive.actionBuilder(new Pose2d(0,48,0))
                    .lineToX(48)
                    .strafeTo(new Vector2d(48,0))
                    .build()
            )
        );


    }
}
