package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.appendeges.Helper;
import org.firstinspires.ftc.teamcode.appendeges.Intake;
import org.firstinspires.ftc.teamcode.appendeges.Pew;
import org.firstinspires.ftc.teamcode.appendeges.Shooter;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

import java.util.concurrent.TimeUnit;

@TeleOp
public final class SquareTest extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    private HuskyLens camq = null;
    private final int READ_PERIOD = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(0, 0, 0);

        Shooter shooter = new Shooter(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Pew pew = new Pew(hardwareMap);
        Helper helper = new Helper(hardwareMap);

        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        camq = hardwareMap.get(HuskyLens.class, "camq");

        Deadline rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);
        rateLimit.expire();
        if (!camq.knock()) {
            telemetry.addData(">>", "Problem communicating with " + camq.getDeviceName());
        } else {
            telemetry.addData(">>", "Press start to continue");
        }

        camq.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

        waitForStart();

        Actions.runBlocking(new SequentialAction(
                shooter.medium(),
                intake.on(),
                pew.set(),
                helper.forward()
        ));
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
