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
    private final int CAM_WIDTH = 320;
    private final int CAM_HEIGHT = 240;

    int tagx;
    int tagy;
    int tagw;
    int tagh;
    int tagid;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(48, 48, Math.toRadians(45));

        Shooter shooter = new Shooter(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Pew pew = new Pew(hardwareMap);
        Helper helper = new Helper(hardwareMap);
        SequentialAction init = new SequentialAction(
                shooter.stop(),
                intake.off(),
                pew.set(),
                helper.off()
        );



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

        HuskyLens.Block[] blocks = camq.blocks();
        telemetry.addData("Block count", blocks.length);
        if (blocks.length > 0) {
            for (int i = 0; i < blocks.length; i++) {
                telemetry.addData("Block", blocks[i].toString());
            }
            tagx = blocks[0].x;
            tagy = blocks[0].y;
            tagw = blocks[0].width;
            tagh = blocks[0].height;
            tagid = blocks[0].id;
        } else {
            tagx = -1;
            tagy = -1;
            tagw = -1;
            tagh = -1;
            tagid = -1;
        }

        waitForStart();

        Actions.runBlocking(init);
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
