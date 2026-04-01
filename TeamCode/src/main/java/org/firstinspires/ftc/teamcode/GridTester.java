package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Autonomous
public final class GridTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(0, 0, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        telemetry.update();
        waitForStart();
        Actions.runBlocking(new SequentialAction(
                drive.actionBuilder(new Pose2d(0,0, Math.toRadians(0)))
                        .strafeToSplineHeading(new Vector2d(50, 50), Math.toRadians(0))
                        .build()
        ));//cat
        telemetry.addLine("Positive, Positive");
        telemetry.update();
    }// cat
}// cat