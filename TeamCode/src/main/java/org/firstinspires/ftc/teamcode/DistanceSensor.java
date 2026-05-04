package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.appendages.Angle;
import org.firstinspires.ftc.teamcode.appendages.Helper;
import org.firstinspires.ftc.teamcode.appendages.Intake;
import org.firstinspires.ftc.teamcode.appendages.LimelightCam;
import org.firstinspires.ftc.teamcode.appendages.Pew;
import org.firstinspires.ftc.teamcode.appendages.Shooter;

@Autonomous
public final class DistanceSensor extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(-63, 24, Math.toRadians(0));

        Shooter shooter = new Shooter(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Pew pew = new Pew(hardwareMap);
        Angle angle = new Angle(hardwareMap);
        Helper helper = new Helper(hardwareMap);
        LimelightCam camq = new LimelightCam(hardwareMap);

        Actions.runBlocking(new SequentialAction(
                pew.set(),
                intake.off(),
                angle.close(),
                helper.off()

        ));

        camq.setPipeline(LimelightCam.Camera.Obelisk);
        camq.switchPipeline(1); //Obelisk

        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);


        telemetry.update();
        Actions.runBlocking(shooter.full());
        waitForStart();


        while (opModeIsActive()) {
            Actions.runBlocking(camq.update());
            telemetry.addData("Distance to Tag",tagDist(camq.getTagArea()));
            telemetry.addData("Tag Area", camq.getTagArea());
            telemetry.addData("Tagx", camq.getTagx());
            telemetry.addData("Tagy", camq.getTagy());
            telemetry.addData("TagID", camq.getTagid());
            telemetry.update();
        }






    }

    public double tagDist(double ta) {
        double scale = 20226.48;
        return scale/ta;
    }
}
