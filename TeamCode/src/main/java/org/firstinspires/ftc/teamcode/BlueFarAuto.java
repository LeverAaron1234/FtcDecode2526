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
                        shooter.bluefar(),
                        intake.off(),
                        pew.set(),
                        helper.off(),
                        angle.bluefar()
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
                        .strafeToSplineHeading(new Vector2d(-60, 20), Math.toRadians(17))// This is the firing position for Blue Far
                        .build(),
                shooter.bluefar(),
                new SleepAction(1.65)
        ));

        for (int i=0; i<3; i++){//This is firing the pre-loaded/first set of balls
            Actions.runBlocking(new SequentialAction(
                    helper.off(),
                    intake.off(),
                    new SleepAction(0.55),
                    pew.launch(),
                    new SleepAction(0.1),
                    pew.set(),
                    new SleepAction(0.2),
                    helper.forward(),
                    intake.firein(),
                    new SleepAction((i<2)?0.6:0.5)
            ));
        }

        Actions.runBlocking(new SequentialAction(// This goes to and picks up the first row of balls
                intake.on(),
                drive.actionBuilder(new Pose2d(-60,20,Math.toRadians(17)))
                        .strafeToSplineHeading(new Vector2d(-53,20),Math.toRadians(90))
                        .strafeTo(new Vector2d(-53,40), new TranslationalVelConstraint(14))
                        .build(),
                new SleepAction(0.16),
                intake.off()
        ));

        Actions.runBlocking(new SequentialAction(// Go back to the firing position and get ready to fire the 2nd set of balls
                drive.actionBuilder(new Pose2d(-54,40,Math.toRadians(90)))
                        //.strafeTo(new Vector2d(-49,20))
                        .strafeToSplineHeading(new Vector2d(-60, 14),Math.toRadians(15.5))// Decemails defently won't break this. - The words spoken before it broke
                        .build()
        ));

        Actions.runBlocking(new SequentialAction(
                intake.off(),
                helper.off(),
                shooter.bluefar(),
                new SleepAction(1)
        ));

        for (int i=0; i<3; i++){// This is firing the second set of balls
            Actions.runBlocking(new SequentialAction(
                    helper.off(),
                    intake.off(),
                    new SleepAction((i==1)?0.45:(i==2)?0.5:0.55),
                    pew.launch(),
                    new SleepAction(0.2),
                    pew.set(),
                    new SleepAction(0.1),
                    helper.forward(),
                    intake.firein(),
                    new SleepAction(0.6)
            ));
        }


        Actions.runBlocking(new SequentialAction(// This is moving to pick up the 2nd set of balls
                intake.on(),
                drive.actionBuilder(new Pose2d(-50,6,Math.toRadians(18)))
                        .strafeTo(new Vector2d(-35, 20))
                        .turn(Math.toRadians(90))
                        // The code that goes to 0, 0, 0 made it run into the other robot that levi was coding in front of the blue goal
                        // It was at aproxamently ~29-34, ~24 so something went wrong
                        .build(),
                new SleepAction(0.1),
                intake.off()

        ));
        new SleepAction(5);
        for (int i=0; i<3; i++){// Firing the 3rd set of balls
            Actions.runBlocking(new SequentialAction(
                    helper.off(),
                    intake.off(),
                    new SleepAction(0.55),
                    pew.launch(),
                    new SleepAction(0.2),
                    pew.set(),
                    new SleepAction(0.1),
                    helper.forward(),
                    intake.firein(),
                    new SleepAction(0.6)
            ));
        }



    }

}
