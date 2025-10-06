/*
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.appendeges.ArmSwing;
import org.firstinspires.ftc.teamcode.appendeges.Reach;
import org.firstinspires.ftc.teamcode.appendeges.Spin;
import org.firstinspires.ftc.teamcode.appendeges.Teeth;
import org.firstinspires.ftc.teamcode.appendeges.Wrist;
import org.firstinspires.ftc.teamcode.drive.PinpointDrive;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.Arrays;

import kotlin.Suppress;

@Autonomous

public class AutoPurple extends LinearOpMode {
    private static final Logger log = LoggerFactory.getLogger(AutoPurple.class);

    public void runOpMode() {
        Pose2d beginPose = new Pose2d(9, -64.5, 0);

        ArmSwing armSwing = new ArmSwing(hardwareMap);
        Teeth teeth = new Teeth(hardwareMap);
        Reach reach = new Reach(hardwareMap);
        Spin spin = new Spin(hardwareMap);
        Wrist wrist = new Wrist(hardwareMap);




        PinpointDrive drive = new PinpointDrive(hardwareMap, beginPose);


        Actions.runBlocking(new ParallelAction(armSwing.init(),spin.offset()));



        waitForStart();


        // Speed constraints //
        VelConstraint baseVelConstraint = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(100.0),
                new AngularVelConstraint(Math.PI / 2)
        ));
        AccelConstraint baseAccelConstraint = new ProfileAccelConstraint(-30.0, 50.0);

*/
 /*/////////////////////////////////////////////////////////////////////////////////////////////////
 Start of Auto RIGHT Start of Auto RIGHT Start of Auto RIGHT Start of Auto RIGHT Start of Auto RIGHT
 /////////////////////////////////////////////////////////////////////////////////////////////////*//*
        telemetry.addData("Odometry x", drive.getPose().position.x);
        telemetry.addData("Odometry y", drive.getPose().position.y);
        telemetry.addData("Odo Pos x", drive.pinpoint.getPosition().getX(DistanceUnit.INCH));
        telemetry.addData("Odo Pos y", drive.pinpoint.getPosition().getY(DistanceUnit.INCH));

        Actions.runBlocking(new ParallelAction(
                armSwing.throughBars2(),
                teeth.closed()));
        sleep(200);
        Actions.runBlocking(wrist.num1());

        telemetry.update();


        Actions.runBlocking(new SequentialAction(
                // Reach out to the top bar
                new ParallelAction(
                drive.actionBuilder(beginPose)
                        .strafeTo(new Vector2d(9,-36))
                        .build(),
                armSwing.throughBars2()),
                drive.actionBuilder(new Pose2d(9,-37,0))
                        .waitSeconds(0.1)
                        .build(),
                armSwing.throughBars3(),
                drive.actionBuilder(new Pose2d(9,-37,0))
                        .strafeTo(new Vector2d(9,-46))
                        .build()

                //reach.middle()

        ));
        sleep(400);
        Actions.runBlocking(reach.inn());
        Actions.runBlocking(new SequentialAction(
                new ParallelAction(
                armSwing.neutral(),
                teeth.open(),
                wrist.offset()),
                drive.actionBuilder(new Pose2d(9,-46,0))
                        .strafeTo(new Vector2d(9,-50))
                        .waitSeconds(0.2)
                        .build()
        ));
        //Go to push blocks into observation zone
        Actions.runBlocking(new SequentialAction(
                new ParallelAction(
                armSwing.neutral(),
                wrist.offset()),
                drive.actionBuilder(new Pose2d(9,-50,Math.toRadians(90)))
                        .strafeTo(new Vector2d(36,-50))
                        .waitSeconds(0.1)
                        .strafeTo(new Vector2d(36,-16))
                        .build()
        ));
        sleep(100);
        Actions.runBlocking(
                //Block #1
                drive.actionBuilder(new Pose2d(45,-16,Math.toRadians(90)))
                        .strafeTo(new Vector2d(45,-55))
                        .build()
        );
        sleep(100);
        Actions.runBlocking(
                //Block #2
                drive.actionBuilder(new Pose2d(45,-55,Math.toRadians(90)))
                        .strafeTo(new Vector2d(45,-16))
                        .waitSeconds(0.10)
                        .strafeTo(new Vector2d(57,-16))
                        .waitSeconds(0.10)
                        .strafeTo(new Vector2d(57,-55))
                        .build()
        );
        sleep(100);
      /*Actions.runBlocking(
                //block #3
                drive.actionBuilder(new Pose2d(59,-55,Math.toRadians(270)))
                        .strafeTo(new Vector2d(59,-12))
                        .waitSeconds(0.1)
                        .strafeTo(new Vector2d(63,-12))
                        .waitSeconds(0.1)
                        .strafeTo(new Vector2d(63,-55))
                        .build()
        );
        sleep(250);*//*
        Actions.runBlocking(new SequentialAction(
                new ParallelAction(
                armSwing.wall(),
                wrist.straight(),
                teeth.open()),
                drive.actionBuilder(new Pose2d(48,-40,Math.toRadians(270)))
                        .waitSeconds(0.25)
                        //go out so that the person can grab blocks
                        .turnTo(Math.toRadians(180))
                        .strafeTo(new Vector2d(48,-55))
                        .build()
        ));
        //grab clipped blocks
        Actions.runBlocking(new SequentialAction(
                //block #1 to bar
                new ParallelAction(
                armSwing.wall(),
                spin.offset(),
                wrist.straight()),
                drive.actionBuilder(new Pose2d(48,-53.5,Math.toRadians(180)))
                        .waitSeconds(0.1)
                        .build(),
                wrist.straight(),
                teeth.closed()
        ));
        sleep(200);
        Actions.runBlocking(new SequentialAction(
                //put on top bar
                teeth.closed(),
                armSwing.throughBars1(),
                drive.actionBuilder(new Pose2d(48,-53.5,Math.toRadians(180)))
                        .waitSeconds(0.1)
                        .turnTo(0)
                        .strafeTo(new Vector2d(2,-50))
                        .waitSeconds(0.1)
                        .build()));
        sleep(100);
        Actions.runBlocking(new SequentialAction(
                drive.actionBuilder(new Pose2d(2,-50,0))
                        .strafeTo(new Vector2d(2,-37))
                        .waitSeconds(0.1)
                        .build(),
                armSwing.throughBars0(),
                drive.actionBuilder(new Pose2d(2,-37,0))
                        .strafeTo(new Vector2d(2,-46))
                        .build(),
                teeth.open()
        ));
        sleep(100);
        Actions.runBlocking(new SequentialAction(
                //block #2 to bar
                armSwing.wall(),
                drive.actionBuilder(new Pose2d(48,-53.5,Math.toRadians(0)))
                        .turnTo(Math.toRadians(180))
                        .build(),
                wrist.straight(),
                teeth.open()
        ));
        sleep(250);
        Actions.runBlocking(new SequentialAction(
                //put on top bar
                teeth.closed(),
                armSwing.throughBars1(),
                drive.actionBuilder(new Pose2d(48,-53.5,Math.toRadians(180)))
                        .waitSeconds(0.1)
                        .turnTo(0)
                        .strafeTo(new Vector2d(2,-50))
                        .waitSeconds(0.1)
                        .strafeTo(new Vector2d(2,-37))
                        .waitSeconds(0.1)
                        .strafeTo(new Vector2d(2,-41))
                        .build()
        ));
      /*Actions.runBlocking(new SequentialAction(
                //block #3 to bar
                drive.actionBuilder(new Pose2d(48,-60,Math.toRadians(180)))
                        .waitSeconds(0.1)
                        .build(),
                teeth.open(),
                armSwing.pickup()
        ));
        sleep(250);
        Actions.runBlocking(new SequentialAction(
                //put on top bar
                teeth.closed(),
                drive.actionBuilder(new Pose2d(48,-60,Math.toRadians(180)))
                        .waitSeconds(0.1)
                        .turnTo(0)
                        .strafeTo(new Vector2d(4,-36))
                        .build(),
                armSwing.throughBars1()
        ));*//*
        Actions.runBlocking(new SequentialAction(
                //park robot, ending setup
                drive.actionBuilder(new Pose2d(4,-40,0))
                        .waitSeconds(1)
                        .strafeTo(new Vector2d(60,-64))
                        .build(),
                wrist.score(),
                armSwing.pickup(),
                reach.inn()
        ));


        sleep(10000); // So the robot doesn't destroy itself.


//
//        .strafeTo(new Vector2d(48,-40))
//                .waitSeconds(0.1)
//                .build()
//        );
//        //grab clipped blocks
//        Actions.runBlocking(new SequentialAction(
//                //block #1 to bar
//                drive.actionBuilder(new Pose2d(48,-60,Math.toRadians(180)))
//                        .waitSeconds(0.1)
//                        .build(),
//                teeth.open(),
//                armSwing.pickup()
//        ));


    }
}*/