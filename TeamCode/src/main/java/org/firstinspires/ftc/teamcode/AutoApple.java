package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

@Disabled
// Has a broken leg (and doesn't work)
@Autonomous
public class AutoApple extends LinearOpMode {
  private final ElapsedTime runtime = new ElapsedTime();

  private DcMotor frontLeftDrive = null;
  private DcMotor backLeftDrive = null;
  private DcMotor frontRightDrive = null;
  private DcMotor backRightDrive = null;
  private DcMotor wheeel = null;
  private HuskyLens camq = null;
  GoBildaPinpointDriver pinpoint = null;



  private final int READ_PERIOD = 1;


  //timer
  private final ElapsedTime timer = new ElapsedTime();





  @Override
  public void runOpMode() {
    // Defining motors
    frontLeftDrive = hardwareMap.get(DcMotor.class, "leftFront");
    backLeftDrive = hardwareMap.get(DcMotor.class, "leftBack");
    frontRightDrive = hardwareMap.get(DcMotor.class, "rightFront");
    backRightDrive = hardwareMap.get(DcMotor.class, "rightBack");
    //wheeel = hardwareMap.get(DcMotor.class, "wheeel");
    camq = hardwareMap.get(HuskyLens.class, "camq");
    pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
    pinpoint.setOffsets(-84.0, -168.0, DistanceUnit.MM);
    pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
    pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
            GoBildaPinpointDriver.EncoderDirection.FORWARD);
    pinpoint.resetPosAndIMU();
    pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
    int slow = 1;


    // Motor directions
    frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
    backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
    frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
    backRightDrive.setDirection(DcMotor.Direction.REVERSE);
    //wheeel.setDirection(DcMotorSimple.Direction.FORWARD);

    /*
     * This sample rate limits the reads solely to allow a user time to observe
     * what is happening on the Driver Station telemetry.  Typical applications
     * would not likely rate limit.
     */

    Deadline rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);

    /*
     * Immediately expire so that the first time through we'll do the read.
     */
    rateLimit.expire();

    /*
     * Basic check to see if the device is alive and communicating.  This is not
     * technically necessary here as the HuskyLens class does this in its
     * doInitialization() method which is called when the device is pulled out of
     * the hardware map.  However, sometimes it's unclear why a device reports as
     * failing on initialization.  In the case of this device, it's because the
     * call to knock() failed.
     */
    if (!camq.knock()) {
      telemetry.addData(">>", "Problem communicating with " + camq.getDeviceName());
    } else {
      telemetry.addData(">>", "Press start to continue");
    }

    camq.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

    waitForStart();
    runtime.reset();
    move(10,-10,0.5);

  }
  void move(int x,int y,double power) {
    if (pinpoint.getPosX(DistanceUnit.INCH) < x && pinpoint.getPosY(DistanceUnit.INCH) < y) {
      // right & forwards
      while (pinpoint.getPosX(DistanceUnit.INCH) < x && pinpoint.getPosY(DistanceUnit.INCH) < y) {
        if (!opModeIsActive()){break;}

        double a = (pinpoint.getPosX(DistanceUnit.INCH) - x);
        double b = (pinpoint.getPosY(DistanceUnit.INCH) - y);
        double dir;
        if (b == 0) {
          if (a < 0) {dir = -90;}
          else {dir = 90;}
        } else {
          if (b < 0){dir = 180 + Math.atan(a/b);}
          else {dir = Math.atan(a/b);}
        }
        double sin = Math.sin(dir - Math.PI/4);
        double cos = Math.cos(dir - Math.PI/4);
        //double max = Math.max(Math.abs(sin),Math.abs(cos));
        telemetry.addData("sin",sin);
        telemetry.addData("cos",cos);
        //telemetry.addData("max",max);

        frontLeftDrive.setPower(power*cos);
        frontRightDrive.setPower(power*sin);
        backLeftDrive.setPower(power*sin);
        backRightDrive.setPower(power*cos);
        telemetry.addData("FrontLeft",frontLeftDrive.getPower());
        telemetry.addData("FrontRight",frontRightDrive.getPower());
        telemetry.addData("BackLeft",frontLeftDrive.getPower());
        telemetry.addData("BackRight",frontRightDrive.getPower());
        telemetry.update();
        pinpoint.update();
      }
    } else if (pinpoint.getPosX(DistanceUnit.INCH) > x && pinpoint.getPosY(DistanceUnit.INCH) > y) {
      // left & backwards
      while (pinpoint.getPosX(DistanceUnit.INCH) > x && pinpoint.getPosY(DistanceUnit.INCH) > y) {
        if (!opModeIsActive()){break;}

        double a = (pinpoint.getPosX(DistanceUnit.INCH) - x);
        double b = (pinpoint.getPosY(DistanceUnit.INCH) - y);
        double dir;
        if (b == 0) {
          if (a < 0){dir = -90;}
          else {dir = 90;}
        } else {
          if (b < 0){dir = 180 + Math.atan(a/b);}
          else {dir = Math.atan(a/b);}
        }
        double sin = Math.sin(dir - Math.PI/4);
        double cos = Math.cos(dir - Math.PI/4);
        //double max = Math.max(Math.abs(sin),Math.abs(cos));
        telemetry.addData("sin",sin);
        telemetry.addData("cos",cos);
        //telemetry.addData("max",max);

        frontLeftDrive.setPower(power*cos);
        frontRightDrive.setPower(power*sin);
        backLeftDrive.setPower(power*sin);
        backRightDrive.setPower(power*cos);
        telemetry.addData("FrontLeft",frontLeftDrive.getPower());
        telemetry.addData("FrontRight",frontRightDrive.getPower());
        telemetry.addData("BackLeft",frontLeftDrive.getPower());
        telemetry.addData("BackRight",frontRightDrive.getPower());
        telemetry.update();
        pinpoint.update();
      }
    } else if (pinpoint.getPosX(DistanceUnit.INCH) < x && pinpoint.getPosY(DistanceUnit.INCH) > y) {
      // right & backwards
      while (pinpoint.getPosX(DistanceUnit.INCH) < x && pinpoint.getPosY(DistanceUnit.INCH) > y) {
        if (!opModeIsActive()){break;}

        double a = (pinpoint.getPosX(DistanceUnit.INCH) - x);
        double b = (pinpoint.getPosY(DistanceUnit.INCH) - y);
        double dir;
        if (b == 0) {
          if (a < 0){dir = -90;}
          else {dir = 90;}
        } else {
          if (b < 0){dir = 180 + Math.atan(a/b);}
          else {dir = Math.atan(a/b);}
        }
        double sin = Math.sin(dir - Math.PI/4);
        double cos = Math.cos(dir - Math.PI/4);
        //double max = Math.max(Math.abs(sin),Math.abs(cos));
        telemetry.addData("sin",sin);
        telemetry.addData("cos",cos);
        //telemetry.addData("max",max);

        frontLeftDrive.setPower(power*cos);
        frontRightDrive.setPower(power*sin);
        backLeftDrive.setPower(power*sin);
        backRightDrive.setPower(power*cos);
        telemetry.addData("FrontLeft",frontLeftDrive.getPower());
        telemetry.addData("FrontRight",frontRightDrive.getPower());
        telemetry.addData("BackLeft",frontLeftDrive.getPower());
        telemetry.addData("BackRight",frontRightDrive.getPower());
        telemetry.update();
        pinpoint.update();
      }
    } else if (pinpoint.getPosX(DistanceUnit.INCH) > x && pinpoint.getPosY(DistanceUnit.INCH) < y) {
      // left & forwards
      while (pinpoint.getPosX(DistanceUnit.INCH) > x && pinpoint.getPosY(DistanceUnit.INCH) < y) {
        if (!opModeIsActive()){break;}

        double a = (pinpoint.getPosX(DistanceUnit.INCH) - x);
        double b = (pinpoint.getPosY(DistanceUnit.INCH) - y);
        double dir;
        if (b == 0) {
          if (a < 0){dir = -90;}
          else {dir = 90;}
        } else {
          if (b < 0){dir = 180 + Math.atan(a/b);}
          else {dir = Math.atan(a/b);}
        }
        double sin = Math.sin(dir - Math.PI/4);
        double cos = Math.cos(dir - Math.PI/4);
        //double max = Math.max(Math.abs(sin),Math.abs(cos));
        telemetry.addData("sin",sin);
        telemetry.addData("cos",cos);
        //telemetry.addData("max",max);

        frontLeftDrive.setPower(power*cos);
        frontRightDrive.setPower(power*sin);
        backLeftDrive.setPower(power*sin);
        backRightDrive.setPower(power*cos);
        telemetry.addData("FrontLeft",frontLeftDrive.getPower());
        telemetry.addData("FrontRight",frontRightDrive.getPower());
        telemetry.addData("BackLeft",frontLeftDrive.getPower());
        telemetry.addData("BackRight",frontRightDrive.getPower());
        telemetry.update();
        pinpoint.update();
      }
    }
    frontLeftDrive.setPower(0);
    frontRightDrive.setPower(0);
    backLeftDrive.setPower(0);
    backRightDrive.setPower(0);
  }
}
