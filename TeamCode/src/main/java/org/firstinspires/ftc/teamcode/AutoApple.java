package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

@Autonomous(name = "AutoAppple", preselectTeleOp = "Launcher")
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

  public void move(int x,int y,float power) {
    while () {

    }
  }


  @Override
  public void runOpMode() {
    // Defining motors
    frontLeftDrive = hardwareMap.get(DcMotor.class, "motorFL");
    backLeftDrive = hardwareMap.get(DcMotor.class, "motorBL");
    //frontRightDrive = hardwareMap.get(DcMotor.class, "motorFR");
    backRightDrive = hardwareMap.get(DcMotor.class, "motorBR");
    wheeel = hardwareMap.get(DcMotor.class, "wheeel");
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
    //frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
    backRightDrive.setDirection(DcMotor.Direction.REVERSE);
    wheeel.setDirection(DcMotorSimple.Direction.FORWARD);

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
  }
}
