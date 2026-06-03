package org.firstinspires.ftc.teamcode.appendages

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.lerp
import com.qualcomm.hardware.limelightvision.LLResult
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.teamcode.DriveConstants
import org.firstinspires.ftc.teamcode.MecanumDrive
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sign
import kotlin.math.sin


class Spin(hardwareMap: HardwareMap) {

    /**
     * @param position the position of the scoringArm in that state, -1 means we don't currently
     * know the position of the scoringArm
     */
    enum class SpinPos(val pwr: Double) {
        Init(-1.0),
        locked(1.0),
        unlocked(0.0)

    }


    private val spin = hardwareMap.get(Servo::class.java, "spin")
    private val encoder = hardwareMap.get(DcMotor::class.java, "intake")
    private val encoderOffset = 125
    private val RADIANS_TO_DEGREES = 180/Math.PI
    private var kP = DriveConstants.spinP
    private var kI = DriveConstants.spinI
    private var kD = DriveConstants.spinD
    private var kIgain = 0.0
    private var camkIgain = 0.0
    private val goalX = 0.0
    private var lastError = 0.0
    private var camLastError = 0.0
    private val deAcellSpeed = 0.01;
    private val angleTolerance = 0.1
    private val MAX_POWER = 1.0
    private var power = 0.0
    private var switched = false
    private val posTolorance = 1.0
    private val creepThreshhold = 8
    private val creepSpeed = 0.08
    private var i = 0.0
    private var j = 0
    var initialized = false
    var lock = true // TODO: change to false to turn off lock

    private val timer = ElapsedTime()


    /**
     * Start: sets the arm state and position;
     * IsFinished: the arm has reached the state's position
     *
     * @param state the state (and associated position) to set the arm to
     */
    inner class SetState(private val state: SpinPos) : Action {
        private var initialized = false

        @Suppress("PARAMETER_NAME_CHANGED_ON_OVERRIDE")
        override fun run(packet: TelemetryPacket): Boolean {
            if (!initialized) {
                packet.put("Power", power)

                spin.position = state.pwr
                initialized = true
            }



            return false
        }
    }


    fun resetTimer() {
        timer.reset()
    }

    /**
     * Reset the encoder
     * */
    fun resetEncoder() {
        encoder.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        encoder.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }

    /**
     * Points the turret at the goal
     * @param drive The current drive, used to get the localizer position
     * @param isRedGoal A flag to change the target goal
     * @param turretOffset The offset of the
     * */
    fun odomUpdate(drive: MecanumDrive, isRedGoal: Boolean, turretOffset:Int, lock: Boolean): MutableList<Double?> {
        switched = true
        val ticksPerDegree = 66.928104575163
        val targetX = if (isRedGoal) -65.0 else -70.0
        val targetY = if (isRedGoal) 65.0 else -60.0
        // TODO: MAKE TURRET SERVO MODE INSTEAD OF CONTINUOUS
        val returnList: MutableList<Double?> = ArrayList()

//      Field heading as a complex number
        val realAng = drive.localizer.pose.heading.real
        val imagAng = drive.localizer.pose.heading.imag
        var currentHeading = atan2(imagAng,realAng)
//      Make the currentHeading go from -180 to 180, to 0 to 360 (Will make everything else not work as intended.)
//        currentHeading = if (currentHeading<0) currentHeading + 2*Math.PI else currentHeading

//      Field position in x,y
        val posX = drive.localizer.pose.position.x + (1.74 * sin(currentHeading*(1/RADIANS_TO_DEGREES)))
        val posY = drive.localizer.pose.position.y + (-1.74 * cos(currentHeading*(1/RADIANS_TO_DEGREES)))


//      encoder pos in degrees + current heading
        val currentAngle = ((-encoder.currentPosition) / ticksPerDegree - encoderOffset) + (currentHeading*RADIANS_TO_DEGREES)

//      Angle of current position (robot) to target (goal)
        val targetXdiff = (targetX - posX)
        val targetYdiff = (targetY - posY)
        val targetHeading = atan2(-targetYdiff,-targetXdiff)
        val targetAngle = (targetHeading)*RADIANS_TO_DEGREES

        //


        val test = invlerp(-125.0,90.0,min(max(targetAngle-(currentHeading*RADIANS_TO_DEGREES),-125.0),90.0))
        spin.position = lerp(spin.position,test,0.1)
        // target angle -> 0.0-1.0
        // angles 225-82
        // 307 degrees
        // -90 - 127

        // 0.32 - 0.67
        // -90  - 90
        lerp(-125.0,90.0,invlerp(0.0,0.67,0.5))

        lerp(currentHeading,targetHeading,0.5)
        //

        returnList.add(posX)
        returnList.add(posY)
        returnList.add(targetX)
        returnList.add(targetY)
        returnList.add(encoder.currentPosition/ticksPerDegree - encoderOffset)
        returnList.add(currentHeading*RADIANS_TO_DEGREES)
        returnList.add(currentAngle)
        returnList.add(targetAngle)
        returnList.add(spin.position)

        return returnList

    }

    fun camUpdate(result: LLResult, leftPressed: Boolean, rightPressed: Boolean, lock: Boolean): MutableList<Double?> {
        kP = DriveConstants.camSpinP
        kI = DriveConstants.camSpinI
        kD = DriveConstants.camSpinD
        val deltaTime = timer.seconds()
        timer.reset()

        val returnList: MutableList<Double?> = ArrayList<Double?>()

        if (!result.isValid) {
            camLastError = 0.0
            returnList.add(DriveConstants.spinP)
            returnList.add(DriveConstants.spinI)
            returnList.add(DriveConstants.spinD)
            returnList.add(power)
            return returnList
        }

        val error = goalX - result.tx
        val pTerm = error * kP

        camkIgain += error * deltaTime
        val iterm = camkIgain * kI

        var dterm = 0.0
        if (deltaTime > 0) {
            dterm = ((error - lastError) / deltaTime) * kD
        }

        if (abs(error) < angleTolerance) {
            power = 0.0
            camkIgain = 0.0
        } else {
            power = Range.clip(pTerm + iterm + dterm, -MAX_POWER, MAX_POWER)
        }

        if ((leftPressed && power < 0) || (rightPressed && power > 0)) {
            power = 0.0
        }

        //spin.power = if (!switched) power else 0.0

        if (!switched) {
            i = 0.0
        } else {
            if (i > 0.1) {switched = false}
            i += deltaTime
        }

        camLastError = error


        returnList.add(DriveConstants.spinP)
        returnList.add(DriveConstants.spinI)
        returnList.add(DriveConstants.spinD)
        returnList.add(power)

        return returnList
    }

    fun lock(): Action = SetState(SpinPos.locked)
    fun unlock(): Action = SetState(SpinPos.unlocked)
    fun Init(): Action = SetState(SpinPos.Init)
    fun lerp(a:Double, b:Double, t:Double): Double {
        return a+(t*(b-a))
    }
    fun invlerp(a:Double, b:Double, value:Double): Double {
        return (value-a)/(b-a)
    }
}