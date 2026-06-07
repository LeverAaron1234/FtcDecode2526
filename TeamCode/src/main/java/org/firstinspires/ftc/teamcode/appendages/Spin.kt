package org.firstinspires.ftc.teamcode.appendages

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.clamp
import com.qualcomm.hardware.limelightvision.LLResult
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareMap
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


    private val spin = hardwareMap.get(CRServo::class.java, "spin")
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
    private val posTolorance = 0.1
    private val creepThreshhold = 2
    private val creepSpeed = 0.02
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

                spin.power = state.pwr
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

    private fun getTurretOffset(isRedGoal: Boolean, encoderDegrees: Double): Double {
        return 0.00;
        return if (isRedGoal) {
            if (encoderDegrees < -180) 0.0
            else -4.0 + if (encoderDegrees > -50) -2.0 else 0.0
        } else {
            (if (encoderDegrees < -180) 10.0 else 0.0) +
                    if (encoderDegrees > -50) 5.0 else 0.0
        }
    }

    /**
     * Points the turret at the goal
     * @param drive The current drive, used to get the localizer position
     * @param isRedGoal A flag to change the target goal
     * @param turretOffset The offset of the
     * */
    fun odomUpdate(drive: MecanumDrive, isRedGoal: Boolean, turretOffset:Int, leftPressed: Boolean, rightPressed: Boolean, lock: Boolean): MutableList<Double?> {
        switched = true
        //val ticksPerDegree = 66.928104575163  // this is correct, putting math in so easier to understand
        val ticksPerDegree = 8192.0 * (150.0 / 51.0) / 360.0  //51 teeth, 150 teeth (turret) /360 degrees * tpi
        val targetX = if (isRedGoal) -65.0 else -65.0     //Why is the goal in a different position, is this the problem or a symptom
        val targetY = if (isRedGoal) 65.0 else -65.0
        // TODO: MAKE TURRET SERVO MODE INSTEAD OF CONTINUOUS
        val returnList: MutableList<Double?> = ArrayList()

//      Field heading as a complex number
        val realAng = drive.localizer.pose.heading.real
        val imagAng = drive.localizer.pose.heading.imag
        var currentHeading = atan2(imagAng,realAng)
//      Make the currentHeading go from -180 to 180, to 0 to 360 (Will make everything else not work as intended.)
//        currentHeading = if (currentHeading<0) currentHeading + 2*Math.PI else currentHeading

//      Mr. Whelan found these were incorrect and made the starting position to be different on each side by about 3.8 inches or 1.74*2
//      Field position in x,y
//        val posX = drive.localizer.pose.position.x + (1.74 * sin(currentHeading*(1/RADIANS_TO_DEGREES)))
//        val posY = drive.localizer.pose.position.y + (-1.74 * cos(currentHeading*(1/RADIANS_TO_DEGREES)))
        val turretRadius = 1.74
        val posX = drive.localizer.pose.position.x + (turretRadius * sin(currentHeading))
        val posY = drive.localizer.pose.position.y + (if (isRedGoal) -turretRadius else turretRadius) * cos(currentHeading)

//      encoder pos in degrees + current heading
        val currentAngle = ((-encoder.currentPosition) / ticksPerDegree - encoderOffset) + (currentHeading*RADIANS_TO_DEGREES) + (turretOffset)


//      Angle of current position (robot) to target (goal)
        val targetXdiff = (targetX - posX)
        val targetYdiff = (targetY - posY)

        val encoderDegrees = encoder.currentPosition / ticksPerDegree
        val targetHeading = atan2(-targetYdiff, -targetXdiff) + Math.toRadians(getTurretOffset(isRedGoal, encoderDegrees))
        // LEVI HARD TO READ val targetHeading = atan2(-targetYdiff,-targetXdiff) + (Math.PI/180 * (if (isRedGoal) (if (encoder.currentPosition/ticksPerDegree < -180) 0 else -4 + if (encoder.currentPosition/ticksPerDegree > -50) -2 else 0) else ((if (encoder.currentPosition/ticksPerDegree < -180) 10 else 0) + if (encoder.currentPosition/ticksPerDegree > -50) 5 /* <- Causes problems */ else 0)))
        // if (encoder.currentPosition/ticksPerDegree < -180) 10 else 0 works on blue
        // if (encoder.currentPosition/ticksPerDegree > -50) -2 else 0 works on red
        // if (isRedGoal) (if (encoder.currentPosition/ticksPerDegree < -180) 10 else 0 + if (encoder.currentPosition/ticksPerDegree > -50) -2 else 0) else (if (encoder.currentPosition/ticksPerDegree < -180) 10 else 0 + if (encoder.currentPosition/ticksPerDegree > -50) 0 else 0)
        val targetAngle = (targetHeading)*RADIANS_TO_DEGREES

        if (leftPressed) {
            j += 1
            if (j > 3) {
                initialized = true
                resetEncoder()
                j = 0
            }
        } else {
            j = 0
        }

        if (lock || !initialized) {
            if (!initialized) {
                spin.power = -0.5
            } else {
                spin.power = 0.001 // to make the turret not move
                //kIgain = 0.0
            }
            if ((leftPressed && power < 0) || (rightPressed && power > 0)) {
                power = 0.0
            }
            returnList.add(posX)
            returnList.add(posY)
            returnList.add(targetX)
            returnList.add(targetY)
            //returnList.add(encoder.currentPosition/ticksPerDegree)
            returnList.add(encoder.currentPosition/1.0)
            returnList.add(currentAngle)
            returnList.add(currentHeading*RADIANS_TO_DEGREES)
            returnList.add(targetAngle)
            returnList.add(spin.power)
            return returnList
        }

        // === CLEAN PID - No Creep - Better Final Accuracy ===
        kP = DriveConstants.spinP
        kI = DriveConstants.spinI
        val kD = DriveConstants.spinD
        val kF = DriveConstants.spinFF

        val deltaTime = timer.seconds()
        timer.reset()
        val error = targetAngle - currentAngle

        val Pterm = error * kP

        // I Term - allows better final accuracy
        kIgain += error * deltaTime
        kIgain = clamp(kIgain, -0.45, 0.45)     // Slightly tighter windup protection

        val Iterm = kIgain * kI

        // D Term
        var Dterm = 0.0
        if (deltaTime > 0.001) {
            Dterm = ((error - lastError) / deltaTime) * kD
        }
        lastError = error

        // Feedforward
        val Fterm = if (error > 3.0) kF else if (error < -3.0) -kF else 0.0

        // Final power with small deadband for stability
        val finalPower = if (abs(error) < 0.8) {     // Tight deadband for accuracy
            0.0
        } else {
            Pterm + Iterm + Dterm + Fterm
        }

        power = Range.clip(finalPower, -0.85, 0.85)


        if ((leftPressed && power < 0) || (rightPressed && power > 0)) {
            power = 0.0
        }

        spin.power = power

        //}
        /*if ((leftPressed && power < 0) || (rightPressed && power > 0)) {
            power = 0.0
            spin.power = 0.0
        }*/

        returnList.add(posX)
        returnList.add(posY)
        returnList.add(targetX)
        returnList.add(targetY)
        returnList.add(encoder.currentPosition/ticksPerDegree)
        returnList.add(currentHeading*RADIANS_TO_DEGREES)
        returnList.add(currentAngle)
        returnList.add(targetAngle)
        returnList.add(spin.power)

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

        spin.power = if (!switched) power else 0.0

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
}