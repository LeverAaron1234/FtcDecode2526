package org.firstinspires.ftc.teamcode.appendeges

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.google.gson.internal.bind.JsonAdapterAnnotationTypeAdapterFactory
import com.qualcomm.hardware.limelightvision.LLResult
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.ElapsedTime
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.teamcode.DriveConstants
import org.firstinspires.ftc.teamcode.MecanumDrive
import kotlin.math.abs
import kotlin.math.atan2


class Spin(hardwareMap: HardwareMap) {

    /**
     * @param position the position of the scoringArm in that state, -1 means we don't currently
     * know the position of the scoringArm
     */
    enum class SpinPos(val position: Double) {
        locked(1.0),
        unlocked(0.0)

    }


    private val spin = hardwareMap.get(CRServo::class.java, "spin")
    private var kP = DriveConstants.spinP
    private var kI = DriveConstants.spinI
    private var kD = DriveConstants.spinD
    private var kIgain = 0.0
    private val goalX = 0.0
    private var lastError = 0.0
    private val deAcellSpeed = 0.01;
    private val angleTolerance = 5.0
    private val MAX_POWER = 0.6
    private var power = 0.0
    private var move_left = true
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
                packet.put("Search Dir", move_left)

                if (state == SpinPos.locked) {
                    spin.power = 0.0
                }
                initialized = true
            }



            return false
        }
    }

    /**
     * manually changes the position of the scoringArm (typically with a joystick)
     *
     * @param input the percent speed (-1 to 1) normalized by delta time (the time between each loop)
     */

    /**
     * Only use in the collect position; used to reset the positions of the arm; should be called
     * alongside a collect action
     */

    fun resetTimer() {
        timer.reset()
    }

    fun odomUpdate(drive: MecanumDrive, redGoal: Boolean): MutableList<Double?> {
        val targetX = -72
        val targetY = if (redGoal) 72 else -72

        val posX = drive.localizer.pose.position.x
        val posY = drive.localizer.pose.position.y

        val targetAngle = atan2(targetY - posY, targetX - posX)

        val returnList: MutableList<Double?> = ArrayList<Double?>()

        //TODO: Change spin to a position
        //spin.setPose(targetAngle/Math.PI)

        returnList.add(posX)
        returnList.add(posY)
        returnList.add(targetAngle)
        returnList.add(spin.power)

        return returnList

    }

    fun update(result: LLResult, leftPressed: Boolean, rightPressed: Boolean, lock: Boolean): MutableList<Double?> {
        kP = DriveConstants.spinP
        kI = DriveConstants.spinI
        kD = DriveConstants.spinD
        val deltaTime = timer.seconds()
        timer.reset()

        val returnList: MutableList<Double?> = ArrayList<Double?>()

        if ((leftPressed && power < 0) || (rightPressed && power > 0)) {
            power = 0.0
        }

        if (!result.isValid) {
            if (leftPressed) {
                move_left = false
            }
            if (rightPressed) {
                move_left = true
            }

            if (!lock) {
                spin.power = (0.3 * (if (move_left) -1 else 1))
            } else {
                power -= deAcellSpeed
                spin.power = (power)
            }
            lastError = 0.0
            returnList.add(power)
            returnList.add(DriveConstants.spinP)
            returnList.add(DriveConstants.spinI)
            returnList.add(DriveConstants.spinD)
            return returnList
        }

        val error = goalX - result.tx
        val pTerm = error * DriveConstants.spinP

        kIgain += error * deltaTime
        val iterm = kIgain * DriveConstants.spinI

        var dterm = 0.0
        if (deltaTime > 0) {
            dterm = ((error - lastError) / deltaTime) * DriveConstants.spinD
        }

        if (abs(error) < angleTolerance) {
            power = 0.0
            kIgain = 0.0
        } else {
            power = Range.clip(pTerm + iterm + dterm, -MAX_POWER, MAX_POWER)
        }


        if (power == 0.0) {
            spin.power = (0.001)
        } else {
            spin.power = (power)
        }
        lastError = error

        returnList.add(power)
        returnList.add(DriveConstants.spinP)
        returnList.add(DriveConstants.spinI)
        returnList.add(DriveConstants.spinD)

        return returnList
    }

    fun lock(): Action = SetState(SpinPos.locked)
    fun unlock(): Action = SetState(SpinPos.unlocked)
}