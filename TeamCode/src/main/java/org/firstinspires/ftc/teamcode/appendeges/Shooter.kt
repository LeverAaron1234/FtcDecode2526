package org.firstinspires.ftc.teamcode.appendeges

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import org.firstinspires.ftc.teamcode.DriveConstants
import kotlin.math.sqrt


class Shooter(hardwareMap: HardwareMap) {

    /**
     * @param position the position of the scoringArm in that state, -1 means we don't currently
     * know the position of the scoringArm
     */


    private val shooter1 = hardwareMap.get(DcMotorEx::class.java, "launcher")
    private val shooter2 = hardwareMap.get(DcMotorEx::class.java, "launcher2")


    private val power = 1.0

    var scoringArmOffset = 0 //offset used to reset the arm positions mid-match
    var targetPower = 0.0

    init {
        shooter1.mode = DcMotor.RunMode.RUN_USING_ENCODER
        shooter1.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        shooter1.direction = DcMotorSimple.Direction.FORWARD
        shooter1.targetPosition = 0
        shooter2.mode = DcMotor.RunMode.RUN_USING_ENCODER
        shooter2.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        shooter2.direction = DcMotorSimple.Direction.FORWARD
        shooter2.targetPosition = 0



        shooter1.setVelocityPIDFCoefficients(
            DriveConstants.p,
            DriveConstants.i,
            DriveConstants.d,
            DriveConstants.f
        )
        shooter2.setVelocityPIDFCoefficients(
            DriveConstants.p,
            DriveConstants.i,
            DriveConstants.d,
            DriveConstants.f
        )
        shooter1.velocity = power
        shooter2.velocity = power
    }

    /**
     * Start: sets the arm state and position;
     * IsFinished: the arm has reached the state's position
     *
     * @param state the state (and associated position) to set the arm to
     */
    inner class SetState(private val state: Double) : Action {
        private var initialized = false

        @Suppress("PARAMETER_NAME_CHANGED_ON_OVERRIDE")
        override fun run(packet: TelemetryPacket): Boolean {
            if (!initialized) {
                targetPower = state

                shooter1.velocity = (targetPower*2800)/2
                shooter2.velocity = (targetPower*2800)/2

                initialized = true
            }
            packet.put("SHOOTER1 Speed", shooter1.velocity*60/28)
            packet.put("SHOOTER2 Speed", -shooter2.velocity*60/28)
            //packet.put("SHOOTER Current Position",shooter.currentPosition)
            packet.put("SHOOTER Target Power", targetPower*2800)
            packet.put("SHOOTER1 Current Power", shooter1.power)
            packet.put("SHOOTER2 Current Power", shooter2.power)

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

    fun full(): Action = SetState(0.74)
    fun fuller(): Action = SetState(0.75)
    fun medium(): Action = SetState(0.60)
    fun low(): Action = SetState(0.55)
    fun stop(): Action = SetState(0.0)
    fun varshooter(spd: Double): Action = SetState(spd)
    // Reset PID so that I can tune.
    fun resetPID(p:Double,i:Double,d:Double) {
        shooter1.setVelocityPIDFCoefficients(p,i,d,0.0)
        shooter2.setVelocityPIDFCoefficients(p,i,d,0.0)
    }
    fun getPID(): PIDFCoefficients {
        return shooter1.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER)
    }

}