package org.firstinspires.ftc.teamcode.appendeges

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.DriveConstants


class Shooter(hardwareMap: HardwareMap) {

    /**
     * @param position the position of the scoringArm in that state, -1 means we don't currently
     * know the position of the scoringArm
     */

    enum class Shooter(val pwr: Double) {
        full(0.78),
        medium(0.63),
        low(0.56),
        off(0.0)

    }

    var shooterState = Shooter.off

    private val shooter = hardwareMap.get(DcMotorEx::class.java, "launcher")


    private val power = 1.0



    init {
        shooter.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        shooter.direction = DcMotorSimple.Direction.FORWARD
        shooter.power = 0.0
        shooter.targetPosition = 0


        shooter.setVelocityPIDFCoefficients(
            DriveConstants.p,
            DriveConstants.i,
            DriveConstants.d,
            DriveConstants.f
        )
        //shooter.mode = DcMotor.RunMode.RUN_USING_ENCODER
        shooter.velocity = power
    }

    /**
     * Start: sets the arm state and position;
     * IsFinished: the arm has reached the state's position
     *
     * @param state the state (and associated position) to set the arm to
     */
    inner class SetState(private val state: Shooter) : Action {
        private var initialized = false

        @Suppress("PARAMETER_NAME_CHANGED_ON_OVERRIDE")
        override fun run(packet: TelemetryPacket): Boolean {
            if (!initialized) {
                shooter.velocity = state.pwr
                shooterState = state

                initialized = true
            }

            packet.put("Target Power", state.pwr)
            packet.put("Current Power", shooter.power)
            packet.put("Current State", shooterState)
            return false
        }
    }

    /**
      Make usable functions for the code.
      These set the velocity to, for example,
      "full" or "medium" which are set above, in ```Shooter(val pwr...)```.
      */

    fun full(): Action = SetState(Shooter.full)
    fun medium(): Action = SetState(Shooter.medium)
    fun low(): Action = SetState(Shooter.low)
    fun stop(): Action = SetState(Shooter.off)

}