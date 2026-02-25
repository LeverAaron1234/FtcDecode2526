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
        full(0.74),
        bluefar(0.84),
        redfar(0.87),
        medium(0.60),
        low(0.55),
        off(0.0)

    }

    var shooterState = Shooter.off

    private val shooter = hardwareMap.get(DcMotorEx::class.java, "launcher")


    private val power = 1.0

    var scoringArmOffset = 0 //offset used to reset the arm positions mid-match
    var targetPower = 0.0

    init {
        shooter.mode = DcMotor.RunMode.RUN_USING_ENCODER
        shooter.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        shooter.direction = DcMotorSimple.Direction.FORWARD
        shooter.targetPosition = 0


        shooter.setVelocityPIDFCoefficients(
            DriveConstants.p,
            DriveConstants.i,
            DriveConstants.d,
            DriveConstants.f
        )
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
                targetPower = state.pwr

                shooter.velocity = targetPower*2800
                shooterState = state

                initialized = true
            }
            packet.put("SHOOTER Speed", shooter.velocity*60/28)
            packet.put("SHOOTER Current Position",shooter.currentPosition)
            packet.put("SHOOTER Target Power", targetPower*2800)
            packet.put("SHOOTER Current Power", shooter.power)

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

    fun full(): Action = SetState(Shooter.full)
    fun bluefar(): Action = SetState(Shooter.bluefar)
    fun redfar(): Action = SetState(Shooter.redfar)
    fun medium(): Action = SetState(Shooter.medium)
    fun low(): Action = SetState(Shooter.low)
    fun stop(): Action = SetState(Shooter.off)

}