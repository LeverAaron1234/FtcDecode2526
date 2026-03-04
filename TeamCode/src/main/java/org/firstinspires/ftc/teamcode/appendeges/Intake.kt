package org.firstinspires.ftc.teamcode.appendeges

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap


class Intake(hardwareMap: HardwareMap) {

    /**
     * @param position the position of the scoringArm in that state, -1 means we don't currently
     * know the position of the scoringArm
     */

    enum class Intake(val pwr: Double) {
        on(1.0),
        fire(0.0),
        invert(-0.4),
        Neutral(0.0)

    }

    var IntakeState = Intake.Neutral

    private val intake = hardwareMap.get(DcMotor::class.java, "intake")


    private val power = 1.0

    var scoringArmOffset = 0 //offset used to reset the arm positions mid-match
    var targetPower = 0.0

    init {
        intake.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        intake.direction = DcMotorSimple.Direction.FORWARD
        intake.power = 0.0
        intake.targetPosition = 0
        intake.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        intake.power = power
    }

    /**
     * Start: sets the arm state and position;
     * IsFinished: the arm has reached the state's position
     *
     * @param state the state (and associated position) to set the arm to
     */
    inner class SetState(private val state: Intake) : Action {
        private var initialized = false

        @Suppress("PARAMETER_NAME_CHANGED_ON_OVERRIDE")
        override fun run(packet: TelemetryPacket): Boolean {
            if (!initialized) {
                targetPower = state.pwr.toDouble()
                intake.power = targetPower
                IntakeState = state
                initialized = true
            }
            intake.currentPosition
            packet.put("Target Power", targetPower)
            packet.put("Current Power", intake.power)
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

    fun on(): Action = SetState(Intake.on)
    fun firein(): Action = SetState(Intake.fire)
    fun off(): Action = SetState(Intake.Neutral)
    fun back(): Action = SetState(Intake.invert)

}