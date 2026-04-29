package org.firstinspires.ftc.teamcode.appendeges

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap


class Intake2(hardwareMap: HardwareMap) {


    enum class Intake2(val pwr: Double) {
        on(1.0),
        fire(0.0),
        Neutral(0.0)

    }

    var IntakeState = Intake2.Neutral

    /**
     * Get an instance of the extra intake motor on the robot.
     */
    private val intake = hardwareMap.get(DcMotor::class.java, "intake2")


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
     * Inner class to work with RoadRunner.
     * @param state The value you want to go to. Range 0.0-1.0
     * Returns `false` so that RoadRunner doesn't try to run it twice.
     */
    inner class SetState(private val state: Intake2) : Action {
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
     * Functions used in code.
     * Function `varangle` is used to set any value, for debug purposes
     */
    fun on(): Action = SetState(Intake2.on)
    fun firein(): Action = SetState(Intake2.fire)
    fun off(): Action = SetState(Intake2.Neutral)

}