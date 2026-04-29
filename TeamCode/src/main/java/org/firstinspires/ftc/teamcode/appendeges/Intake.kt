package org.firstinspires.ftc.teamcode.appendeges

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap


class Intake(hardwareMap: HardwareMap) {


    enum class Intake(val pwr: Double) {
        on(1.0),
        fire(0.0),
        invert(-0.4),
        Neutral(0.0)

    }

    var IntakeState = Intake.Neutral

    /**
     * Get an instance of the intake motor on the robot.
     */
    private val intake = hardwareMap.get(DcMotor::class.java, "intake")


    private val power = 0.0

    var scoringArmOffset = 0 //offset used to reset the arm positions mid-match
    var targetPower = 0.0

    init {
        intake.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        intake.direction = DcMotorSimple.Direction.FORWARD
        intake.power = 0.0
        intake.targetPosition = 0
        intake.power = power
    }

    /**
     * Inner class to work with RoadRunner.
     * @param state The value you want to go to. Range 0.0-1.0
     * Returns `false` so that RoadRunner doesn't try to run it twice.
     */
    inner class SetState(private val state: Intake) : Action {
        private var initialized = false

        @Suppress("PARAMETER_NAME_CHANGED_ON_OVERRIDE")
        override fun run(packet: TelemetryPacket): Boolean {
            if (!initialized) {
                targetPower = state.pwr
                intake.power = targetPower
                initialized = true
            }
            intake.currentPosition
            packet.put("Intake Target Power", targetPower)
            packet.put("Intake Current Power", intake.power)
            return false
        }
    }

    /**
     * Functions used in code.
     * Function `varangle` is used to set any value, for debug purposes
     */
    fun on(): Action = SetState(Intake.on)
    fun firein(): Action = SetState(Intake.fire)
    fun off(): Action = SetState(Intake.Neutral)
    fun back(): Action = SetState(Intake.invert)
    fun getPos(): Int {
        return intake.currentPosition
    }

}