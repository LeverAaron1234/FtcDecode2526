package org.firstinspires.ftc.teamcode.appendeges

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap


class Helper(hardwareMap: HardwareMap) {


    enum class Helper(val pwr: Double) {
        forward(1.0),
        Neutral(0.0),
        backward(-1.0)

    }

    var HelperState = Helper.Neutral

    /**
     * Get an instance of the extra intake motor on the robot.
     */
    private val helper = hardwareMap.get(DcMotorEx::class.java, "helper")


    private val power = 1.0

    var scoringArmOffset = 0 //offset used to reset the arm positions mid-match
    var targetPower = 0.0

    init {
        helper.power = 0.0
        helper.power = power
    }

    /**
     * Inner class to work with RoadRunner.
     * @param state The value you want to go to. Range 0.0-1.0
     * Returns `false` so that RoadRunner doesn't try to run it twice.
     */
    inner class SetState(private val state: Helper) : Action {
        private var initialized = false

        @Suppress("PARAMETER_NAME_CHANGED_ON_OVERRIDE")
        override fun run(packet: TelemetryPacket): Boolean {
            if (!initialized) {
                targetPower = state.pwr.toDouble()
                helper.power = targetPower
                HelperState = state
                initialized = true
            }
            packet.put("Target Power", targetPower)
            packet.put("Current Power", helper.power)
            return false
        }
    }

    /**
     * Functions used in code.
     * Function `varangle` is used to set any value, for debug purposes
     */
    fun forward(): Action = SetState(Helper.forward)
    fun off(): Action = SetState(Helper.Neutral)
    fun backward(): Action = SetState(Helper.backward)

}