package org.firstinspires.ftc.teamcode.appendeges

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.HardwareMap

//ROCKS AND STICKS, I CAST FIX!!
class Helper(hardwareMap: HardwareMap) {

    /**
     * @param position the position of the scoringArm in that state, -1 means we don't currently
     * know the position of the scoringArm
     */

    enum class Helper(val pwr: Double) {
        forward(1.0),
        Neutral(0.001),
        backward(-1.0)

    }

    var HelperState = Helper.Neutral

    private val helper = hardwareMap.get(CRServo::class.java, "helper")


    private val power = 1.0

    var scoringArmOffset = 0 //offset used to reset the arm positions mid-match
    var targetPower = 0.0

    init {
        helper.power = 0.0
        helper.power = power
    }

    /**
     * Start: sets the arm state and position;
     * IsFinished: the arm has reached the state's position
     *
     * @param state the state (and associated position) to set the arm to
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
     * manually changes the position of the scoringArm (typically with a joystick)
     *
     * @param input the percent speed (-1 to 1) normalized by delta time (the time between each loop)
     */

    /**
     * Only use in the collect position; used to reset the positions of the arm; should be called
     * alongside a collect action
     */

    fun forward(): Action = SetState(Helper.forward)
    fun off(): Action = SetState(Helper.Neutral)
    fun backward(): Action = SetState(Helper.backward)

}