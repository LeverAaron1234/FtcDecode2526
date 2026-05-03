package org.firstinspires.ftc.teamcode.appendages

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap


class Pew(hardwareMap: HardwareMap) {

    enum class PewPos(val position: Double) {
        invert(-0.4),
        set(0.0),
        launch(1.0)

    }


    private val pew = hardwareMap.get(DcMotorEx::class.java, "pew")




    var targetPosition = 0.0



    inner class SetState(private val state: PewPos) : Action {
        private var initialized = false

        @Suppress("PARAMETER_NAME_CHANGED_ON_OVERRIDE")
        override fun run(packet: TelemetryPacket): Boolean {
            if (!initialized) {
                targetPosition = state.position.toDouble()
                pew.power = targetPosition

                initialized = true
            }
            pew.power


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

    fun back(): Action = SetState(PewPos.invert)
    fun set(): Action = SetState(PewPos.set)
    fun launch(): Action = SetState(PewPos.launch)

}