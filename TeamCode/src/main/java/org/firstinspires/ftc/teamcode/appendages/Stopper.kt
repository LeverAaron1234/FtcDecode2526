package org.firstinspires.ftc.teamcode.appendages

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.clamp
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo


class Stopper(hardwareMap: HardwareMap) {

    /**
     * @param position the position of the scoringArm in that state, -1 means we don't currently
     * know the position of the scoringArm
     */
    enum class StopperPos(val position: Double) {
        In(0.5),
        Out(0.0)

    }


    private val stopper = hardwareMap.get(Servo::class.java, "stopper")


    var targetPosition = 0.0


    /**
     * Start: sets the arm state and position;
     * IsFinished: the arm has reached the state's position
     *
     * @param state the state (and associated position) to set the arm to
     */
    inner class SetState(private val state: StopperPos) : Action {
        private var initialized = false

        @Suppress("PARAMETER_NAME_CHANGED_ON_OVERRIDE")
        override fun run(packet: TelemetryPacket): Boolean {
            if (!initialized) {
                targetPosition = state.position.toDouble()
                stopper.position = clamp(targetPosition, 0.1,1.0)

                initialized = true
            }
            packet.put("Stopper Positon",stopper.position)

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

    fun In(): Action = SetState(StopperPos.In)
    fun Out(): Action = SetState(StopperPos.Out)

}