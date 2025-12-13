package org.firstinspires.ftc.teamcode.appendeges

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.clamp
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo


class Angle(hardwareMap: HardwareMap) {

    /**
     * @param position the position of the scoringArm in that state, -1 means we don't currently
     * know the position of the scoringArm
     */
    enum class AnglePos(val position: Double) {
        Far(0.75),
        Middle(0.28),
        Close(0.1)

    }


    private val angle = hardwareMap.get(Servo::class.java, "angle")


    var targetPosition = 0.28


    /**
     * Start: sets the arm state and position;
     * IsFinished: the arm has reached the state's position
     *
     * @param state the state (and associated position) to set the arm to
     */
    inner class SetState(private val state: AnglePos) : Action {
        private var initialized = false

        @Suppress("PARAMETER_NAME_CHANGED_ON_OVERRIDE")
        override fun run(packet: TelemetryPacket): Boolean {
            if (!initialized) {
                targetPosition = state.position.toDouble()
                angle.position = clamp(targetPosition, 0.1,1.0)

                initialized = true
            }
            packet.put("Angle Positon",angle.position)

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

    fun far(): Action = SetState(AnglePos.Far)
    fun middle(): Action = SetState(AnglePos.Middle)
    fun down(): Action = SetState(AnglePos.Close)

}