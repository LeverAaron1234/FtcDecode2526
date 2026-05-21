package org.firstinspires.ftc.teamcode.appendages

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.clamp
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo


class Angle(hardwareMap: HardwareMap) {


    /**
     * Get an instance of the hood on the robot.
     */
    private val angle = hardwareMap.get(Servo::class.java, "angle")


    var targetPosition = 0.28


    /**
     * Inner class to work with RoadRunner.
     * @param state The value you want to go to. Range 0.0-1.0
     * Returns `false` so that RoadRunner doesn't try to run it twice.
     */
    inner class SetState(private val state: Double) : Action {
        private var initialized = false

        @Suppress("PARAMETER_NAME_CHANGED_ON_OVERRIDE")
        override fun run(packet: TelemetryPacket): Boolean {
            if (!initialized) {
                targetPosition = state
                angle.position = clamp(targetPosition, 0.0,1.0)

                initialized = true
            }
            packet.put("Angle Positon",angle.position)

            return false
        }
    }


    /**
     * Functions used in code.
     * Function `varangle()` is used to set any value, for debug purposes
     */
    fun far(): Action = SetState(0.48)
    fun middle(): Action = SetState(0.22)
    fun close(): Action = SetState(0.0)
    fun varangle(ang:Double): Action = SetState(ang)

}