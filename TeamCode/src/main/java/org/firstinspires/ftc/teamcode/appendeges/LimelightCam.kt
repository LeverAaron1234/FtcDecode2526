package org.firstinspires.ftc.teamcode.appendeges

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.hardware.HardwareMap


class LimelightCam(hardwareMap: HardwareMap) {

    /**
     * @param position the position of the scoringArm in that state, -1 means we don't currently
     * know the position of the scoringArm
     */

    enum class Camera(val pipe: Int) {
        Goal(0),
        Obelisk(1)

    }
    //ROCKS AND STICKS, I CAST FIX!!
    var pipeline = Camera.Goal

    private val camq = hardwareMap.get(Limelight3A::class.java, "limelight")
    private var result = camq.latestResult
    private var initialzed = false


    private var tagx = 0.0
    private var tagy = 0.0
    private var tagArea = -1.0
    private var tagid = -1


    init {
        camq.pipelineSwitch((pipeline.pipe))
        camq.start()
    }

    /**
     * Start: sets the arm state and position;
     * IsFinished: the arm has reached the state's position
     *
     * @param state the state (and associated position) to set the arm to
     */
    inner class SetState(private val state: Camera) : Action {
        private var initialized = false

        @Suppress("PARAMETER_NAME_CHANGED_ON_OVERRIDE")
        override fun run(packet: TelemetryPacket): Boolean {
            if (!initialized) {
                camq.pipelineSwitch(pipeline.pipe)
                initialzed = true
            }
            result = camq.latestResult
            if (!(result.isValid && result != null)) {
                tagx = 0.0
                tagy = 0.0
                tagid = 0
                tagArea = -1.0
            } else {
                tagx = result.tx
                tagy = result.ty
                tagid = result.fiducialResults[0].fiducialId
                tagArea = result.ta
            }
            packet.put("TagID of tag 0", tagid)
            packet.put("X offset", tagx)
            packet.put("Y offset", tagy)
            packet.put("Area of screen", tagArea)
            packet.put("All tag data", result.fiducialResults.toString())
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


    fun update(): Action = SetState(pipeline)

    fun switchPipeline(pipe: Int): Boolean {
        pipeline = Camera.entries[pipe]
        return camq.pipelineSwitch(pipe)
    }

    fun getTagid(): Int {
        return tagid
    }

    fun getTagArea(): Double {
        return tagArea
    }

    fun getTagx(): Double {
        return tagx
    }

    fun getTagy(): Double {
        return tagy
    }


}