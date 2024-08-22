package frc.robot.subsystems.vision

import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import org.photonvision.targeting.PhotonPipelineResult

interface VisionIO {
    class VisionInputs : LoggableInputs {
        var latestResult: PhotonPipelineResult = PhotonPipelineResult()

        override fun toLog(table: LogTable) {
            table.put("Latest Result", latestResult)
        }

        override fun fromLog(table: LogTable) {
            table.get("Latest Results", PhotonPipelineResult()).let { latestResult = it }
        }
    }

    fun updateInputs(inputs: VisionInputs) {}


}