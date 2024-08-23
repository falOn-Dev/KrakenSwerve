package frc.robot.subsystems.vision

import org.photonvision.PhotonCamera

class VisionIOReal(name: String) : VisionIO {
    val camera: PhotonCamera = PhotonCamera(name)

    override fun updateInputs(inputs: VisionIO.VisionInputs) {
        inputs.latestResult = camera.latestResult
    }
}
