package frc.robot.subsystems.vision

import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.geometry.Pose2d
import frc.robot.Constants
import org.photonvision.PhotonCamera
import org.photonvision.simulation.PhotonCameraSim
import org.photonvision.simulation.SimCameraProperties
import org.photonvision.simulation.VisionSystemSim
import java.util.function.Supplier

class VisionIOSim(private val name: String, private val poseSupplier: Supplier<Pose2d>, layout: AprilTagFieldLayout) : VisionIO {
    private val visionSim: VisionSystemSim = VisionSystemSim("visionSim")
    private val cam: PhotonCamera = PhotonCamera(name)
    private val camProp: SimCameraProperties = SimCameraProperties()
    private val camSim: PhotonCameraSim = PhotonCameraSim(cam, camProp)

    init {
        visionSim.addAprilTags(layout)
        visionSim.addCamera(camSim, Constants.VisionConstants.robotToCam)
    }

    override fun updateInputs(inputs: VisionIO.VisionInputs) {
        visionSim.update(poseSupplier.get())

        inputs.latestResult = camSim.camera.latestResult
    }
}