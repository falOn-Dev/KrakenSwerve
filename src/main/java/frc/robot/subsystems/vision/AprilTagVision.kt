package frc.robot.subsystems.vision

import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants
import org.littletonrobotics.junction.Logger
import org.photonvision.EstimatedRobotPose
import org.photonvision.PhotonPoseEstimator
import java.util.*

class AprilTagVision : SubsystemBase() {
    private val io: VisionIO = when (Constants.RobotConstants.mode) {
        Constants.RobotConstants.Mode.REAL -> VisionIOReal("tags")
        else -> object : VisionIO {}
    }

    val inputs: VisionIO.VisionInputs = VisionIO.VisionInputs()

    val pose: Optional<EstimatedRobotPose>
        get() = estimator.update(inputs.latestResult)

    private val estimator: PhotonPoseEstimator = PhotonPoseEstimator(
        Constants.VisionConstants.aprilTagField,
        PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        Constants.VisionConstants.robotToCam,
    )

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("vision/Pose Estimation", inputs)

        if (pose.isPresent) Logger.recordOutput("vision/Estimated Pose", pose.get().estimatedPose)
        inputs.latestResult.targets.forEachIndexed { index, target ->
            Logger.recordOutput("vision/Target $index", target)
        }
    }
}
