package frc.robot.subsystems.vision

import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants
import lib.near
import org.littletonrobotics.junction.Logger
import org.photonvision.EstimatedRobotPose
import org.photonvision.PhotonPoseEstimator
import org.photonvision.targeting.PhotonTrackedTarget
import java.util.*
import java.util.function.Consumer
import java.util.function.Supplier
import kotlin.jvm.optionals.getOrDefault
import kotlin.jvm.optionals.getOrNull
import kotlin.math.abs

class AprilTagVision(poseSupplier: Supplier<Pose2d>) : SubsystemBase() {
    private val layout: AprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo)

    private val io: VisionIO = when (Constants.RobotConstants.mode) {
        Constants.RobotConstants.Mode.REAL -> VisionIOReal("tags")
        Constants.RobotConstants.Mode.SIM -> VisionIOSim("tags", poseSupplier, layout)
        Constants.RobotConstants.Mode.REPLAY -> object : VisionIO {}
    }

    val inputs: VisionIO.VisionInputs = VisionIO.VisionInputs()

    var pose: EstimatedRobotPose? = null

    private val estimator: PhotonPoseEstimator = PhotonPoseEstimator(
        Constants.VisionConstants.aprilTagField,
        PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        Constants.VisionConstants.robotToCam,
    )

    fun updateOdometryCommand(poseConsumer: Consumer<EstimatedRobotPose>): Command {
        return this.run {
            if(pose == null) return@run
            if(pose!!.estimatedPose.translation.x < 0.0 || pose!!.estimatedPose.translation.x > layout.fieldLength) return@run
            if(pose!!.estimatedPose.translation.y < 0.0 || pose!!.estimatedPose.translation.y > layout.fieldWidth) return@run
            if(abs(pose!!.estimatedPose.translation.z) > 0.1) return@run

            poseConsumer.accept(pose!!)
            println("Updated Odometry")
        }
    }

    override fun periodic() {
        io.updateInputs(inputs)

        pose = estimator.update(inputs.latestResult).getOrNull()

        Logger.processInputs("vision/Pose Estimation", inputs)

        if(pose != null) {
            Logger.recordOutput("vision/Estimated Pose", Pose3d.struct, pose!!.estimatedPose)
            Logger.recordOutput("vision/Pose Present", true)
        } else {
            Logger.recordOutput("vision/Pose Present", false)
        }

        val targets: Array<Pose3d> = inputs.latestResult.targets.map { layout.getTagPose(it.fiducialId).get() }.toTypedArray()

        Logger.recordOutput("vision/Targets", Pose3d.struct, *targets)
    }
}
