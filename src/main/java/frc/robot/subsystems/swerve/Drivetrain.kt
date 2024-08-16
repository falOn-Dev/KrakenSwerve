package frc.robot.subsystems.swerve

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger
import java.util.function.DoubleSupplier

class Drivetrain(val io: DrivetrainIO) : SubsystemBase() {
    val inputs: DrivetrainIO.DrivetrainInputsCollection = DrivetrainIO.DrivetrainInputsCollection()

    val speeds: ChassisSpeeds
        get() {
            return ChassisSpeeds(
                inputs.drivetrainInputs.measuredVXMetersPerSec,
                inputs.drivetrainInputs.measuredVYMetersPerSec,
                inputs.drivetrainInputs.measuredAngularVelocityRadPerSec,
            )
        }

    val heading: Double
        get() = inputs.gyroInputs.yawDeg

    val pose: Pose2d
        get() = inputs.drivetrainInputs.robotPose

    fun driveCommand(
        forward: DoubleSupplier,
        strafe: DoubleSupplier,
        rotation: DoubleSupplier,
        fieldOriented: Boolean,
        isOpenLoop: Boolean,
    ): Command {
        return this.run {
            if (fieldOriented) {
                io.driveFieldRelative(forward.asDouble, strafe.asDouble, rotation.asDouble, isOpenLoop)
            } else {
                io.driveRobotRelative(forward.asDouble, strafe.asDouble, rotation.asDouble, isOpenLoop)
            }
        }
    }

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("swerve/Drivetrain", inputs.drivetrainInputs)
        Logger.processInputs("swerve/Gyro", inputs.gyroInputs)
        inputs.moduleInputs.forEachIndexed { index, moduleInputs ->
            Logger.processInputs("swerve/Drivetrain/Module$index", moduleInputs)
        }
    }
}
