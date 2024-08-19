package frc.robot.subsystems

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants
import frc.robot.subsystems.swerve.gyro.GyroIO
import frc.robot.subsystems.swerve.gyro.GyroIOPigeon2
import frc.robot.subsystems.swerve.module.SwerveModule
import org.littletonrobotics.junction.Logger
import java.util.function.DoubleSupplier

class Drivetrain(
    private val drivetrainConstants: SwerveDrivetrainConstants,
    vararg val moduleConstants: SwerveModuleConstants
) : SubsystemBase() {
    private val gyro: GyroIO = when (Constants.RobotConstants.mode) {
        Constants.RobotConstants.Mode.REAL -> GyroIOPigeon2(drivetrainConstants)
        Constants.RobotConstants.Mode.SIM -> object : GyroIO {}
        Constants.RobotConstants.Mode.REPLAY -> object : GyroIO {}
    }

    val gyroInputs: GyroIO.GyroInputs = GyroIO.GyroInputs()

    private val modules: Array<SwerveModule> = moduleConstants.map { SwerveModule(it) }.toTypedArray() // FL, FR, BL, BR

    private val kinematics: SwerveDriveKinematics = SwerveDriveKinematics(*getModuleTranslations())

    private val poseEstimator: SwerveDrivePoseEstimator = SwerveDrivePoseEstimator(
        kinematics,
        gyroInputs.yawDegrees,
        getModulePositions(),
        Pose2d()
    )


    private fun getModuleTranslations(): Array<Translation2d> {
        val translations: Array<Translation2d> = emptyArray()

        modules.forEachIndexed { index, module ->
            translations[index] = Translation2d(module.config.LocationX, module.config.LocationY)
        }

        return translations
    }

    private fun getModulePositions(): Array<SwerveModulePosition> {
        val positions: Array<SwerveModulePosition> = emptyArray()

        modules.forEachIndexed { index, module ->
            positions[index] = module.modulePosition
        }

        return positions
    }

    fun resetHeading() {
        gyro.setYaw(0.0)
    }

    fun driveCommand(forwards: DoubleSupplier, strafe: DoubleSupplier, rotation: DoubleSupplier) {
        val swerveModuleStates = kinematics.toSwerveModuleStates(
            ChassisSpeeds.fromRobotRelativeSpeeds(
                forwards.asDouble * 3.5,
                strafe.asDouble * 3.5,
                rotation.asDouble * ( Math.PI ),
                gyroInputs.yawDegrees
            )
        )

        modules.forEachIndexed { index, module ->
            module.apply(swerveModuleStates[index])
        }
    }

    override fun periodic() {
        gyro.updateInputs(gyroInputs)
        Logger.processInputs("swerve/gyro", gyroInputs)
        modules.forEachIndexed { index, it ->
            it.updateInputs()
            Logger.processInputs("swerve/module[${index + 1}]", it.inputs)
        }
        poseEstimator.update(gyroInputs.yawDegrees, getModulePositions())

        Logger.recordOutput("swerve/pose", poseEstimator.estimatedPosition)
    }
}