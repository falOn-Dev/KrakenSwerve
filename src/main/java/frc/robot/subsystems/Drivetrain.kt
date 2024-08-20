package frc.robot.subsystems

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants
import frc.robot.subsystems.swerve.TunerConstants
import frc.robot.subsystems.swerve.gyro.GyroIO
import frc.robot.subsystems.swerve.gyro.GyroIOPigeon2
import frc.robot.subsystems.swerve.gyro.GyroIOSim
import frc.robot.subsystems.swerve.module.SwerveModule
import org.littletonrobotics.junction.Logger
import java.util.function.BooleanSupplier
import java.util.function.DoubleSupplier

class Drivetrain(
    private val drivetrainConstants: SwerveDrivetrainConstants,
    vararg val moduleConstants: SwerveModuleConstants
) : SubsystemBase() {
    private val gyro: GyroIO = when (Constants.RobotConstants.mode) {
        Constants.RobotConstants.Mode.REAL -> GyroIOPigeon2(drivetrainConstants)
        Constants.RobotConstants.Mode.SIM -> GyroIOSim(this::currentSpeeds)
        Constants.RobotConstants.Mode.REPLAY -> object : GyroIO {}
    }

    val gyroInputs: GyroIO.GyroInputs = GyroIO.GyroInputs()

    private val modules: Array<SwerveModule> = moduleConstants.map { SwerveModule(it) }.toTypedArray() // FL, FR, BL, BR
    private val desiredStates: Array<SwerveModuleState> = arrayOf(
        SwerveModuleState(),
        SwerveModuleState(),
        SwerveModuleState(),
        SwerveModuleState(),
    )
    private val measuredStates: Array<SwerveModuleState> = arrayOf(
        SwerveModuleState(),
        SwerveModuleState(),
        SwerveModuleState(),
        SwerveModuleState(),
    )

    private var currentSpeeds: ChassisSpeeds = ChassisSpeeds()

    private val kinematics: SwerveDriveKinematics = SwerveDriveKinematics(*getModuleTranslations())

    private val poseEstimator: SwerveDrivePoseEstimator = SwerveDrivePoseEstimator(
        kinematics,
        gyroInputs.yawDegrees,
        getModulePositions(),
        Pose2d()
    )


    private fun getModuleTranslations(): Array<Translation2d> {
        val translations: Array<Translation2d> = arrayOf(
            Translation2d(),
            Translation2d(),
            Translation2d(),
            Translation2d(),
        )

        modules.forEachIndexed { index, module ->
            translations[index] = Translation2d(module.config.LocationX, module.config.LocationY)
        }

        return translations
    }

    private fun getModulePositions(): Array<SwerveModulePosition> {
        val positions: Array<SwerveModulePosition> = arrayOf(
            SwerveModulePosition(),
            SwerveModulePosition(),
            SwerveModulePosition(),
            SwerveModulePosition(),
        )

        modules.forEachIndexed { index, module ->
            positions[index] = module.modulePosition
        }

        return positions
    }

    fun resetHeading() {
        gyro.setYaw(0.0)
    }

    fun driveCommand(forwards: DoubleSupplier, strafe: DoubleSupplier, rotation: DoubleSupplier, isFieldOriented: BooleanSupplier): Command? {
        return this.run {
            if(isFieldOriented.asBoolean){
                currentSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    forwards.asDouble * TunerConstants.kSpeedAt12VoltsMps,
                    strafe.asDouble * TunerConstants.kSpeedAt12VoltsMps,
                    rotation.asDouble  * (Math.PI),
                    gyroInputs.yawDegrees
                )
            } else {
                currentSpeeds = ChassisSpeeds(
                    forwards.asDouble * 3.5,
                    strafe.asDouble * 3.5,
                    rotation.asDouble * (Math.PI),
                )
            }

            val swerveModuleStates = kinematics.toSwerveModuleStates(currentSpeeds)

            SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, TunerConstants.kSpeedAt12VoltsMps)

            modules.forEachIndexed { index, module ->
                desiredStates[index] = swerveModuleStates[index]
                module.apply(swerveModuleStates[index])
            }
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
        modules.forEachIndexed { index, swerveModule ->
            measuredStates[index] = swerveModule.state
        }
        Logger.recordOutput("swerve/measuredState", *measuredStates)
        Logger.recordOutput("swerve/desiredState", *desiredStates)
    }
}