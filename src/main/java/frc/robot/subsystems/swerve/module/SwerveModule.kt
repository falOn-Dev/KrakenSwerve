package frc.robot.subsystems.swerve.module

import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.util.Units
import frc.robot.Constants
import kotlin.math.cos

class SwerveModule(val config: SwerveModuleConstants) {
    private val io: ModuleIO
    val inputs: ModuleIO.ModuleInputs = ModuleIO.ModuleInputs()

    val positionMeters: Double
        get() = inputs.drivePositionRads * Units.inchesToMeters(config.WheelRadius)

    val velocityMetersPerSecond: Double
        get() = inputs.driveVelocityRadPerSec * Units.inchesToMeters(config.WheelRadius)

    val positionRads: Double
        get() = inputs.drivePositionRads

    val angle: Rotation2d
        get() = inputs.absoluteTurnPosition

    val state: SwerveModuleState
        get() = SwerveModuleState(velocityMetersPerSecond, angle)

    val modulePosition: SwerveModulePosition
        get() = SwerveModulePosition(positionMeters, angle)



    init {
        io = when (Constants.RobotConstants.mode) {
            Constants.RobotConstants.Mode.REAL -> ModuleIOKraken(config)
            Constants.RobotConstants.Mode.SIM -> object : ModuleIO {}
            Constants.RobotConstants.Mode.REPLAY -> object : ModuleIO {}
        }

        io.reset()
    }

    fun apply(state: SwerveModuleState) {
        val optimized = SwerveModuleState.optimize(state, inputs.turnPosition)

//        println(optimized.speedMetersPerSecond)

        var speed = optimized.speedMetersPerSecond / Units.inchesToMeters(config.WheelRadius)

        val steerError: Rotation2d = optimized.angle.minus(inputs.absoluteTurnPosition)
        speed *= cos(steerError.radians)

//        println("RadPerSec: $speed")

        io.runDriveVelocitySetpoint(speed)
        io.runTurnPositionSetpoint(optimized.angle.radians)
    }

    fun updateInputs() {
        io.updateInputs(inputs)
    }

    fun stop() {
        io.stop()
    }
}