package frc.robot.subsystems.swerve

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusCode
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants
import com.ctre.phoenix6.signals.DeviceEnableValue
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.util.Units
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem so it can be used
 * in command-based projects easily.
 */
class DrivetrainIO(driveTrainConstants: SwerveDrivetrainConstants?, vararg modules: SwerveModuleConstants?) :
    SwerveDrivetrain(driveTrainConstants, *modules) {
    class ModuleInputs : LoggableInputs {
        var driveEnabled: Boolean = false
        var driveDistanceMeters: Double = 0.0
        var driveVelocityMetersPerSec: Double = 0.0
        var driveVelocityReferenceMetersPerSec: Double = 0.0
        var driveVelocityErrorMetersPerSec: Double = 0.0
        var driveAccelerationMetersPerSecPerSec: Double = 0.0
        var driveAppliedVolts: Double = 0.0
        var driveStatorCurrentAmps: Double = 0.0
        var driveSupplyCurrentAmps: Double = 0.0
        var driveTempCelsius: Double = 0.0

        var steerEnabled: Boolean = false
        var steerAbsolutePositionDeg: Double = 0.0
        var steerPositionDeg: Double = 0.0
        var steerPositionReferenceDeg: Double = 0.0
        var steerPositionErrorDeg: Double = 0.0
        var steerVelocityRevPerMin: Double = 0.0
        var steerAccelerationMetersPerSecPerSec: Double = 0.0
        var steerAppliedVolts: Double = 0.0
        var steerStatorCurrentAmps: Double = 0.0
        var steerSupplyCurrentAmps: Double = 0.0
        var steerTempCelsius: Double = 0.0

        override fun toLog(table: LogTable?) {
            table?.put("Drive Enabled", driveEnabled)
            table?.put("Drive Distance (m)", driveDistanceMeters)
            table?.put("Drive Velocity (m/s)", driveVelocityMetersPerSec)
            table?.put("Drive Velocity Reference (m/s)", driveVelocityReferenceMetersPerSec)
            table?.put("Drive Velocity Error (m/s)", driveVelocityErrorMetersPerSec)
            table?.put("Drive Acceleration (m/s^2)", driveAccelerationMetersPerSecPerSec)
            table?.put("Drive Applied Voltage (V)", driveAppliedVolts)
            table?.put("Drive Stator Current (A)", driveStatorCurrentAmps)
            table?.put("Drive Supply Current (A)", driveSupplyCurrentAmps)
            table?.put("Drive Temperature (C)", driveTempCelsius)

            table?.put("Steer Enabled", steerEnabled)
            table?.put("Steer Absolute Position (deg)", steerAbsolutePositionDeg)
            table?.put("Steer Position (deg)", steerPositionDeg)
            table?.put("Steer Position Reference (deg)", steerPositionReferenceDeg)
            table?.put("Steer Position Error (deg)", steerPositionErrorDeg)
            table?.put("Steer Velocity (rev/min)", steerVelocityRevPerMin)
            table?.put("Steer Acceleration (m/s^2)", steerAccelerationMetersPerSecPerSec)
            table?.put("Steer Applied Voltage (V)", steerAppliedVolts)
            table?.put("Steer Stator Current (A)", steerStatorCurrentAmps)
            table?.put("Steer Supply Current (A)", steerSupplyCurrentAmps)
            table?.put("Steer Temperature (C)", steerTempCelsius)
        }

        override fun fromLog(table: LogTable?) {
            table?.get("Drive Enabled")?.let { driveEnabled = it.boolean }
            table?.get("Drive Distance (m)")?.let { driveDistanceMeters = it.double }
            table?.get("Drive Velocity (m/s)")?.let { driveVelocityMetersPerSec = it.double }
            table?.get("Drive Velocity Reference (m/s)")?.let { driveVelocityReferenceMetersPerSec = it.double }
            table?.get("Drive Velocity Error (m/s)")?.let { driveVelocityErrorMetersPerSec = it.double }
            table?.get("Drive Acceleration (m/s^2)")?.let { driveAccelerationMetersPerSecPerSec = it.double }
            table?.get("Drive Applied Voltage (V)")?.let { driveAppliedVolts = it.double }
            table?.get("Drive Stator Current (A)")?.let { driveStatorCurrentAmps = it.double }
            table?.get("Drive Supply Current (A)")?.let { driveSupplyCurrentAmps = it.double }
            table?.get("Drive Temperature (C)")?.let { driveTempCelsius = it.double }

            table?.get("Steer Enabled")?.let { steerEnabled = it.boolean }
            table?.get("Steer Absolute Position (deg)")?.let { steerAbsolutePositionDeg = it.double }
            table?.get("Steer Position (deg)")?.let { steerPositionDeg = it.double }
            table?.get("Steer Position Reference (deg)")?.let { steerPositionReferenceDeg = it.double }
            table?.get("Steer Position Error (deg)")?.let { steerPositionErrorDeg = it.double }
            table?.get("Steer Velocity (rev/min)")?.let { steerVelocityRevPerMin = it.double }
            table?.get("Steer Acceleration (m/s^2)")?.let { steerAccelerationMetersPerSecPerSec = it.double }
            table?.get("Steer Applied Voltage (V)")?.let { steerAppliedVolts = it.double }
            table?.get("Steer Stator Current (A)")?.let { steerStatorCurrentAmps = it.double }
            table?.get("Steer Supply Current (A)")?.let { steerSupplyCurrentAmps = it.double }
        }

    }

    class GyroInputs : LoggableInputs {
        var connected: Boolean = false
        var yawDeg: Double = 0.0
        var yawDegPerSec: Double = 0.0
        var pitchDeg: Double = 0.0
        var pitchDegPerSec: Double = 0.0
        var rollDeg: Double = 0.0
        var rollDegPerSec: Double = 0.0
        override fun toLog(table: LogTable?) {
            table?.put("Gyro Connected", connected)
            table?.put("Yaw (deg)", yawDeg)
            table?.put("Yaw Rate (deg/s)", yawDegPerSec)
            table?.put("Pitch (deg)", pitchDeg)
            table?.put("Pitch Rate (deg/s)", pitchDegPerSec)
            table?.put("Roll (deg)", rollDeg)
            table?.put("Roll Rate (deg/s)", rollDegPerSec)
        }

        override fun fromLog(table: LogTable?) {
            table?.get("Gyro Connected")?.let { connected = it.boolean }
            table?.get("Yaw (deg)")?.let { yawDeg = it.double }
            table?.get("Yaw Rate (deg/s)")?.let { yawDegPerSec = it.double }
            table?.get("Pitch (deg)")?.let { pitchDeg = it.double }
            table?.get("Pitch Rate (deg/s)")?.let { pitchDegPerSec = it.double }
            table?.get("Roll (deg)")?.let { rollDeg = it.double }
            table?.get("Roll Rate (deg/s)")?.let { rollDegPerSec = it.double }
        }
    }

    class DrivetrainInputs : LoggableInputs {
        var targetVXMetersPerSec: Double = 0.0
        var targetVYMetersPerSec: Double = 0.0
        var targetAngularVelocityRadPerSec: Double = 0.0

        var measuredVXMetersPerSec: Double = 0.0
        var measuredVYMetersPerSec: Double = 0.0
        var measuredAngularVelocityRadPerSec: Double = 0.0

        var swerveReferenceStates: Array<SwerveModuleState> = arrayOf(
            SwerveModuleState(),
            SwerveModuleState(),
            SwerveModuleState(),
            SwerveModuleState()
        )
        var swerveMeasuredStates: Array<SwerveModuleState> = arrayOf(
            SwerveModuleState(),
            SwerveModuleState(),
            SwerveModuleState(),
            SwerveModuleState()
        )

        var robotPose: Pose2d = Pose2d()
        var rotation: Rotation2d = Rotation2d()

        override fun toLog(table: LogTable?) {
            table?.put("Target VX (m/s)", targetVXMetersPerSec)
            table?.put("Target VY (m/s)", targetVYMetersPerSec)
            table?.put("Target Angular Velocity (rad/s)", targetAngularVelocityRadPerSec)

            table?.put("Measured VX (m/s)", measuredVXMetersPerSec)
            table?.put("Measured VY (m/s)", measuredVYMetersPerSec)
            table?.put("Measured Angular Velocity (rad/s)", measuredAngularVelocityRadPerSec)

            for (i in 0..3) {
                table?.put("Swerve Module $i Reference State", swerveReferenceStates[i])
                table?.put("Swerve Module $i Measured State", swerveMeasuredStates[i])
            }

            table?.put("Robot Pose", robotPose)
            table?.put("Rotation", rotation)
        }

        override fun fromLog(table: LogTable?) {
            table?.get("Target VX (m/s)")?.let { targetVXMetersPerSec = it.double }
            table?.get("Target VY (m/s)")?.let { targetVYMetersPerSec = it.double }
            table?.get("Target Angular Velocity (rad/s)")?.let { targetAngularVelocityRadPerSec = it.double }

            table?.get("Measured VX (m/s)")?.let { measuredVXMetersPerSec = it.double }
            table?.get("Measured VY (m/s)")?.let { measuredVYMetersPerSec = it.double }
            table?.get("Measured Angular Velocity (rad/s)")?.let { measuredAngularVelocityRadPerSec = it.double }

            for (i in 0..3) {
                table?.get("Swerve Module $i Reference State", SwerveModuleState.struct, SwerveModuleState())
                    ?.let { swerveReferenceStates[i] = it }
                table?.get("Swerve Module $i Measured State", SwerveModuleState.struct, SwerveModuleState())
                    ?.let { swerveMeasuredStates[i] = it }
            }

            table?.get("Robot Pose", Pose2d.struct, Pose2d())?.let { robotPose = it }
            table?.get("Rotation", Rotation2d.struct, Rotation2d())?.let { rotation = it }
        }

    }

    class DrivetrainInputsCollection {
        val moduleInputs: Array<ModuleInputs> = arrayOf(
            ModuleInputs(),
            ModuleInputs(),
            ModuleInputs(),
            ModuleInputs()
        )
        val gyroInputs: GyroInputs = GyroInputs()
        val drivetrainInputs: DrivetrainInputs = DrivetrainInputs()
    }

    class SwerveModuleSignals(driveMotor: TalonFX, steerMotor: TalonFX) {
        var steerVelocityStatusSignal: StatusSignal<Double> = steerMotor.velocity.clone()
        var steerAccelerationStatusSignal: StatusSignal<Double> = steerMotor.acceleration.clone()
        var steerPositionErrorStatusSignal: StatusSignal<Double> = steerMotor.closedLoopError.clone()
        var steerPositionReferenceStatusSignal: StatusSignal<Double> = steerMotor.closedLoopReference.clone()
        var drivePositionStatusSignal: StatusSignal<Double> = driveMotor.position.clone()
        var driveVelocityErrorStatusSignal: StatusSignal<Double> = driveMotor.closedLoopError.clone()
        var driveVelocityReferenceStatusSignal: StatusSignal<Double> = driveMotor.closedLoopReference.clone()
        var driveAccelerationStatusSignal: StatusSignal<Double> = driveMotor.acceleration.clone()
    }

    val swerveModuleSignals: Array<SwerveModuleSignals> = arrayOf(
        SwerveModuleSignals(this.Modules[0].driveMotor, this.Modules[0].steerMotor),
        SwerveModuleSignals(this.Modules[1].driveMotor, this.Modules[1].steerMotor),
        SwerveModuleSignals(this.Modules[2].driveMotor, this.Modules[2].steerMotor),
        SwerveModuleSignals(this.Modules[3].driveMotor, this.Modules[3].steerMotor)
    )

    val pitchStatusSignal: StatusSignal<Double> = this.m_pigeon2.pitch.clone()
    val rollStatusSignal: StatusSignal<Double> = this.m_pigeon2.roll.clone()
    val angularVelocityXStatusSignal: StatusSignal<Double> = this.m_pigeon2.angularVelocityXWorld.clone()
    val angularVelocityYStatusSignal: StatusSignal<Double> = this.m_pigeon2.angularVelocityYWorld.clone()

    init {
        pitchStatusSignal.setUpdateFrequency(100.0)
        rollStatusSignal.setUpdateFrequency(100.0)
        angularVelocityXStatusSignal.setUpdateFrequency(100.0)
        angularVelocityYStatusSignal.setUpdateFrequency(100.0)
    }

    fun updateInputs(inputs: DrivetrainInputsCollection) {
        updateGyroInputs(inputs.gyroInputs)

        swerveModuleSignals.forEachIndexed { index, signals ->
            updateSwerveModuleInputs(inputs.moduleInputs[index], this.Modules[index], signals)
        }
    }

    private fun updateGyroInputs(gyroInputs: GyroInputs) {
        BaseStatusSignal.refreshAll(
            pitchStatusSignal,
            rollStatusSignal,
            angularVelocityXStatusSignal,
            angularVelocityYStatusSignal
        )

        gyroInputs.connected = this.m_yawGetter.status == StatusCode.OK
        gyroInputs.yawDeg =
            BaseStatusSignal.getLatencyCompensatedValue(this.m_yawGetter, this.m_angularVelocity)
        gyroInputs.pitchDeg =
            BaseStatusSignal.getLatencyCompensatedValue(pitchStatusSignal, angularVelocityYStatusSignal)
        gyroInputs.rollDeg =
            BaseStatusSignal.getLatencyCompensatedValue(rollStatusSignal, angularVelocityXStatusSignal)

        gyroInputs.yawDegPerSec = m_angularVelocity.valueAsDouble
        gyroInputs.pitchDegPerSec = angularVelocityYStatusSignal.valueAsDouble
        gyroInputs.rollDegPerSec = angularVelocityXStatusSignal.valueAsDouble
    }

    private fun updateSwerveModuleInputs(
        inputs: ModuleInputs,
        module: SwerveModule,
        signals: SwerveModuleSignals
    ) {
        BaseStatusSignal.refreshAll(
            signals.steerVelocityStatusSignal,
            signals.steerAccelerationStatusSignal,
            signals.steerPositionErrorStatusSignal,
            signals.steerPositionReferenceStatusSignal,
            signals.drivePositionStatusSignal,
            signals.driveVelocityErrorStatusSignal,
            signals.driveVelocityReferenceStatusSignal,
            signals.driveAccelerationStatusSignal
        )

        val position = module.getPosition(false)
        val state = module.currentState

        inputs.driveEnabled =
            module.driveMotor.deviceEnable.value == DeviceEnableValue.Enabled

        inputs.driveDistanceMeters = position.distanceMeters
        inputs.driveVelocityMetersPerSec = state.speedMetersPerSecond

        inputs.driveVelocityReferenceMetersPerSec =
            falconRPSToMechanismMPS(
                module.driveMotor.closedLoopReference.valueAsDouble,
                Units.inchesToMeters(4.0) * Math.PI,
                TunerConstants.kDriveGearRatio
            )
        inputs.driveVelocityErrorMetersPerSec =
            falconRPSToMechanismMPS(
                module.driveMotor.closedLoopError.valueAsDouble,
                Units.inchesToMeters(4.0) * Math.PI,
                TunerConstants.kDriveGearRatio
            )
        inputs.driveAccelerationMetersPerSecPerSec =
            falconRPSToMechanismMPS(
                signals.driveAccelerationStatusSignal.valueAsDouble,
                Units.inchesToMeters(4.0) * Math.PI,
                TunerConstants.kDriveGearRatio
            )

        inputs.driveAppliedVolts = module.driveMotor.motorVoltage.value
        inputs.driveStatorCurrentAmps = module.driveMotor.statorCurrent.value
        inputs.driveSupplyCurrentAmps = module.driveMotor.supplyCurrent.value
        inputs.driveTempCelsius = module.driveMotor.deviceTemp.value

        inputs.steerAbsolutePositionDeg = module.caNcoder.absolutePosition.value * 360.0

        inputs.steerEnabled =
            module.steerMotor.deviceEnable.value == DeviceEnableValue.Enabled


        // since we are using the FusedCANcoder feature, the position and velocity signal for the angle
        // motor accounts for the gear ratio; so, pass a gear ratio of 1 to just convert from rotations
        // to degrees.
        inputs.steerPositionDeg = position.angle.degrees

        inputs.steerPositionReferenceDeg =
            falconRotationsToMechanismDegrees(
                module.steerMotor.closedLoopReference.valueAsDouble,
                1.0
            )

        inputs.steerPositionErrorDeg =
            falconRotationsToMechanismDegrees(
                module.steerMotor.closedLoopError.valueAsDouble,
                1.0
            )

        inputs.steerVelocityRevPerMin =
            falconRPSToMechanismRPM(
                signals.steerVelocityStatusSignal.valueAsDouble,
                1.0
            )

        inputs.steerAccelerationMetersPerSecPerSec =
            falconRPSToMechanismRPM(
                signals.steerAccelerationStatusSignal.valueAsDouble,
                1.0
            )

        inputs.steerAppliedVolts = module.steerMotor.motorVoltage.value
        inputs.steerStatorCurrentAmps = module.steerMotor.statorCurrent.value
        inputs.steerSupplyCurrentAmps = module.steerMotor.supplyCurrent.value
        inputs.steerTempCelsius = module.steerMotor.deviceTemp.value
    }


}
