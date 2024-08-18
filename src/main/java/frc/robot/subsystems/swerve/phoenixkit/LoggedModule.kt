package frc.robot.subsystems.swerve.phoenixkit

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusCode
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.controls.TorqueCurrentFOC
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC
import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType
import com.ctre.phoenix6.signals.DeviceEnableValue
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.util.Units
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import kotlin.math.cos

class LoggedModule(
    private val constants: SwerveModuleConstants,
    private val canbusName: String
) {

    class ModuleInputs : LoggableInputs {
        var steerEnabled: Boolean = false
        var steerPositionRads: Double = 0.0
        var steerVelocityRadsPerSec: Double = 0.0
        var steerSuppliedVolts: Double = 0.0
        var steerMotorVoltage: Double = 0.0

        var driveEnabled: Boolean = false
        var drivePositionMeters: Double = 0.0
        var driveVelocityMetersPerSec: Double = 0.0
        var driveSuppliedVolts: Double = 0.0
        var driveMotorVoltage: Double = 0.0

        var absoluteEncoderPositionRads: Double = 0.0
        var absoluteEncoderVelocityRadsPerSec: Double = 0.0

        override fun toLog(table: LogTable?) {
            table?.put("steerEnabled", steerEnabled)
            table?.put("steerPositionRads", steerPositionRads)
            table?.put("steerVelocityRadsPerSec", steerVelocityRadsPerSec)
            table?.put("steerSupplyVolts", steerSuppliedVolts)
            table?.put("steerMotorVoltage", steerMotorVoltage)

            table?.put("driveEnabled", driveEnabled)
            table?.put("drivePositionMeters", drivePositionMeters)
            table?.put("driveVelocityMetersPerSec", driveVelocityMetersPerSec)
            table?.put("driveSupplyVolts", driveSuppliedVolts)
            table?.put("driveMotorVoltage", driveMotorVoltage)

            table?.put("absoluteEncoderPositionRads", absoluteEncoderPositionRads)
            table?.put("absoluteEncoderVelocityRadsPerSec", absoluteEncoderVelocityRadsPerSec)
        }

        override fun fromLog(table: LogTable?) {
            table?.get("steerConnected")?.let { steerEnabled = it.boolean }
            table?.get("steerPositionRads")?.let { steerPositionRads = it.double }
            table?.get("steerVelocityRadsPerSec")?.let { steerVelocityRadsPerSec = it.double }
            table?.get("steerSupplyVolts")?.let { steerSuppliedVolts = it.double }
            table?.get("steerMotorVoltage")?.let { steerMotorVoltage = it.double }

            table?.get("driveConnected")?.let { driveEnabled = it.boolean }
            table?.get("drivePositionMeters")?.let { drivePositionMeters = it.double }
            table?.get("driveVelocityMetersPerSec")?.let { driveVelocityMetersPerSec = it.double }
            table?.get("driveSupplyVolts")?.let { driveSuppliedVolts = it.double }
            table?.get("driveMotorVoltage")?.let { driveMotorVoltage = it.double }

            table?.get("absoluteEncoderPositionRads")?.let { absoluteEncoderPositionRads = it.double }
            table?.get("absoluteEncoderVelocityRadsPerSec")?.let { absoluteEncoderVelocityRadsPerSec = it.double }
        }
    }


    val driveMotor: TalonFX = TalonFX(constants.DriveMotorId, canbusName)
    val steerMotor: TalonFX = TalonFX(constants.SteerMotorId, canbusName)
    val absoluteEncoder: CANcoder = CANcoder(constants.CANcoderId, canbusName)

    private val drivePosition: StatusSignal<Double> = driveMotor.position.clone()
    private val driveVelocity: StatusSignal<Double> = driveMotor.velocity.clone()
    private val steerPosition: StatusSignal<Double> = steerMotor.position.clone()
    private val steerVelocity: StatusSignal<Double> = steerMotor.velocity.clone()
    val signals: Array<BaseStatusSignal> = arrayOf(
        drivePosition,
        driveVelocity,
        steerPosition,
        steerVelocity,
    )

    private val driveRotationsPerMeter: Double
    private val couplingRatioDriveRotorToCANcoder: Double

    private val speedAt12VoltsMPS: Double

    private val angleVoltageControl: MotionMagicVoltage = MotionMagicVoltage(0.0)
    private val angleTorqueControl: MotionMagicTorqueCurrentFOC = MotionMagicTorqueCurrentFOC(0.0)
    private val angleVoltageExpoControl: MotionMagicExpoVoltage = MotionMagicExpoVoltage(0.0)
    private val angleVoltageTorqueControl: MotionMagicExpoTorqueCurrentFOC = MotionMagicExpoTorqueCurrentFOC(0.0)

    private val voltageOpenLoopControl: VoltageOut = VoltageOut(0.0)
    private val velocityVoltageControl: VelocityVoltage = VelocityVoltage(0.0)
    private val velocityTorqueControl: VelocityTorqueCurrentFOC = VelocityTorqueCurrentFOC(0.0)

    private val steerClosedLoopOutput: ClosedLoopOutputType
    private val driveClosedLoopOutput: ClosedLoopOutputType

    private val internalState: SwerveModulePosition = SwerveModulePosition()
    var targetState: SwerveModuleState = SwerveModuleState()
    val currentState: SwerveModuleState
        get() = SwerveModuleState(
            driveVelocity.valueAsDouble / driveRotationsPerMeter,
            Rotation2d.fromRotations(steerPosition.valueAsDouble)
        )

    val inputs: ModuleInputs = ModuleInputs()

    init {
        val driveConfigs: TalonFXConfiguration = constants.DriveMotorInitialConfigs
        driveConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake

        driveConfigs.Slot0 = constants.DriveMotorGains
        driveConfigs.TorqueCurrent.PeakForwardTorqueCurrent = constants.SlipCurrent
        driveConfigs.TorqueCurrent.PeakReverseTorqueCurrent = -constants.SlipCurrent
        driveConfigs.CurrentLimits.StatorCurrentLimit = constants.SlipCurrent
        driveConfigs.CurrentLimits.StatorCurrentLimitEnable = true

        driveConfigs.MotorOutput.Inverted = if (constants.DriveMotorInverted) InvertedValue.Clockwise_Positive
        else InvertedValue.CounterClockwise_Positive
        var response: StatusCode = driveMotor.configurator.apply(driveConfigs)
        if (!response.isOK) {
            println(
                "TalonFX ID " + driveMotor.deviceID + " failed config with error " + response.toString()
            )
        }

        val steerConfigs = constants.SteerMotorInitialConfigs
        steerConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake

        steerConfigs.Slot0 = constants.SteerMotorGains

        // Modify configuration to use remote CANcoder fused
        steerConfigs.Feedback.FeedbackRemoteSensorID = constants.CANcoderId
        when (constants.FeedbackSource) {
            SteerFeedbackType.RemoteCANcoder -> steerConfigs.Feedback.FeedbackSensorSource =
                FeedbackSensorSourceValue.RemoteCANcoder

            SteerFeedbackType.FusedCANcoder -> steerConfigs.Feedback.FeedbackSensorSource =
                FeedbackSensorSourceValue.FusedCANcoder

            SteerFeedbackType.SyncCANcoder -> steerConfigs.Feedback.FeedbackSensorSource =
                FeedbackSensorSourceValue.SyncCANcoder
        }
        steerConfigs.Feedback.RotorToSensorRatio = constants.SteerMotorGearRatio

        steerConfigs.MotionMagic.MotionMagicCruiseVelocity = 100.0 / constants.SteerMotorGearRatio
        steerConfigs.MotionMagic.MotionMagicAcceleration = steerConfigs.MotionMagic.MotionMagicCruiseVelocity / 0.100
        steerConfigs.MotionMagic.MotionMagicExpo_kV = 0.12 * constants.SteerMotorGearRatio
        steerConfigs.MotionMagic.MotionMagicExpo_kA = 0.1

        steerConfigs.ClosedLoopGeneral.ContinuousWrap = true // Enable continuous wrap for swerve modules

        steerConfigs.MotorOutput.Inverted = if (constants.SteerMotorInverted
        ) InvertedValue.Clockwise_Positive
        else InvertedValue.CounterClockwise_Positive
        response = steerMotor.configurator.apply(steerConfigs)
        if (!response.isOK) {
            println(
                "TalonFX ID " + steerMotor.deviceID + " failed config with error: " + response.toString()
            )
        }

        val cancoderConfigs = constants.CANcoderInitialConfigs
        cancoderConfigs.MagnetSensor.MagnetOffset = constants.CANcoderOffset
        response = absoluteEncoder.configurator.apply(cancoderConfigs)
        if (!response.isOK) {
            println(
                "CANcoder ID " + absoluteEncoder.deviceID + " failed config with error: " + response.toString()
            )
        }

        /* Calculate the ratio of drive motor rotation to meter on ground */
        val rotationsPerWheelRotation = constants.DriveMotorGearRatio
        val metersPerWheelRotation = 2 * Math.PI * Units.inchesToMeters(constants.WheelRadius)
        driveRotationsPerMeter = rotationsPerWheelRotation / metersPerWheelRotation
        couplingRatioDriveRotorToCANcoder = constants.CouplingGearRatio

        angleVoltageControl.UpdateFreqHz = 0.0
        angleTorqueControl.UpdateFreqHz = 0.0
        angleVoltageExpoControl.UpdateFreqHz = 0.0
        angleVoltageTorqueControl.UpdateFreqHz = 0.0

        voltageOpenLoopControl.UpdateFreqHz = 0.0
        velocityVoltageControl.UpdateFreqHz = 0.0
        velocityTorqueControl.UpdateFreqHz = 0.0

        steerClosedLoopOutput = constants.SteerMotorClosedLoopOutput
        driveClosedLoopOutput = constants.DriveMotorClosedLoopOutput

        speedAt12VoltsMPS = constants.SpeedAt12VoltsMps
    }

    fun periodic() {
        BaseStatusSignal.refreshAll(*signals)

        inputs.steerEnabled = steerMotor.deviceEnable.value == DeviceEnableValue.Enabled
        inputs.steerPositionRads = steerPosition.value
        inputs.steerVelocityRadsPerSec = steerVelocity.value
        inputs.steerSuppliedVolts = steerMotor.supplyVoltage.valueAsDouble
        inputs.steerMotorVoltage = steerMotor.motorVoltage.valueAsDouble

        inputs.driveEnabled = driveMotor.deviceEnable.value == DeviceEnableValue.Enabled
        inputs.drivePositionMeters = drivePosition.value
        inputs.driveVelocityMetersPerSec = driveVelocity.value
        inputs.driveSuppliedVolts = driveMotor.supplyVoltage.valueAsDouble
        inputs.driveMotorVoltage = driveMotor.motorVoltage.valueAsDouble

        inputs.absoluteEncoderPositionRads = Units.rotationsToRadians(absoluteEncoder.position.valueAsDouble)
        inputs.absoluteEncoderVelocityRadsPerSec = Units.rotationsToRadians(absoluteEncoder.velocity.valueAsDouble)
    }

    fun getPosition(refresh: Boolean): SwerveModulePosition {
        if (refresh) {
            BaseStatusSignal.refreshAll(*signals)
        }

        var driveRot = BaseStatusSignal.getLatencyCompensatedValue(drivePosition, driveVelocity)
        val steerRot = BaseStatusSignal.getLatencyCompensatedValue(steerPosition, steerVelocity)

        driveRot -= steerRot * couplingRatioDriveRotorToCANcoder

        internalState.distanceMeters = driveRot / driveRotationsPerMeter
        internalState.angle = Rotation2d.fromRadians(steerRot)

        return internalState
    }

    /**
     * Get the cached position of the module
     */
    fun getCachedPosition(): SwerveModulePosition = internalState

    fun apply(state: SwerveModuleState, driveRequestType: DriveRequestType) =
        apply(state, driveRequestType, SteerRequestType.MotionMagic)

    fun apply(state: SwerveModuleState, driveRequestType: DriveRequestType, steerRequestType: SteerRequestType) {
        val optimized = SwerveModuleState.optimize(state, internalState.angle)
        targetState = optimized

        val angleToSet = optimized.angle.rotations
        when (steerRequestType) {
            SteerRequestType.MotionMagic -> {
                when (steerClosedLoopOutput) {
                    ClosedLoopOutputType.Voltage ->
                        steerMotor.setControl(angleVoltageControl.withPosition(angleToSet))

                    ClosedLoopOutputType.TorqueCurrentFOC ->
                        steerMotor.setControl(angleTorqueControl.withPosition(angleToSet))
                }
            }

            SteerRequestType.MotionMagicExpo -> {
                when (steerClosedLoopOutput) {
                    ClosedLoopOutputType.Voltage ->
                        steerMotor.setControl(angleVoltageExpoControl.withPosition(angleToSet))

                    ClosedLoopOutputType.TorqueCurrentFOC ->
                        steerMotor.setControl(angleVoltageTorqueControl.withPosition(angleToSet))
                }
            }
        }

        var velocityToSet = optimized.speedMetersPerSecond * driveRotationsPerMeter

        val steerError = angleToSet - steerPosition.valueAsDouble

        var cosineScalar = cos(Units.rotationsToRadians(steerError))

        if (cosineScalar < 0.0) cosineScalar = 0.0

        velocityToSet *= cosineScalar

        /* Back out the expected shimmy the drive motor will see */
        /* Find the angular rate to determine what to back out */
        val azimuthTurnRps: Double = steerVelocity.getValue()

        /* Azimuth turn rate multiplied by coupling ratio provides back-out rps */
        val driveRateBackOut: Double = azimuthTurnRps * couplingRatioDriveRotorToCANcoder
        velocityToSet += driveRateBackOut

        when (driveRequestType) {
            DriveRequestType.OpenLoopVoltage -> {
                /* Open loop ignores the driveRotationsPerMeter since it only cares about the open loop at the mechanism */
                /* But we do care about the backout due to coupling, so we keep it in */
                velocityToSet /= driveRotationsPerMeter
                driveMotor.setControl(voltageOpenLoopControl.withOutput(velocityToSet / speedAt12VoltsMPS * 12.0))
            }

            DriveRequestType.Velocity -> {
                when (driveClosedLoopOutput) {
                    ClosedLoopOutputType.Voltage ->
                        driveMotor.setControl(velocityVoltageControl.withVelocity(velocityToSet))

                    ClosedLoopOutputType.TorqueCurrentFOC ->
                        driveMotor.setControl(velocityTorqueControl.withVelocity(velocityToSet))
                }
            }
        }
    }

    fun applyCharacterization(steerTarget: Rotation2d, driveRequest: VoltageOut) {
        val angleToSet: Double = steerTarget.rotations

        when (steerClosedLoopOutput) {
            ClosedLoopOutputType.Voltage ->
                steerMotor.setControl(angleVoltageControl.withPosition(angleToSet))

            ClosedLoopOutputType.TorqueCurrentFOC ->
                steerMotor.setControl(angleTorqueControl.withPosition(angleToSet))
        }

        driveMotor.setControl(driveRequest)
    }

    fun applyCharacterization(steerTarget: Rotation2d, driveRequest: TorqueCurrentFOC) {
        val angleToSet: Double = steerTarget.rotations

        when (steerClosedLoopOutput) {
            ClosedLoopOutputType.Voltage ->
                steerMotor.setControl(angleVoltageControl.withPosition(angleToSet))

            ClosedLoopOutputType.TorqueCurrentFOC ->
                steerMotor.setControl(angleTorqueControl.withPosition(angleToSet))
        }

        driveMotor.setControl(driveRequest)
    }

    fun configNeutralMode(neutralMode: NeutralModeValue): StatusCode {
        val configs = MotorOutputConfigs()

        var status: StatusCode = driveMotor.configurator.refresh(configs)
        if (status.isOK) {
            configs.NeutralMode = neutralMode
            status = driveMotor.configurator.apply(configs)
        }
        if (!status.isOK) {
            println(
                "TalonFX ID " + driveMotor.deviceID + " failed config neutral mode with error " + status.toString()
            )
        }
        return status
    }

    fun resetPosition() = driveMotor.setPosition(0.0)

}

