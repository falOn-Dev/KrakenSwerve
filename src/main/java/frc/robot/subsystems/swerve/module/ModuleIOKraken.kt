package frc.robot.subsystems.swerve.module

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusCode
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.CANcoderConfiguration
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage
import com.ctre.phoenix6.controls.NeutralOut
import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.util.Units
import frc.robot.Constants

class ModuleIOKraken(
    private val config: SwerveModuleConstants
) : ModuleIO {

    private val driveMotor: TalonFX = TalonFX(config.DriveMotorId, Constants.SwerveConstants.CANBusName)
    private val turnMotor: TalonFX = TalonFX(config.SteerMotorId, Constants.SwerveConstants.CANBusName)
    private val turnEncoder: CANcoder = CANcoder(config.CANcoderId, Constants.SwerveConstants.CANBusName)

    private val drivePosition: StatusSignal<Double> = driveMotor.position
    private val driveVelocity: StatusSignal<Double> = driveMotor.velocity
    private val driveSupplyVoltage: StatusSignal<Double> = driveMotor.supplyVoltage
    private val driveMotorVoltage: StatusSignal<Double> = driveMotor.motorVoltage
    private val driveSupplyCurrent: StatusSignal<Double> = driveMotor.supplyCurrent
    private val driveStatorCurrent: StatusSignal<Double> = driveMotor.statorCurrent

    private val turnPosition: StatusSignal<Double> = turnMotor.position
    private val turnVelocity: StatusSignal<Double> = turnMotor.velocity
    private val turnSupplyVoltage: StatusSignal<Double> = turnMotor.supplyVoltage
    private val turnMotorVoltage: StatusSignal<Double> = turnMotor.motorVoltage
    private val turnSupplyCurrent: StatusSignal<Double> = turnMotor.supplyCurrent
    private val turnStatorCurrent: StatusSignal<Double> = turnMotor.statorCurrent

    private val turnAbsolutePosition: StatusSignal<Double> = turnEncoder.position

    private val turnOpenLoop: VoltageOut = VoltageOut(0.0)
    private val driveOpenLoop: VoltageOut = VoltageOut(0.0)

    private val turnClosedLoop: MotionMagicExpoVoltage = MotionMagicExpoVoltage(0.0)
    private val driveClosedLoop: VelocityVoltage = VelocityVoltage(0.0)

    private val neutralOut: NeutralOut = NeutralOut()

    private val speedAt12VoltsMps: Double = config.SpeedAt12VoltsMps

    init    {
        val driveConfigs: TalonFXConfiguration = config.DriveMotorInitialConfigs
        driveConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake

        driveConfigs.Slot0 = config.DriveMotorGains
        driveConfigs.TorqueCurrent.PeakForwardTorqueCurrent = config.SlipCurrent
        driveConfigs.TorqueCurrent.PeakReverseTorqueCurrent = -config.SlipCurrent
        driveConfigs.CurrentLimits.StatorCurrentLimit = config.SlipCurrent
        driveConfigs.CurrentLimits.StatorCurrentLimitEnable = true
        driveConfigs.Feedback.SensorToMechanismRatio = config.DriveMotorGearRatio

        driveConfigs.MotorOutput.Inverted = if (config.DriveMotorInverted) InvertedValue.Clockwise_Positive
        else InvertedValue.CounterClockwise_Positive

        var status: StatusCode = driveMotor.configurator.apply(driveConfigs)
        if (!status.isOK) {
            println("Drive Motor ${config.DriveMotorId} failed to configure: $status")
        }

        val turnConfigs: TalonFXConfiguration = config.SteerMotorInitialConfigs
        turnConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake

        turnConfigs.Slot0 = config.SteerMotorGains
        turnConfigs.MotorOutput.Inverted = if (config.SteerMotorInverted) InvertedValue.Clockwise_Positive
        else InvertedValue.CounterClockwise_Positive
        turnConfigs.Feedback.FeedbackRemoteSensorID = config.CANcoderId

        if (turnMotor.isProLicensed.value) {
            turnConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder
        } else {
            turnConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder
        }

        turnConfigs.Feedback.RotorToSensorRatio = config.CouplingGearRatio

        turnConfigs.MotionMagic.MotionMagicCruiseVelocity = 100.0 / config.SteerMotorGearRatio
        turnConfigs.MotionMagic.MotionMagicAcceleration = turnConfigs.MotionMagic.MotionMagicCruiseVelocity / 0.1
        turnConfigs.MotionMagic.MotionMagicExpo_kV = 0.12 * config.SteerMotorGearRatio
        turnConfigs.MotionMagic.MotionMagicExpo_kA = 0.1

        turnConfigs.ClosedLoopGeneral.ContinuousWrap = true

        status = turnMotor.configurator.apply(turnConfigs)
        if (!status.isOK) {
            println("Turn Motor ${config.SteerMotorId} failed to configure: $status")
        }

        val cancoderConfigs: CANcoderConfiguration = config.CANcoderInitialConfigs
        cancoderConfigs.MagnetSensor.MagnetOffset = config.CANcoderOffset
        status = turnEncoder.configurator.apply(cancoderConfigs)
        if (!status.isOK) {
            println("CANCoder ${config.CANcoderId} failed to configure: $status")
        }
    }

    override fun updateInputs(inputs: ModuleIO.ModuleInputs) {
        inputs.driveMotorConnected = BaseStatusSignal.refreshAll(
            drivePosition,
            driveVelocity,
            driveSupplyVoltage,
            driveMotorVoltage,
            driveStatorCurrent,
            driveSupplyCurrent
        ).isOK

        inputs.turnMotorConnected = BaseStatusSignal.refreshAll(
            turnPosition,
            turnVelocity,
            turnSupplyVoltage,
            turnMotorVoltage,
            turnStatorCurrent,
            turnSupplyCurrent,
            turnAbsolutePosition
        ).isOK

        inputs.drivePositionRads = Units.rotationsToRadians(drivePosition.value)
        inputs.driveVelocityRadPerSec = (driveVelocity.value)
        inputs.driveSupplyVolts = driveSupplyVoltage.value
        inputs.driveMotorVolts = driveMotorVoltage.value
        inputs.driveStatorCurrent = driveStatorCurrent.value
        inputs.driveSupplyCurrent = driveSupplyCurrent.value

        inputs.turnPosition = Rotation2d.fromRotations(turnPosition.value)
        inputs.absoluteTurnPosition = Rotation2d.fromRotations(turnAbsolutePosition.value)
        inputs.turnVelocityRadPerSec = Units.rotationsToRadians(turnVelocity.value)
        inputs.turnSupplyVolts = turnSupplyVoltage.value
        inputs.turnMotorVolts = turnMotorVoltage.value
        inputs.turnStatorCurrent = turnStatorCurrent.value
        inputs.turnSupplyCurrent = turnSupplyCurrent.value
    }

    override fun runDriveVolts(volts: Double) {
        driveMotor.setControl(driveOpenLoop.withOutput(volts))
    }

    override fun runTurnVolts(volts: Double) {
        turnMotor.setControl(turnOpenLoop.withOutput(volts))
    }

    override fun runTurnPositionSetpoint(positionRads: Double) {
        turnMotor.setControl(turnClosedLoop.withPosition(Units.radiansToRotations(positionRads)))
    }

    override fun runDriveVelocitySetpoint(velocityRadPerSec: Double) {
        val speedRotations: Double = Units.radiansToRotations(velocityRadPerSec)
//        println("RotPerSec: $speedRotations")
        driveMotor.setControl(driveClosedLoop.withVelocity(speedRotations))
    }

    override fun stop() {
        driveMotor.setControl(neutralOut)
        turnMotor.setControl(neutralOut)
    }

    override fun reset() {
        driveMotor.setPosition(0.0)
    }
}