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

/**
 * Swerve module IO for the Kraken
 *
 * This class implements the ModuleIO interface for the Kraken swerve module
 *
 * @param config The module's configuration
 * @see ModuleIO
 * @see SwerveModuleConstants
 * @constructor Creates a new IO layer from given configuration
 */
class ModuleIOKraken(
    private val config: SwerveModuleConstants,
) : ModuleIO {

    /** Drive motor, Kraken x60 w/ TalonFX */
    private val driveMotor: TalonFX = TalonFX(config.DriveMotorId, Constants.SwerveConstants.CANBusName)
    /** Turn motor, Kraken x60 w/ TalonFX */
    private val turnMotor: TalonFX = TalonFX(config.SteerMotorId, Constants.SwerveConstants.CANBusName)
    /** Turn motor's encoder, CANCoder magnetic encoder */
    private val turnEncoder: CANcoder = CANcoder(config.CANcoderId, Constants.SwerveConstants.CANBusName)

    /** The drive motor's position signal */
    private val drivePosition: StatusSignal<Double> = driveMotor.position

    /** The drive motor's velocity signal */
    private val driveVelocity: StatusSignal<Double> = driveMotor.velocity

    /** The drive motor's supply voltage signal */
    private val driveSupplyVoltage: StatusSignal<Double> = driveMotor.supplyVoltage

    /** The drive motor's voltage signal */
    private val driveMotorVoltage: StatusSignal<Double> = driveMotor.motorVoltage

    /** The drive motor's supply current signal */
    private val driveSupplyCurrent: StatusSignal<Double> = driveMotor.supplyCurrent

    /** The drive motor's stator current signal */
    private val driveStatorCurrent: StatusSignal<Double> = driveMotor.statorCurrent

    /** The turn motor's position signal */
    private val turnPosition: StatusSignal<Double> = turnMotor.position

    /** The turn motor's velocity signal */
    private val turnVelocity: StatusSignal<Double> = turnMotor.velocity

    /** The turn motor's supply voltage signal */
    private val turnSupplyVoltage: StatusSignal<Double> = turnMotor.supplyVoltage

    /** The turn motor's voltage signal */
    private val turnMotorVoltage: StatusSignal<Double> = turnMotor.motorVoltage

    /** The turn motor's supply current signal */
    private val turnSupplyCurrent: StatusSignal<Double> = turnMotor.supplyCurrent

    /** The turn motor's stator current signal */
    private val turnStatorCurrent: StatusSignal<Double> = turnMotor.statorCurrent

    /** The turn motor's absolute position signal */
    private val turnAbsolutePosition: StatusSignal<Double> = turnEncoder.position

    /** Open loop voltage command for the turn motor */
    private val turnOpenLoop: VoltageOut = VoltageOut(0.0)

    /** Open loop voltage command for the drive motor */
    private val driveOpenLoop: VoltageOut = VoltageOut(0.0)

    /**
     * Closed loop control request for the turn motor
     *
     * Change this to different control modes as needed
     */
    private val turnClosedLoop: MotionMagicExpoVoltage = MotionMagicExpoVoltage(0.0)

    /**
     * Closed loop control request for the drive motor
     *
     * Change this to different control modes as needed
     */
    private val driveClosedLoop: VelocityVoltage = VelocityVoltage(0.0)

    /** Neutral output command for both motors */
    private val neutralOut: NeutralOut = NeutralOut()

    /** The speed of the module at 12 volts in meters per second */
    private val speedAt12VoltsMps: Double = config.SpeedAt12VoltsMps

    init {
        val driveConfigs: TalonFXConfiguration = config.DriveMotorInitialConfigs
        driveConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake

        driveConfigs.Slot0 = config.DriveMotorGains
        driveConfigs.TorqueCurrent.PeakForwardTorqueCurrent = config.SlipCurrent
        driveConfigs.TorqueCurrent.PeakReverseTorqueCurrent = -config.SlipCurrent
        driveConfigs.CurrentLimits.StatorCurrentLimit = config.SlipCurrent
        driveConfigs.CurrentLimits.StatorCurrentLimitEnable = true
        driveConfigs.Feedback.SensorToMechanismRatio = config.DriveMotorGearRatio

        driveConfigs.MotorOutput.Inverted = if (config.DriveMotorInverted) {
            InvertedValue.Clockwise_Positive
        } else {
            InvertedValue.CounterClockwise_Positive
        }

        var status: StatusCode = driveMotor.configurator.apply(driveConfigs)
        if (!status.isOK) {
            println("Drive Motor ${config.DriveMotorId} failed to configure: $status")
        }

        val turnConfigs: TalonFXConfiguration = config.SteerMotorInitialConfigs
        turnConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake

        turnConfigs.Slot0 = config.SteerMotorGains
        turnConfigs.MotorOutput.Inverted = if (config.SteerMotorInverted) {
            InvertedValue.Clockwise_Positive
        } else {
            InvertedValue.CounterClockwise_Positive
        }
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


    /**
     * Update the inputs using the current state of the module
     * @param inputs The inputs to update (mutated in place)
     */
    override fun updateInputs(inputs: ModuleIO.ModuleInputs) {
        inputs.driveMotorConnected = BaseStatusSignal.refreshAll(
            drivePosition,
            driveVelocity,
            driveSupplyVoltage,
            driveMotorVoltage,
            driveStatorCurrent,
            driveSupplyCurrent,
        ).isOK

        inputs.turnMotorConnected = BaseStatusSignal.refreshAll(
            turnPosition,
            turnVelocity,
            turnSupplyVoltage,
            turnMotorVoltage,
            turnStatorCurrent,
            turnSupplyCurrent,
            turnAbsolutePosition,
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

    /**
     * Run the drive motor at a given voltage
     *
     * @param volts The voltage to run the motor at
     */
    override fun runDriveVolts(volts: Double) {
        driveMotor.setControl(driveOpenLoop.withOutput(volts))
    }

    /**
     * Run the turn motor at a given voltage
     *
     * @param volts The voltage to run the motor at
     */
    override fun runTurnVolts(volts: Double) {
        turnMotor.setControl(turnOpenLoop.withOutput(volts))
    }

    /**
     * Set the position setpoint for the turn motor
     *
     * @param positionRads The position to set the motor to in radians
     */
    override fun runTurnPositionSetpoint(positionRads: Double) {
        turnMotor.setControl(turnClosedLoop.withPosition(Units.radiansToRotations(positionRads)))
    }

    /**
     * Set the velocity setpoint for the drive motor
     *
     * @param velocityRadPerSec The velocity to set the motor to in radians per second
     */
    override fun runDriveVelocitySetpoint(velocityRadPerSec: Double) {
        val speedRotations: Double = Units.radiansToRotations(velocityRadPerSec)
//        println("RotPerSec: $speedRotations")
        driveMotor.setControl(driveClosedLoop.withVelocity(speedRotations))
    }

    /**
     * Stop the module
     */
    override fun stop() {
        driveMotor.setControl(neutralOut)
        turnMotor.setControl(neutralOut)
    }

    /**
     * Reset the module's position, for odometry purposes
     */
    override fun reset() {
        driveMotor.setPosition(0.0)
    }

    /**
     * Set the PID constants for a motor
     *
     * @param p The proportional constant
     * @param i The integral constant
     * @param d The derivative constant
     * @param motor The motor to set the constants for
     */
    override fun setPID(p: Double, i: Double, d: Double, motor: ModuleIO.ModuleMotor) {
        when(motor){
            ModuleIO.ModuleMotor.DRIVE -> {
                config.DriveMotorGains.kP = p
                config.DriveMotorGains.kI = i
                config.DriveMotorGains.kD = d
                driveMotor.configurator.apply(config.DriveMotorGains)
            }
            ModuleIO.ModuleMotor.TURN -> {
                config.SteerMotorGains.kP = p
                config.SteerMotorGains.kI = i
                config.SteerMotorGains.kD = d
                turnMotor.configurator.apply(config.SteerMotorGains)
            }
        }
    }

    /**
     * Set the feedforward constants for a motor
     *
     * @param kV The velocity feedforward constant
     * @param kA The acceleration feedforward constant
     * @param kS The static feedforward constant
     * @param motor The motor to set the constants for
     */
    override fun setFF(kV: Double, kA: Double, kS: Double, motor: ModuleIO.ModuleMotor) {
        when(motor){
            ModuleIO.ModuleMotor.DRIVE -> {
                config.DriveMotorGains.kV = kV
                config.DriveMotorGains.kA = kA
                config.DriveMotorGains.kS = kS
                driveMotor.configurator.apply(config.DriveMotorGains)
            }
            ModuleIO.ModuleMotor.TURN -> {
                config.SteerMotorGains.kV = kV
                config.SteerMotorGains.kA = kA
                config.SteerMotorGains.kS = kS
                turnMotor.configurator.apply(config.SteerMotorGains)
            }
        }
    }
}
