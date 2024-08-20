package frc.robot.subsystems.swerve.module

import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.simulation.DCMotorSim


class ModuleIOSim(private val configs: SwerveModuleConstants): ModuleIO {
    private val driveMotorSim: DCMotorSim =
        DCMotorSim(DCMotor.getKrakenX60(1), configs.DriveMotorGearRatio, 0.025)
    private val turnMotorSim: DCMotorSim =
        DCMotorSim(DCMotor.getKrakenX60(1), configs.SteerMotorGearRatio, 0.004)

    private val driveFeedback = PIDController(0.0, 0.0, 0.0, 0.02)
    private val turnFeedback = PIDController(1.0, 0.0, 0.0, 0.02)

    private val driveFeedforward: SimpleMotorFeedforward = SimpleMotorFeedforward(configs.DriveMotorGains.kS, configs.DriveMotorGains.kV, configs.DriveMotorGains.kA)
    private val turnFeedforward: SimpleMotorFeedforward = SimpleMotorFeedforward(configs.SteerMotorGains.kS, configs.SteerMotorGains.kV, configs.SteerMotorGains.kA)

    private var driveAppliedVolts: Double = 0.0
    private var steerAppliedVolts: Double = 0.0
    private val turnAbsoluteInitPosition: Rotation2d = Rotation2d.fromRotations(configs.CANcoderOffset)

    init {
        turnFeedback.enableContinuousInput(-Math.PI, Math.PI)
    }

    override fun updateInputs(inputs: ModuleIO.ModuleInputs) {
        driveMotorSim.update(0.02)
        turnMotorSim.update(0.02)

        inputs.driveMotorConnected = true
        inputs.turnMotorConnected = true

        inputs.drivePositionRads = driveMotorSim.angularPositionRad
        inputs.driveVelocityRadPerSec = driveMotorSim.angularVelocityRadPerSec
        inputs.driveSupplyVolts = driveAppliedVolts
        inputs.driveMotorVolts = driveAppliedVolts
        inputs.driveStatorCurrent = driveMotorSim.currentDrawAmps
        inputs.driveSupplyCurrent = driveMotorSim.currentDrawAmps

        inputs.turnPosition = Rotation2d.fromRadians(turnMotorSim.angularPositionRad)
        inputs.absoluteTurnPosition = Rotation2d.fromRadians(turnMotorSim.angularPositionRad)
        inputs.turnVelocityRadPerSec = turnMotorSim.angularVelocityRadPerSec
        inputs.turnSupplyVolts = steerAppliedVolts
        inputs.turnMotorVolts = steerAppliedVolts
        inputs.turnStatorCurrent = turnMotorSim.currentDrawAmps
        inputs.turnSupplyCurrent = turnMotorSim.currentDrawAmps
    }

    override fun runDriveVolts(volts: Double) {
        driveAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0)
        driveMotorSim.setInputVoltage(driveAppliedVolts)
    }

    override fun runTurnVolts(volts: Double) {
        steerAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0)
        turnMotorSim.setInputVoltage(steerAppliedVolts)
    }

    override fun runTurnPositionSetpoint(positionRads: Double) {
        runTurnVolts(
            turnFeedback.calculate(turnMotorSim.angularPositionRad, positionRads) + turnFeedforward.calculate(positionRads)
        )
    }

    override fun runDriveVelocitySetpoint(velocityRadPerSec: Double) {
        runDriveVolts(
            driveFeedback.calculate(driveMotorSim.angularVelocityRadPerSec, velocityRadPerSec) + driveFeedforward.calculate(velocityRadPerSec)
        )
    }

    override fun stop() {
        runTurnVolts(0.0)
        runDriveVolts(0.0)
    }

    override fun reset() {
        driveMotorSim.setState(0.0, 0.0)
    }
}