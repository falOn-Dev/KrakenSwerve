package frc.robot.subsystems.swerve.module

import edu.wpi.first.math.geometry.Rotation2d
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs

interface ModuleIO {
    class ModuleInputs: LoggableInputs {
        var driveMotorConnected: Boolean = false
        var turnMotorConnected: Boolean = false

        var drivePositionRads: Double = 0.0
        var driveVelocityRadPerSec: Double = 0.0
        var driveSupplyVolts: Double = 0.0
        var driveMotorVolts: Double = 0.0
        var driveStatorCurrent: Double = 0.0
        var driveSupplyCurrent: Double = 0.0

        var turnPosition: Rotation2d = Rotation2d()
        var absoluteTurnPosition: Rotation2d = Rotation2d()
        var turnVelocityRadPerSec: Double = 0.0
        var turnSupplyVolts: Double = 0.0
        var turnMotorVolts: Double = 0.0
        var turnStatorCurrent: Double = 0.0
        var turnSupplyCurrent: Double = 0.0
        override fun toLog(table: LogTable?) {
            table?.put("driveMotorConnected", driveMotorConnected)
            table?.put("turnMotorConnected", turnMotorConnected)

            table?.put("drivePositionRads", drivePositionRads)
            table?.put("driveVelocityRadPerSec", driveVelocityRadPerSec)
            table?.put("driveSupplyVolts", driveSupplyVolts)
            table?.put("driveMotorVolts", driveMotorVolts)
            table?.put("driveStatorCurrent", driveStatorCurrent)
            table?.put("driveSupplyCurrent", driveSupplyCurrent)

            table?.put("turnPosition", turnPosition.degrees)
            table?.put("absoluteTurnPosition", absoluteTurnPosition.degrees)
            table?.put("turnVelocityRadPerSec", turnVelocityRadPerSec)
            table?.put("turnSupplyVolts", turnSupplyVolts)
            table?.put("turnMotorVolts", turnMotorVolts)
            table?.put("turnStatorCurrent", turnStatorCurrent)
            table?.put("turnSupplyCurrent", turnSupplyCurrent)
        }

        override fun fromLog(table: LogTable?) {
            table?.get("driveMotorConnected")?.let { driveMotorConnected = it.boolean }
            table?.get("turnMotorConnected")?.let { turnMotorConnected = it.boolean }

            table?.get("drivePositionRads")?.let { drivePositionRads = it.double }
            table?.get("driveVelocityRadPerSec")?.let { driveVelocityRadPerSec = it.double }
            table?.get("driveSupplyVolts")?.let { driveSupplyVolts = it.double }
            table?.get("driveMotorVolts")?.let { driveMotorVolts = it.double }
            table?.get("driveStatorCurrent")?.let { driveStatorCurrent = it.double }
            table?.get("driveSupplyCurrent")?.let { driveSupplyCurrent = it.double }

            table?.get("turnPosition", Rotation2d.struct, Rotation2d())?.let { turnPosition = it }
            table?.get("absoluteTurnPosition", Rotation2d.struct, Rotation2d())?.let { absoluteTurnPosition = it }
            table?.get("turnVelocityRadPerSec")?.let { turnVelocityRadPerSec = it.double }
            table?.get("turnSupplyVolts")?.let { turnSupplyVolts = it.double }
            table?.get("turnMotorVolts")?.let { turnMotorVolts = it.double }
            table?.get("turnStatorCurrent")?.let { turnStatorCurrent = it.double }
            table?.get("turnSupplyCurrent")?.let { turnSupplyCurrent = it.double }
        }
    }

    fun updateInputs(inputs: ModuleInputs) {}

    fun runDriveVolts(volts: Double) {}

    fun runTurnVolts(volts: Double) {}

    fun runTurnPositionSetpoint(positionRads: Double) {}

    fun runDriveVelocitySetpoint(velocityRadPerSec: Double) {}

    fun stop() {}
}