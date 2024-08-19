package frc.robot.subsystems.swerve.gyro

import edu.wpi.first.math.geometry.Rotation2d
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs

interface GyroIO {
    class GyroInputs: LoggableInputs {
        var connected: Boolean = false
        var yawDegrees: Rotation2d = Rotation2d()
        var yawVelocityDegreesPerSecond: Double = 0.0
        override fun toLog(table: LogTable?) {
            table?.put("Connected", connected)
            table?.put("YawDegrees", yawDegrees)
            table?.put("YawVelocityDegreesPerSecond", yawVelocityDegreesPerSecond)
        }

        override fun fromLog(table: LogTable?) {
            table?.get("Connected")?.let { connected = it.boolean }
            table?.get("YawDegrees", Rotation2d.struct, Rotation2d())?.let { yawDegrees = it }
            table?.get("YawVelocityDegreesPerSecond")?.let { yawVelocityDegreesPerSecond = it.double }
        }
    }

    fun updateInputs(inputs: GyroInputs) {}

    fun setYaw(newYaw: Double) {}
}