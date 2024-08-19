package frc.robot.subsystems.swerve.gyro

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.Pigeon2Configuration
import com.ctre.phoenix6.hardware.Pigeon2
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants
import edu.wpi.first.math.geometry.Rotation2d
import frc.robot.Constants

class GyroIOPigeon2(private val configs: SwerveDrivetrainConstants) : GyroIO {
    private val gyro: Pigeon2 = Pigeon2(configs.Pigeon2Id, Constants.SwerveConstants.CANBusName)

    private val yawGetter: StatusSignal<Double> = gyro.yaw
    private val yawRateGetter: StatusSignal<Double> = gyro.angularVelocityZWorld

    init {
        val gyroConfigs: Pigeon2Configuration = configs.Pigeon2Configs
        gyro.configurator.setYaw(0.0)
        gyro.configurator.apply(gyroConfigs)
        gyro.optimizeBusUtilization()
    }

    override fun updateInputs(inputs: GyroIO.GyroInputs) {
        inputs.connected =
            BaseStatusSignal.refreshAll(
                yawGetter,
                yawRateGetter
            ).isOK

        inputs.yawDegrees = Rotation2d.fromDegrees(BaseStatusSignal.getLatencyCompensatedValue(yawGetter, yawRateGetter))
        inputs.yawVelocityDegreesPerSecond = yawRateGetter.value
    }
}