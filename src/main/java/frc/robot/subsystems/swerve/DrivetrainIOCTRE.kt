package frc.robot.subsystems.swerve

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusCode
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.Utils
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.ApplyChassisSpeeds
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.FieldCentric
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.Idle
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.RobotCentric
import com.ctre.phoenix6.signals.DeviceEnableValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.Notifier
import edu.wpi.first.wpilibj.RobotController
import frc.robot.Constants


/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem so it can be used
 * in command-based projects easily.
 */
class DrivetrainIOCTRE(driveTrainConstants: SwerveDrivetrainConstants?, vararg modules: SwerveModuleConstants?) :
    SwerveDrivetrain(driveTrainConstants, *modules), DrivetrainIO {

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

    private val kSimLoopPeriod: Double = 0.005 // 5 ms
    private var m_simNotifier: Notifier? = null
    private var m_lastSimTime = 0.0


    val swerveModuleSignals: Array<SwerveModuleSignals> = arrayOf(
        SwerveModuleSignals(this.Modules[0].driveMotor, this.Modules[0].steerMotor),
        SwerveModuleSignals(this.Modules[1].driveMotor, this.Modules[1].steerMotor),
        SwerveModuleSignals(this.Modules[2].driveMotor, this.Modules[2].steerMotor),
        SwerveModuleSignals(this.Modules[3].driveMotor, this.Modules[3].steerMotor),
    )

    val pitchStatusSignal: StatusSignal<Double> = this.m_pigeon2.pitch.clone()
    val rollStatusSignal: StatusSignal<Double> = this.m_pigeon2.roll.clone()
    val angularVelocityXStatusSignal: StatusSignal<Double> = this.m_pigeon2.angularVelocityXWorld.clone()
    val angularVelocityYStatusSignal: StatusSignal<Double> = this.m_pigeon2.angularVelocityYWorld.clone()

    private val idleRequest = Idle()
    private val driveRobotCentricRequest = RobotCentric()
    private val driveFieldCentricRequest = FieldCentric()
    private val applyChassisSpeedsRequest = ApplyChassisSpeeds()

    private var targetChassisSpeeds: ChassisSpeeds = ChassisSpeeds()

    init {
        pitchStatusSignal.setUpdateFrequency(100.0)
        rollStatusSignal.setUpdateFrequency(100.0)
        angularVelocityXStatusSignal.setUpdateFrequency(100.0)
        angularVelocityYStatusSignal.setUpdateFrequency(100.0)
        if(Constants.RobotConstants.mode == Constants.RobotConstants.Mode.SIM) {
            setupSim()
        }
    }

    private fun setupSim() {
        m_lastSimTime = Utils.getCurrentTimeSeconds()

        m_simNotifier = Notifier {
            val currentTime = Utils.getCurrentTimeSeconds()
            val dt = currentTime - m_lastSimTime
            m_lastSimTime = currentTime

            updateSimState(dt, RobotController.getBatteryVoltage())
        }
    }

    override fun updateInputs(inputs: DrivetrainIO.DrivetrainInputsCollection) {
        updateGyroInputs(inputs.gyroInputs)

        swerveModuleSignals.forEachIndexed { index, signals ->
            updateSwerveModuleInputs(inputs.moduleInputs[index], this.Modules[index], signals)
        }

        inputs.drivetrainInputs.swerveMeasuredStates = this.state.ModuleStates
        inputs.drivetrainInputs.swerveReferenceStates = this.state.ModuleTargets

        inputs.drivetrainInputs.robotPose = this.state.Pose

        inputs.drivetrainInputs.targetVXMetersPerSec = targetChassisSpeeds.vxMetersPerSecond
        inputs.drivetrainInputs.targetVYMetersPerSec = targetChassisSpeeds.vyMetersPerSecond
        inputs.drivetrainInputs.targetAngularVelocityRadPerSec =
            targetChassisSpeeds.omegaRadiansPerSecond

        inputs.drivetrainInputs.measuredVXMetersPerSec = this.state.speeds.vxMetersPerSecond
        inputs.drivetrainInputs.measuredVYMetersPerSec = this.state.speeds.vyMetersPerSecond
        inputs.drivetrainInputs.measuredAngularVelocityRadPerSec = this.state.speeds.omegaRadiansPerSecond

        inputs.drivetrainInputs.rotation = this.state.Pose.rotation
    }

    override fun driveFieldRelative(
        xVelocity: Double,
        yVelocity: Double,
        rotationalVelocity: Double,
        isOpenLoop: Boolean,
    ) {
        this.targetChassisSpeeds =
            ChassisSpeeds.discretize(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    xVelocity, yVelocity, rotationalVelocity, this.state.Pose.rotation,
                ),
                0.02,
            )

        if (isOpenLoop) {
            this.setControl(
                driveFieldCentricRequest
                    .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
                    .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo)
                    .withVelocityX(xVelocity)
                    .withVelocityY(yVelocity)
                    .withRotationalRate(rotationalVelocity),
            )
        } else {
            this.setControl(
                driveFieldCentricRequest
                    .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
                    .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo)
                    .withVelocityX(xVelocity)
                    .withVelocityY(yVelocity)
                    .withRotationalRate(rotationalVelocity),
            )
        }
    }

    override fun driveRobotRelative(
        xVelocity: Double,
        yVelocity: Double,
        rotationalVelocity: Double,
        isOpenLoop: Boolean,
    ) {
        this.targetChassisSpeeds =
            ChassisSpeeds.discretize(
                ChassisSpeeds(xVelocity, yVelocity, rotationalVelocity),
                0.02,
            )

        if (isOpenLoop) {
            this.setControl(
                driveRobotCentricRequest
                    .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
                    .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo)
                    .withVelocityX(xVelocity)
                    .withVelocityY(yVelocity)
                    .withRotationalRate(rotationalVelocity),
            )
        } else {
            this.setControl(
                driveRobotCentricRequest
                    .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
                    .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo)
                    .withVelocityX(xVelocity)
                    .withVelocityY(yVelocity)
                    .withRotationalRate(rotationalVelocity),
            )
        }
    }

    override fun setChassisSpeeds(speeds: ChassisSpeeds?, isOpenLoop: Boolean) {
        targetChassisSpeeds.omegaRadiansPerSecond = speeds!!.omegaRadiansPerSecond
        targetChassisSpeeds.vxMetersPerSecond = speeds!!.vxMetersPerSecond
        targetChassisSpeeds.vyMetersPerSecond = speeds!!.vyMetersPerSecond

        if (isOpenLoop) {
            this.setControl(
                applyChassisSpeedsRequest
                    .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
                    .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo)
                    .withSpeeds(speeds),
            )
        } else {
            this.setControl(
                applyChassisSpeedsRequest
                    .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
                    .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo)
                    .withSpeeds(speeds),
            )
        }
    }

    override fun resetPose(pose: Pose2d?) {
        this.seedFieldRelative(pose)
    }

    override fun resetPose() {
        this.seedFieldRelative()
    }

    override fun setBrakeMode(enable: Boolean) {
        for (swerveModule in this.Modules) {
            val config = MotorOutputConfigs()
            swerveModule.driveMotor.configurator.refresh(config)
            config.NeutralMode = if (enable) NeutralModeValue.Brake else NeutralModeValue.Coast
            swerveModule.driveMotor.configurator.apply(config)
        }
    }

    private fun updateGyroInputs(gyroInputs: DrivetrainIO.GyroInputs) {
        BaseStatusSignal.refreshAll(
            pitchStatusSignal,
            rollStatusSignal,
            angularVelocityXStatusSignal,
            angularVelocityYStatusSignal,
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
        inputs: DrivetrainIO.ModuleInputs,
        module: SwerveModule,
        signals: SwerveModuleSignals,
    ) {
        BaseStatusSignal.refreshAll(
            signals.steerVelocityStatusSignal,
            signals.steerAccelerationStatusSignal,
            signals.steerPositionErrorStatusSignal,
            signals.steerPositionReferenceStatusSignal,
            signals.drivePositionStatusSignal,
            signals.driveVelocityErrorStatusSignal,
            signals.driveVelocityReferenceStatusSignal,
            signals.driveAccelerationStatusSignal,
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
                TunerConstants.kDriveGearRatio,
            )
        inputs.driveVelocityErrorMetersPerSec =
            falconRPSToMechanismMPS(
                module.driveMotor.closedLoopError.valueAsDouble,
                Units.inchesToMeters(4.0) * Math.PI,
                TunerConstants.kDriveGearRatio,
            )
        inputs.driveAccelerationMetersPerSecPerSec =
            falconRPSToMechanismMPS(
                signals.driveAccelerationStatusSignal.valueAsDouble,
                Units.inchesToMeters(4.0) * Math.PI,
                TunerConstants.kDriveGearRatio,
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
                1.0,
            )

        inputs.steerPositionErrorDeg =
            falconRotationsToMechanismDegrees(
                module.steerMotor.closedLoopError.valueAsDouble,
                1.0,
            )

        inputs.steerVelocityRevPerMin =
            falconRPSToMechanismRPM(
                signals.steerVelocityStatusSignal.valueAsDouble,
                1.0,
            )

        inputs.steerAccelerationMetersPerSecPerSec =
            falconRPSToMechanismRPM(
                signals.steerAccelerationStatusSignal.valueAsDouble,
                1.0,
            )

        inputs.steerAppliedVolts = module.steerMotor.motorVoltage.value
        inputs.steerStatorCurrentAmps = module.steerMotor.statorCurrent.value
        inputs.steerSupplyCurrentAmps = module.steerMotor.supplyCurrent.value
        inputs.steerTempCelsius = module.steerMotor.deviceTemp.value
    }
}
