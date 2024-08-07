package frc.robot.subsystems.swerve

import com.ctre.phoenix6.SignalLogger
import com.ctre.phoenix6.Utils
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.util.HolonomicPathFollowerConfig
import com.pathplanner.lib.util.PIDConstants
import com.pathplanner.lib.util.ReplanningConfig
import edu.wpi.first.math.filter.SlewRateLimiter
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units
import edu.wpi.first.units.Voltage
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Notifier
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Subsystem
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import java.util.function.DoubleSupplier
import java.util.function.Supplier

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem so it can be used
 * in command-based projects easily.
 */
class Drivetrain : SwerveDrivetrain, Subsystem {
    private var m_simNotifier: Notifier? = null
    private var m_lastSimTime = 0.0

    private val rotationSysIDReq: SwerveRequest.SysIdSwerveRotation =
        SwerveRequest.SysIdSwerveRotation()
    private val steerSysIDReq: SwerveRequest.SysIdSwerveSteerGains =
        SwerveRequest.SysIdSwerveSteerGains()
    private val translationSysIDReq: SwerveRequest.SysIdSwerveTranslation =
        SwerveRequest.SysIdSwerveTranslation()

    private val rotationSysIDRoutine: SysIdRoutine =
        SysIdRoutine(
            SysIdRoutine.Config(
                null,
                Units.Volts.of(4.0),
                null,
                { state -> SignalLogger.writeString("state", state.toString()) },
            ),
            SysIdRoutine.Mechanism(
                { volts: Measure<Voltage> -> setControl(rotationSysIDReq.withVolts(volts)) },
                null,
                this,
            ),
        )

    private val steerSysIDRoutine: SysIdRoutine =
        SysIdRoutine(
            SysIdRoutine.Config(
                null,
                Units.Volts.of(7.0),
                null,
                { state -> SignalLogger.writeString("state", state.toString()) },
            ),
            SysIdRoutine.Mechanism(
                { volts: Measure<Voltage> -> setControl(steerSysIDReq.withVolts(volts)) },
                null,
                this,
            ),
        )

    private val translationSysIDRoutine: SysIdRoutine =
        SysIdRoutine(
            SysIdRoutine.Config(
                null,
                Units.Volts.of(4.0),
                Units.Seconds.of(3.0),
                { state -> SignalLogger.writeString("state", state.toString()) },
            ),
            SysIdRoutine.Mechanism(
                { volts: Measure<Voltage> -> setControl(translationSysIDReq.withVolts(volts)) },
                null,
                this,
            ),
        )

    private val routineToApply = translationSysIDRoutine // Change this to test different routines

    val xLimiter: SlewRateLimiter = SlewRateLimiter(0.1)
    val yLimiter: SlewRateLimiter = SlewRateLimiter(0.1)
    val rotLimiter: SlewRateLimiter = SlewRateLimiter(0.1)

    val teleopDriveRequest: SwerveRequest.FieldCentric =
        SwerveRequest.FieldCentric()
            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
            .withDeadband(TunerConstants.kSpeedAt12VoltsMps * 0.1)
            .withRotationalDeadband((1.5 * Math.PI) * 0.1)

    private val autoRequest: SwerveRequest.ApplyChassisSpeeds =
        SwerveRequest.ApplyChassisSpeeds()
            .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
            .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo)

    val chassisSpeeds: ChassisSpeeds
        get() = m_kinematics.toChassisSpeeds(*state.ModuleStates)

    constructor(
        driveTrainConstants: SwerveDrivetrainConstants?,
        OdometryUpdateFrequency: Double,
        vararg modules: SwerveModuleConstants?,
    ) : super(driveTrainConstants, OdometryUpdateFrequency, *modules) {
        init()
    }

    private fun init(){
        if (Utils.isSimulation()) {
            startSimThread()
        }
        configurePathPlanner()

        this.Modules.forEach {
            Shuffleboard.getTab("Swerve").addDouble(
                "Module ${it.driveMotor.deviceID}"
            ) { it.driveMotor.motorVoltage.valueAsDouble }
        }
    }

    private fun configurePathPlanner() {
        var driveBaseRadius = 0.0
        for (moduleLocation in m_moduleLocations) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.norm)
        }

        AutoBuilder.configureHolonomic(
            { this.state.Pose },
            this::seedFieldRelative,
            this::chassisSpeeds,
            { speeds -> this.setControl(autoRequest.withSpeeds(speeds)) },
            HolonomicPathFollowerConfig(
                PIDConstants(10.0, 0.0, 0.0),
                PIDConstants(10.0, 0.0, 0.0),
                TunerConstants.kSpeedAt12VoltsMps,
                driveBaseRadius,
                ReplanningConfig(true, true),
            ),
            {
                DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) ==
                    DriverStation.Alliance.Red
            },
            this,
        )
    }

    constructor(
        driveTrainConstants: SwerveDrivetrainConstants?,
        vararg modules: SwerveModuleConstants?,
    ) : super(
        driveTrainConstants,
        *modules,
    ) {
        init()
    }

    fun applyRequest(requestSupplier: Supplier<SwerveRequest?>): Command {
        return run { this.setControl(requestSupplier.get()) }
    }

    private fun startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds()

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = Notifier {
            val currentTime = Utils.getCurrentTimeSeconds()
            val deltaTime = currentTime - m_lastSimTime
            m_lastSimTime = currentTime

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage())
        }
        m_simNotifier!!.startPeriodic(kSimLoopPeriod)
    }

    fun sysIdQuasistatic(direction: SysIdRoutine.Direction): Command {
        return routineToApply.quasistatic(direction)
    }

    fun sysIdDynamic(direction: SysIdRoutine.Direction): Command {
        return routineToApply.dynamic(direction)
    }

    fun teleopDriveCommand(
        xVelocity: DoubleSupplier,
        yVelocity: DoubleSupplier,
        rotationalRate: DoubleSupplier,
    ): Command {
        return this.run {
            this.setControl(
                teleopDriveRequest
                    .withVelocityX(
                        (xVelocity.asDouble) * TunerConstants.kSpeedAt12VoltsMps,
                    )
                    .withVelocityY(
                        (yVelocity.asDouble) * TunerConstants.kSpeedAt12VoltsMps,
                    )
                    .withRotationalRate(
                        (rotationalRate.asDouble) *
                            TunerConstants.kSpeedAt12VoltsMps,
                    ),
            )
        }
    }

    override fun periodic() {}

    companion object {
        private const val kSimLoopPeriod = 0.005 // 5 ms
    }
}
