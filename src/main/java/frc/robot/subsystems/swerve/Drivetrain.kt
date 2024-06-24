package frc.robot.subsystems.swerve

import com.ctre.phoenix6.SignalLogger
import com.ctre.phoenix6.Utils
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units
import edu.wpi.first.units.Voltage
import edu.wpi.first.wpilibj.Notifier
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Subsystem
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import java.util.function.Supplier

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem so it can be used in command-based projects easily.
 */
class Drivetrain : SwerveDrivetrain, Subsystem {
    private var m_simNotifier: Notifier? = null
    private var m_lastSimTime = 0.0

    private val rotationSysIDReq: SwerveRequest.SysIdSwerveRotation = SwerveRequest.SysIdSwerveRotation()
    private val steerSysIDReq: SwerveRequest.SysIdSwerveSteerGains = SwerveRequest.SysIdSwerveSteerGains()
    private val translationSysIDReq: SwerveRequest.SysIdSwerveTranslation = SwerveRequest.SysIdSwerveTranslation()

    private val rotationSysIDRoutine: SysIdRoutine = SysIdRoutine(
        SysIdRoutine.Config(
            null,
            Units.Volts.of(4.0),
            null,
            { state -> SignalLogger.writeString("state", state.toString()) }
        ),
        SysIdRoutine.Mechanism(
            { volts: Measure<Voltage> -> setControl(rotationSysIDReq.withVolts(volts))},
            null,
            this
        )
    )

    private val steerSysIDRoutine: SysIdRoutine = SysIdRoutine(
        SysIdRoutine.Config(
            null,
            Units.Volts.of(7.0),
            null,
            { state -> SignalLogger.writeString("state", state.toString()) }
        ),
        SysIdRoutine.Mechanism(
            { volts: Measure<Voltage> -> setControl(steerSysIDReq.withVolts(volts))},
            null,
            this
        )
    )

    private val translationSysIDRoutine: SysIdRoutine = SysIdRoutine(
        SysIdRoutine.Config(
            null,
            Units.Volts.of(4.0),
            null,
            { state -> SignalLogger.writeString("state", state.toString()) }
        ),
        SysIdRoutine.Mechanism(
            { volts: Measure<Voltage> -> setControl(translationSysIDReq.withVolts(volts))},
            null,
            this
        )
    )

    private val routineToApply = rotationSysIDRoutine // Change this to test different routines


    constructor(
        driveTrainConstants: SwerveDrivetrainConstants?,
        OdometryUpdateFrequency: Double,
        vararg modules: SwerveModuleConstants?
    ) : super(driveTrainConstants, OdometryUpdateFrequency, *modules) {
        if (Utils.isSimulation()) {
            startSimThread()
        }
    }

    constructor(driveTrainConstants: SwerveDrivetrainConstants?, vararg modules: SwerveModuleConstants?) : super(
        driveTrainConstants,
        *modules
    ) {
        if (Utils.isSimulation()) {
            startSimThread()
        }
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

    fun sysIdDynamic(direction: SysIdRoutine.Direction): Command? {
        return routineToApply.dynamic(direction)
    }

    override fun periodic() {}

    companion object {
        private const val kSimLoopPeriod = 0.005 // 5 ms
    }
}