package frc.robot

import com.ctre.phoenix6.SignalLogger
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.button.Trigger
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import frc.robot.Constants.OperatorConstants
import frc.robot.commands.Autos
import frc.robot.subsystems.swerve.Drivetrain
import frc.robot.subsystems.swerve.SwerveTelemetry
import frc.robot.subsystems.swerve.TunerConstants

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the [Robot]
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 *
 * In Kotlin, it is recommended that all your Subsystems are Kotlin objects. As such, there can only
 * ever be a single instance. This eliminates the need to create reference variables to the various
 * subsystems in this container to pass into to commands. The commands can just directly reference
 * the (single instance of the) object.
 */
object RobotContainer {

    private val driverController = CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT)

    val drivetrain: Drivetrain = TunerConstants.drivetrain
    private val telemetry: SwerveTelemetry = SwerveTelemetry()

    //    private val logger: SwerveLogger = SwerveLogger()

    init {
        configureBindings()
        // Reference the Autos object so that it is initialized, placing the chooser on the dashboard
        Autos
    }

    // Replace with CommandPS4Controller or CommandJoystick if needed

    /**
     * Use this method to define your `trigger->command` mappings. Triggers can be created via the
     * [Trigger] constructor that takes a [BooleanSupplier][java.util.function.BooleanSupplier] with
     * an arbitrary predicate, or via the named factories in
     * [GenericHID][edu.wpi.first.wpilibj2.command.button.CommandGenericHID] subclasses such for
     * [Xbox][CommandXboxController]/[PS4][edu.wpi.first.wpilibj2.command.button.CommandPS4Controller]
     * controllers or [Flight joysticks][edu.wpi.first.wpilibj2.command.button.CommandJoystick].
     */
    private fun configureBindings() {
        drivetrain.defaultCommand =
            drivetrain.teleopDriveCommand(
                { -driverController.leftY },
                { -driverController.leftX },
                { -driverController.rightX },
            )

//        driverController.b().whileTrue(
//            Commands.sequence(
//                drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward),
//                Commands.waitSeconds(1.0),
//                drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse),
//                Commands.waitSeconds(1.0),
//                drivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward),
//                Commands.waitSeconds(1.0),
//                drivetrain.sysIdDynamic(SysIdRoutine.Direction.kReverse),
//                Commands.waitSeconds(1.0),
//            )
//        )

        driverController.leftBumper().onTrue(Commands.runOnce(SignalLogger::start))
        driverController.rightBumper().onTrue(Commands.runOnce(SignalLogger::stop))

        drivetrain.registerTelemetry { state -> telemetry.telemetrize(state) }
    }
}
