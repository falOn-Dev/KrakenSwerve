package frc.robot.commands

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.PrintCommand
import frc.robot.RobotContainer
import frc.robot.commands.auto.ChoreoAuto
import frc.robot.subsystems.ExampleSubsystem
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser
import java.util.function.Supplier

object Autos {
    private fun waitPrint(msg: String, wait: Double): Command {
        return Commands.parallel(
            Commands.print(msg),
            Commands.waitSeconds(wait),
        )
    }

    private val autoModeChooser =
        SendableChooser<AutoMode>().apply {
            AutoMode.values().forEach { addOption(it.optionName, it) }
            setDefaultOption(AutoMode.default.optionName, AutoMode.default)
        }

    private val loggedAutoChooser: LoggedDashboardChooser<AutoMode> =
        LoggedDashboardChooser("Auto Mode", autoModeChooser)

    val defaultAutonomousCommand: ChoreoAuto
        get() = AutoMode.default.command.get()

    val selectedAutonomousCommand: ChoreoAuto
        get() = autoModeChooser.selected.command.get() ?: defaultAutonomousCommand

    /** Example static factory for an autonomous command. */
    private fun exampleAuto(): Command =
        Commands.sequence(ExampleSubsystem.exampleMethodCommand(), ExampleCommand())

    private fun exampleAuto2() = PrintCommand("An example Auto Mode that just prints a value")

    val basic3note: ChoreoAuto = ChoreoAuto(
        "3NoteAuto",
        RobotContainer.drivetrain,
        sequentialEventMap = mapOf(
            0 to Supplier {
                waitPrint("Shoot Stored Note 1", 2.0)
            },
            1 to Supplier {
                waitPrint("Shoot Stored Note 2", 2.0)
            },
            2 to Supplier {
                waitPrint("Shoot Stored Note 3", 2.0)
            },
        ),
        parallelEventMap = mapOf(
            0 to Supplier { PrintCommand("Picking Up Note 1") },
            1 to Supplier { PrintCommand("Picking Up Note 2") },
            2 to Supplier { PrintCommand("Picking Up Note 3") },
        ),
        startCommand = Supplier { waitPrint("Shooting Stored Note", 2.0) },
    )

    /**
     * An enumeration of the available autonomous modes. It provides an easy way to manage all our
     * autonomous modes. The [autoModeChooser] iterates over its values, adding each value to the
     * chooser.
     *
     * @param optionName The name for the [autoModeChooser] option.
     * @param command The [Command] to run for this mode.
     */
    @Suppress("unused")
    private enum class AutoMode(val optionName: String, val command: Supplier<ChoreoAuto>) {
        // TODO: Replace with real auto modes and their corresponding commands
        CUSTOM_AUTO_1("Custom Auto Mode 1", { basic3note }),
        ;

        companion object {
            /** The default auto mode. */
            val default = CUSTOM_AUTO_1
        }
    }
}
