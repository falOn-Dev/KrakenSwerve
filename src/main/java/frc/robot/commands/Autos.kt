package frc.robot.commands

import com.choreo.lib.Choreo
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.PrintCommand
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.robot.Constants
import frc.robot.RobotContainer
import frc.robot.commands.auto.getPath
import frc.robot.subsystems.ExampleSubsystem
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser

object Autos {
    private val autoModeChooser =
        SendableChooser<AutoMode>().apply {
            AutoMode.values().forEach { addOption(it.optionName, it) }
            setDefaultOption(AutoMode.default.optionName, AutoMode.default)
        }

    private val loggedAutoChooser: LoggedDashboardChooser<AutoMode> =
        LoggedDashboardChooser("Auto Mode", autoModeChooser)

    val defaultAutonomousCommand: Command
        get() = AutoMode.default.command

    val selectedAutonomousCommand: Command
        get() = autoModeChooser.selected?.command ?: defaultAutonomousCommand

    /** Example static factory for an autonomous command. */
    private fun exampleAuto(): Command =
        Commands.sequence(ExampleSubsystem.exampleMethodCommand(), ExampleCommand())

    private fun exampleAuto2() = PrintCommand("An example Auto Mode that just prints a value")

    private fun test3Note(): Command {
        val paths = Choreo.getTrajectoryGroup("test_note_solution")

        val group = SequentialCommandGroup()

        group.addCommands(
            getPath(paths[0], false, drivebase = RobotContainer.drivetrain),
        )
    }

    /**
     * An enumeration of the available autonomous modes. It provides an easy way to manage all our
     * autonomous modes. The [autoModeChooser] iterates over its values, adding each value to the
     * chooser.
     *
     * @param optionName The name for the [autoModeChooser] option.
     * @param command The [Command] to run for this mode.
     */
    @Suppress("unused")
    private enum class AutoMode(val optionName: String, val command: Command) {
        // TODO: Replace with real auto modes and their corresponding commands
        CUSTOM_AUTO_1("Custom Auto Mode 1", exampleAuto()),
        CUSTOM_AUTO_2("Custom Auto Mode 2", exampleAuto2()),
        CUSTOM_AUTO_3("Custom Auto Mode 3", ExampleCommand()),
        ;

        companion object {
            /** The default auto mode. */
            val default = CUSTOM_AUTO_1
        }
    }
}
