package frc.robot.commands.swerve

import com.fasterxml.jackson.databind.ObjectMapper
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj.Filesystem
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.robot.subsystems.swerve.Drivetrain
import lib.MotionCommand
import lib.RobotMotion
import java.io.File

class FollowMotion(
    private val instructions: String,
    private val swerve: Drivetrain
) : SequentialCommandGroup() {
    private val instructionFile = File(Filesystem.getDeployDirectory(), "motion/$instructions.json")
    private val json = ObjectMapper()
    private val motion = json.readValue(instructionFile, RobotMotion::class.java)

    init {
        motion.commands.forEach {
            addCommands(SimpleMoveCommand(it, swerve))
        }
    }


}