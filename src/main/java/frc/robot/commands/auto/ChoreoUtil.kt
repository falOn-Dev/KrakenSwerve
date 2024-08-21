package lib.auto

import com.choreo.lib.Choreo
import com.choreo.lib.ChoreoTrajectory
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import frc.robot.subsystems.Drivetrain

fun getPath(traj: ChoreoTrajectory, isRed: Boolean, drivebase: Drivetrain, parallel: Command = InstantCommand()): Command {
    return Choreo.choreoSwerveCommand(
        traj,
        drivebase::pose,
        PIDController(0.7, 0.0, 0.0),
        PIDController(0.7, 0.0, 0.0),
        PIDController(0.4, 0.0, 0.01),
        drivebase::applyChassisSpeeds,
        { isRed },
        drivebase,
    ).alongWith(parallel)
}
